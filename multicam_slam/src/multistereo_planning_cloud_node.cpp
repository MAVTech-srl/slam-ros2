// multistereo_planning_cloud_node.cpp (extended to up to 8 cameras = 4 stereo pairs)
//
// Changes vs previous version:
// - Supports up to 8 cameras (4 stereo pairs).
// - Default uses 6 cameras (3 stereo pairs) via parameter num_cameras=6.
// - Calibration JSON can contain 6 or 8 cameras; node uses the first num_cameras entries.
// - Topics for stereo4 (cam6/cam7) are optional unless num_cameras==8.
// - Publishes ONE PointCloud2 in IMU frame (imu/base_link).
//
// Assumptions remain:
// - Images already rectified.
// - JSON order: (s1L,s1R,s2L,s2R,s3L,s3R,s4L,s4R) if 8 cams.
// - T_imu_cam is IMU->CAM (confirmed by you); we invert to transform CAM->IMU.
//
// Dependencies:
//   rclcpp, rclcpp_components, sensor_msgs, cv_bridge, OpenCV, Eigen3, nlohmann-json3-dev

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

#include <fstream>
#include <string>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace multicam_depth
{

struct PinholeIntrinsics
{
  double fx = 0, fy = 0, cx = 0, cy = 0;
  int width = 0, height = 0;
};

struct SE3
{
  Eigen::Quaterniond q;
  Eigen::Vector3d t;

  Eigen::Matrix3d R() const { return q.toRotationMatrix(); }

  SE3 inverse() const
  {
    SE3 inv;
    inv.q = q.conjugate();
    inv.t = -(inv.R() * t);
    return inv;
  }

  Eigen::Vector3d transform(const Eigen::Vector3d& p) const
  {
    return R() * p + t;
  }
};

static SE3 se3FromImuToCamEntry(const json& j)
{
  SE3 T;
  T.t = Eigen::Vector3d(j.at("px").get<double>(),
                        j.at("py").get<double>(),
                        j.at("pz").get<double>());
  // JSON: qx,qy,qz,qw
  T.q = Eigen::Quaterniond(j.at("qw").get<double>(),
                          j.at("qx").get<double>(),
                          j.at("qy").get<double>(),
                          j.at("qz").get<double>());
  T.q.normalize();
  return T;
}

static PinholeIntrinsics pinholeFromEntry(const json& intr_j, const std::array<int,2>& res)
{
  PinholeIntrinsics K;
  K.fx = intr_j.at("fx").get<double>();
  K.fy = intr_j.at("fy").get<double>();
  K.cx = intr_j.at("cx").get<double>();
  K.cy = intr_j.at("cy").get<double>();
  K.width = res[0];
  K.height = res[1];
  return K;
}

// Simple voxel centroid accumulator
struct VoxelAcc
{
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  int count = 0;
};

static inline int64_t packKey(int ix, int iy, int iz)
{
  constexpr int64_t OFF = (1LL << 20);
  int64_t x = (int64_t)(ix + OFF) & ((1LL << 21) - 1);
  int64_t y = (int64_t)(iy + OFF) & ((1LL << 21) - 1);
  int64_t z = (int64_t)(iz + OFF) & ((1LL << 21) - 1);
  return (x << 42) | (y << 21) | z;
}

class MultiStereoPlanningCloudNode : public rclcpp::Node
{
public:
  explicit MultiStereoPlanningCloudNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("multistereo_planning_cloud_node", options)
  {
    // ---- Parameters
    calib_json_path_   = this->declare_parameter<std::string>("calib_json_path", "");
    calib_json_string_ = this->declare_parameter<std::string>("calib_json_string", "");

    imu_frame_    = this->declare_parameter<std::string>("imu_frame", "imu/base_link");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "planning/cloud");

    // Up to 8 cameras (default 6)
    num_cameras_ = this->declare_parameter<int>("num_cameras", 6);
    if (num_cameras_ != 6 && num_cameras_ != 8) {
      RCLCPP_WARN(this->get_logger(), "num_cameras should be 6 or 8. Forcing to 6.");
      num_cameras_ = 6;
    }
    num_stereo_pairs_ = num_cameras_ / 2; // 3 or 4

    // Topics: by stereo pair (left/right)
    // Defaults: first 3 pairs match your bag names; stereo4 left/right are configurable if you have 8 cams.
    topic_left_[0]  = this->declare_parameter<std::string>("stereo1_left_topic",  "left/image_rect_raw");
    topic_right_[0] = this->declare_parameter<std::string>("stereo1_right_topic", "right/image_rect_raw");
    topic_left_[1]  = this->declare_parameter<std::string>("stereo2_left_topic",  "cam2/image_rect_raw");
    topic_right_[1] = this->declare_parameter<std::string>("stereo2_right_topic", "cam3/image_rect_raw");
    topic_left_[2]  = this->declare_parameter<std::string>("stereo3_left_topic",  "cam4/image_rect_raw");
    topic_right_[2] = this->declare_parameter<std::string>("stereo3_right_topic", "cam5/image_rect_raw");
    topic_left_[3]  = this->declare_parameter<std::string>("stereo4_left_topic",  "cam6/image_rect_raw");
    topic_right_[3] = this->declare_parameter<std::string>("stereo4_right_topic", "cam7/image_rect_raw");

    // Output cadence
    output_rate_hz_ = this->declare_parameter<double>("output_rate_hz", 10.0);
    max_stamp_skew_ms_ = this->declare_parameter<double>("max_stamp_skew_ms", 5.0);

    // Depth generation / sampling
    downscale_  = this->declare_parameter<double>("image_downscale", 1.0);
    pixel_step_ = this->declare_parameter<int>("pixel_step", 4);
    depth_min_  = this->declare_parameter<double>("depth_min_m", 0.30);
    depth_max_  = this->declare_parameter<double>("depth_max_m", 10.0);

    // Planning voxel filter
    voxel_leaf_ = this->declare_parameter<double>("voxel_leaf_m", 0.08);

    // SGBM params
    sgbm_min_disp_   = this->declare_parameter<int>("sgbm_min_disparity", 0);
    sgbm_num_disp_   = this->declare_parameter<int>("sgbm_num_disparities", 128); // multiple of 16
    sgbm_block_sz_   = this->declare_parameter<int>("sgbm_block_size", 5);
    sgbm_P1_         = this->declare_parameter<int>("sgbm_P1", 8 * sgbm_block_sz_ * sgbm_block_sz_);
    sgbm_P2_         = this->declare_parameter<int>("sgbm_P2", 32 * sgbm_block_sz_ * sgbm_block_sz_);
    sgbm_disp12_     = this->declare_parameter<int>("sgbm_disp12_max_diff", 1);
    sgbm_uniqueness_ = this->declare_parameter<int>("sgbm_uniqueness_ratio", 10);
    sgbm_speckle_ws_ = this->declare_parameter<int>("sgbm_speckle_window_size", 50);
    sgbm_speckle_rg_ = this->declare_parameter<int>("sgbm_speckle_range", 2);
    sgbm_mode_       = this->declare_parameter<int>("sgbm_mode", (int)cv::StereoSGBM::MODE_SGBM_3WAY);

    // ---- Load calibration JSON (>= num_cameras entries)
    loadCalibrationOrThrow_();

    // ---- Create SGBM
    sgbm_ = cv::StereoSGBM::create(
      sgbm_min_disp_, sgbm_num_disp_, sgbm_block_sz_,
      sgbm_P1_, sgbm_P2_, sgbm_disp12_, 0,
      sgbm_uniqueness_, sgbm_speckle_ws_, sgbm_speckle_rg_, sgbm_mode_);

    // ---- Subs/Pubs
    auto qos_img = rclcpp::SensorDataQoS();

    for (int s = 0; s < num_stereo_pairs_; ++s) {
      sub_left_[s] = this->create_subscription<sensor_msgs::msg::Image>(
        topic_left_[s], qos_img,
        [this, s](sensor_msgs::msg::Image::ConstSharedPtr msg){ onImage_(s, true, msg); });

      sub_right_[s] = this->create_subscription<sensor_msgs::msg::Image>(
        topic_right_[s], qos_img,
        [this, s](sensor_msgs::msg::Image::ConstSharedPtr msg){ onImage_(s, false, msg); });

      RCLCPP_INFO(this->get_logger(), "Stereo%d topics: L=%s R=%s",
        s+1, topic_left_[s].c_str(), topic_right_[s].c_str());
    }

    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::QoS(10));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, output_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MultiStereoPlanningCloudNode::onTimer_, this));

    RCLCPP_INFO(this->get_logger(),
      "Started. num_cameras=%d num_stereo_pairs=%d output=%s@%.2fHz frame=%s",
      num_cameras_, num_stereo_pairs_, output_topic_.c_str(), output_rate_hz_, imu_frame_.c_str());
  }

private:
  static constexpr int kMaxCams   = 8;
  static constexpr int kMaxStereo = 4;

  // ---- Calibration (use first num_cameras entries)
  std::array<PinholeIntrinsics, kMaxCams> K_{};
  std::array<SE3, kMaxCams> T_imu_cam_{};
  std::array<SE3, kMaxCams> T_cam_imu_{};
  std::array<double, kMaxStereo> baseline_m_{};

  int num_cameras_ = 6;
  int num_stereo_pairs_ = 3;

  void loadCalibrationOrThrow_()
  {
    json root;
    if (!calib_json_string_.empty()) {
      root = json::parse(calib_json_string_);
    } else if (!calib_json_path_.empty()) {
      std::ifstream ifs(calib_json_path_);
      if (!ifs.is_open()) {
        throw std::runtime_error("Cannot open calib_json_path: " + calib_json_path_);
      }
      ifs >> root;
    } else {
      throw std::runtime_error("Provide either calib_json_path or calib_json_string");
    }

    const auto& v0    = root.at("value0");
    const auto& Tlist = v0.at("T_imu_cam");
    const auto& intr  = v0.at("intrinsics");
    const auto& res   = v0.at("resolution");

    const int available = static_cast<int>(std::min({Tlist.size(), intr.size(), res.size()}));
    if (available < num_cameras_) {
      throw std::runtime_error("Calibration JSON has only " + std::to_string(available) +
                               " cameras but num_cameras=" + std::to_string(num_cameras_));
    }

    for (int i = 0; i < num_cameras_; ++i) {
      T_imu_cam_[i] = se3FromImuToCamEntry(Tlist.at(i));
      T_cam_imu_[i] = T_imu_cam_[i].inverse();

      std::array<int,2> r = {res.at(i).at(0).get<int>(), res.at(i).at(1).get<int>()};
      const auto& intr_i = intr.at(i).at("intrinsics");
      K_[i] = pinholeFromEntry(intr_i, r);
    }

    // Baselines for each stereo pair (0,1), (2,3), (4,5), (6,7)
    for (int s = 0; s < num_stereo_pairs_; ++s) {
      computeBaselineForPair_(s);
    }

    std::string bl = "";
    for (int s = 0; s < num_stereo_pairs_; ++s) {
      bl += " s" + std::to_string(s+1) + "=" + std::to_string(baseline_m_[s]);
    }
    RCLCPP_INFO(this->get_logger(), "Loaded calibration (%d cams). Baselines:%s",
      num_cameras_, bl.c_str());
  }

  void computeBaselineForPair_(int stereo_id)
  {
    const int camL = 2 * stereo_id;
    const int camR = 2 * stereo_id + 1;

    // T_camL_camR = (camL->imu) * (imu->camR)
    SE3 T_camL_camR;
    // Compose in SE3 terms:
    // We don't have operator* for SE3 (only transform), so do it manually:
    // T_camL_camR.R = R_camL_imu * R_imu_camR = R_camL_imu * R_imu_camR
    // T_camL_camR.t = t_camL_imu + R_camL_imu * t_imu_camR
    const Eigen::Matrix3d R_camL_imu = T_cam_imu_[camL].R();
    const Eigen::Matrix3d R_imu_camR = T_imu_cam_[camR].R();

    T_camL_camR.q = Eigen::Quaterniond(R_camL_imu * R_imu_camR);
    T_camL_camR.q.normalize();
    T_camL_camR.t = T_cam_imu_[camL].t + R_camL_imu * T_imu_cam_[camR].t;

    const double tx = T_camL_camR.t.x();
    const double tn = T_camL_camR.t.norm();
    baseline_m_[stereo_id] = (std::abs(tx) > 1e-4) ? std::abs(tx) : tn;
  }

  // ---- Image buffering
  struct StereoBuffer
  {
    sensor_msgs::msg::Image::ConstSharedPtr left;
    sensor_msgs::msg::Image::ConstSharedPtr right;
  };

  std::mutex mtx_;
  std::array<StereoBuffer, kMaxStereo> stereo_buf_{};

  void onImage_(int stereo_id, bool is_left, const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (is_left) stereo_buf_[stereo_id].left = msg;
    else         stereo_buf_[stereo_id].right = msg;
  }

  static rclcpp::Duration absDiff_(const builtin_interfaces::msg::Time& a,
                                  const builtin_interfaces::msg::Time& b)
  {
    rclcpp::Time ta(a), tb(b);
    return (ta > tb) ? (ta - tb) : (tb - ta);
  }

  void onTimer_()
  {
    std::array<StereoBuffer, kMaxStereo> snap;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      snap = stereo_buf_;
    }

    // Validate we have all images for enabled stereo pairs
    for (int s = 0; s < num_stereo_pairs_; ++s) {
      if (!snap[s].left || !snap[s].right) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Waiting for stereo %d images...", s+1);
        return;
      }
    }

    // Check stamp skew across all enabled images
    const auto t0 = snap[0].left->header.stamp;
    auto max_skew = rclcpp::Duration::from_nanoseconds(0);
    for (int s = 0; s < num_stereo_pairs_; ++s) {
      max_skew = std::max(max_skew, absDiff_(t0, snap[s].left->header.stamp));
      max_skew = std::max(max_skew, absDiff_(t0, snap[s].right->header.stamp));
    }
    if (max_skew.seconds() * 1000.0 > max_stamp_skew_ms_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Stamp skew too large (%.2f ms). Skipping tick.", max_skew.seconds() * 1000.0);
      return;
    }

    // Voxel accum
    std::unordered_map<int64_t, VoxelAcc> vox;
    vox.reserve(250000);

    // Process each stereo pair:
    // stereo_id s uses camL index = 2*s (in JSON order)
    for (int s = 0; s < num_stereo_pairs_; ++s) {
      processStereoPair_(snap[s].left, snap[s].right, /*camL=*/2*s, /*stereo_id=*/s, vox);
    }

    publishVoxelCloud_(vox, t0);
  }

  void processStereoPair_(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                          const sensor_msgs::msg::Image::ConstSharedPtr& right_msg,
                          int cam_left_index,
                          int stereo_id,
                          std::unordered_map<int64_t, VoxelAcc>& vox)
  {
    cv::Mat left  = cv_bridge::toCvShare(left_msg, "mono8")->image;
    cv::Mat right = cv_bridge::toCvShare(right_msg, "mono8")->image;
    if (left.empty() || right.empty()) return;

    // Optional downscale
    const double scale = std::max(1e-6, downscale_);
    cv::Mat l_ds, r_ds;
    if (std::abs(scale - 1.0) > 1e-6) {
      cv::resize(left,  l_ds, cv::Size(), scale, scale, cv::INTER_AREA);
      cv::resize(right, r_ds, cv::Size(), scale, scale, cv::INTER_AREA);
    } else {
      l_ds = left;
      r_ds = right;
    }

    cv::Mat disp16;
    sgbm_->compute(l_ds, r_ds, disp16); // CV_16S, disparity*16

    // Effective intrinsics after scaling
    PinholeIntrinsics K = K_[cam_left_index];
    K.fx *= scale; K.fy *= scale; K.cx *= scale; K.cy *= scale;

    const double baseline = baseline_m_[stereo_id];

    const int rows = disp16.rows;
    const int cols = disp16.cols;

    const SE3& T_cam_imu = T_cam_imu_[cam_left_index]; // CAM->IMU

    const int step = std::max(1, pixel_step_);
    for (int v = 0; v < rows; v += step) {
      const int16_t* dptr = disp16.ptr<int16_t>(v);
      for (int u = 0; u < cols; u += step) {
        const int16_t d_raw = dptr[u];
        if (d_raw <= 0) continue;

        const double disp = static_cast<double>(d_raw) / 16.0;
        if (disp < 0.5) continue; // guard

        const double z = (K.fx * baseline) / disp;
        if (!(z >= depth_min_ && z <= depth_max_)) continue;

        const double x = (static_cast<double>(u) - K.cx) * z / K.fx;
        const double y = (static_cast<double>(v) - K.cy) * z / K.fy;

        Eigen::Vector3d p_cam(x, y, z);
        Eigen::Vector3d p_imu = T_cam_imu.transform(p_cam);

        // voxelize in IMU frame
        const double inv_leaf = 1.0 / std::max(1e-6, voxel_leaf_);
        const int ix = static_cast<int>(std::floor(p_imu.x() * inv_leaf));
        const int iy = static_cast<int>(std::floor(p_imu.y() * inv_leaf));
        const int iz = static_cast<int>(std::floor(p_imu.z() * inv_leaf));
        const int64_t key = packKey(ix, iy, iz);

        auto& acc = vox[key];
        acc.sum += p_imu;
        acc.count += 1;
      }
    }
  }

  void publishVoxelCloud_(const std::unordered_map<int64_t, VoxelAcc>& vox,
                          const builtin_interfaces::msg::Time& stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = imu_frame_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(vox.size());
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(vox.size());

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

    size_t i = 0;
    for (const auto& kv : vox) {
      const auto& acc = kv.second;
      if (acc.count <= 0) continue;

      const Eigen::Vector3d p = acc.sum / static_cast<double>(acc.count);
      *it_x = static_cast<float>(p.x());
      *it_y = static_cast<float>(p.y());
      *it_z = static_cast<float>(p.z());

      ++it_x; ++it_y; ++it_z;
      ++i;
    }

    // shrink if needed
    if (i != vox.size()) {
      cloud.width = static_cast<uint32_t>(i);
      cloud.row_step = cloud.point_step * cloud.width;
      cloud.data.resize(cloud.row_step * cloud.height);
    }

    pub_cloud_->publish(cloud);
  }

private:
  // Params
  std::string calib_json_path_;
  std::string calib_json_string_;
  std::string imu_frame_;
  std::string output_topic_;

  std::array<std::string, kMaxStereo> topic_left_{};
  std::array<std::string, kMaxStereo> topic_right_{};

  double output_rate_hz_ = 10.0;
  double max_stamp_skew_ms_ = 5.0;

  double downscale_ = 1.0;
  int pixel_step_ = 4;
  double depth_min_ = 0.3;
  double depth_max_ = 10.0;
  double voxel_leaf_ = 0.08;

  // SGBM params
  int sgbm_min_disp_ = 0;
  int sgbm_num_disp_ = 128;
  int sgbm_block_sz_ = 5;
  int sgbm_P1_ = 0;
  int sgbm_P2_ = 0;
  int sgbm_disp12_ = 1;
  int sgbm_uniqueness_ = 10;
  int sgbm_speckle_ws_ = 50;
  int sgbm_speckle_rg_ = 2;
  int sgbm_mode_ = (int)cv::StereoSGBM::MODE_SGBM_3WAY;

  // ROS
  std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, kMaxStereo> sub_left_{};
  std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, kMaxStereo> sub_right_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::TimerBase::SharedPtr timer_;

  // OpenCV
  cv::Ptr<cv::StereoSGBM> sgbm_;
};

} // namespace multicam_depth

RCLCPP_COMPONENTS_REGISTER_NODE(multicam_depth::MultiStereoPlanningCloudNode)

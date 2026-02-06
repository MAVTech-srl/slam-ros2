#include "multistereo_planning_cloud/multistereo_planning_cloud_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <fstream>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <sstream>

static inline double stampToSec(const builtin_interfaces::msg::Time& t)
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

static inline std::string fmtStamp(const builtin_interfaces::msg::Time& t)
{
  std::ostringstream oss;
  oss << t.sec << "." << std::setw(9) << std::setfill('0') << t.nanosec;
  return oss.str();
}

static inline double dtMs(const builtin_interfaces::msg::Time& a,
                          const builtin_interfaces::msg::Time& b)
{
  return std::abs(stampToSec(a) - stampToSec(b)) * 1000.0;
}


using json = nlohmann::json;

namespace multicam_depth
{

// ---------- static helpers ----------
SE3 MultiStereoPlanningCloudNode::se3FromImuToCamEntry_(const json& j)
{
  SE3 T;
  T.t = Eigen::Vector3d(j.at("px").get<double>(),
                        j.at("py").get<double>(),
                        j.at("pz").get<double>());
  T.q = Eigen::Quaterniond(j.at("qw").get<double>(),
                          j.at("qx").get<double>(),
                          j.at("qy").get<double>(),
                          j.at("qz").get<double>());
  T.q.normalize();
  return T;
}

PinholeIntrinsics MultiStereoPlanningCloudNode::pinholeFromEntry_(const json& intr_j,
                                                                  const std::array<int,2>& res)
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

int64_t MultiStereoPlanningCloudNode::packKey_(int ix, int iy, int iz)
{
  constexpr int64_t OFF = (1LL << 20);
  int64_t x = (int64_t)(ix + OFF) & ((1LL << 21) - 1);
  int64_t y = (int64_t)(iy + OFF) & ((1LL << 21) - 1);
  int64_t z = (int64_t)(iz + OFF) & ((1LL << 21) - 1);
  return (x << 42) | (y << 21) | z;
}

rclcpp::Duration MultiStereoPlanningCloudNode::absDiff_(const builtin_interfaces::msg::Time& a,
                                                       const builtin_interfaces::msg::Time& b)
{
  rclcpp::Time ta(a), tb(b);
  return (ta > tb) ? (ta - tb) : (tb - ta);
}

// ---------- ctor ----------
MultiStereoPlanningCloudNode::MultiStereoPlanningCloudNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("multistereo_planning_cloud_node", options)
{
  auto config_dir = this->declare_parameter<std::string>("config_dir", "");
  auto calib_rel  = this->declare_parameter<std::string>("config", "6cams_triangle.json");
  calib_json_path_   = config_dir.empty() ? calib_rel : (config_dir + "/" + calib_rel);
  calib_json_string_ = this->declare_parameter<std::string>("calib_json_string", "");


  imu_frame_    = this->declare_parameter<std::string>("imu_frame", "cam/imu_link");
  output_topic_ = this->declare_parameter<std::string>("output_topic", "planning/cloud");
  stale_timeout_ms_ = this->declare_parameter<double>("stale_timeout_ms", stale_timeout_ms_);

  num_cameras_ = this->declare_parameter<int>("num_cameras", 6);
  if (num_cameras_ != 6 && num_cameras_ != 8) {
    RCLCPP_WARN(this->get_logger(), "num_cameras should be 6 or 8. Forcing to 6.");
    num_cameras_ = 6;
  }
  num_stereo_pairs_ = num_cameras_ / 2;

  topic_left_[0]  = this->declare_parameter<std::string>("stereo1_left_topic",  "left/image_rect_raw");
  topic_right_[0] = this->declare_parameter<std::string>("stereo1_right_topic", "right/image_rect_raw");
  topic_left_[1]  = this->declare_parameter<std::string>("stereo2_left_topic",  "cam2/image_rect_raw");
  topic_right_[1] = this->declare_parameter<std::string>("stereo2_right_topic", "cam3/image_rect_raw");
  topic_left_[2]  = this->declare_parameter<std::string>("stereo3_left_topic",  "cam4/image_rect_raw");
  topic_right_[2] = this->declare_parameter<std::string>("stereo3_right_topic", "cam5/image_rect_raw");
  topic_left_[3]  = this->declare_parameter<std::string>("stereo4_left_topic",  "cam6/image_rect_raw");
  topic_right_[3] = this->declare_parameter<std::string>("stereo4_right_topic", "cam7/image_rect_raw");
  publish_debug_disparity_ = this->declare_parameter<bool>("publish_debug_disparity", false);
  disparity_topic_prefix_  = this->declare_parameter<std::string>("disparity_topic_prefix", "debug/disparity");


  output_rate_hz_ = this->declare_parameter<double>("output_rate_hz", 10.0);
  max_stamp_skew_ms_ = this->declare_parameter<double>("max_stamp_skew_ms", 5.0);

  disp_min_px_ = this->declare_parameter<double>("disp_min_px", 4.0);
  disp_local_thr_px_ = this->declare_parameter<double>("disp_local_thr_px", 1.5);
  disp_local_support_ = this->declare_parameter<int>("disp_local_support", 2);


  downscale_  = this->declare_parameter<double>("image_downscale", 1.0);
  pixel_step_ = this->declare_parameter<int>("pixel_step", 4);
  depth_min_  = this->declare_parameter<double>("depth_min_m", 0.30);
  depth_max_  = this->declare_parameter<double>("depth_max_m", 10.0);

  voxel_leaf_ = this->declare_parameter<double>("voxel_leaf_m", 0.08);

  sgbm_min_disp_   = this->declare_parameter<int>("sgbm_min_disparity", 0);
  sgbm_num_disp_   = this->declare_parameter<int>("sgbm_num_disparities", 128);
  sgbm_block_sz_   = this->declare_parameter<int>("sgbm_block_size", 5);
  sgbm_P1_         = this->declare_parameter<int>("sgbm_P1", 8 * sgbm_block_sz_ * sgbm_block_sz_);
  sgbm_P2_         = this->declare_parameter<int>("sgbm_P2", 32 * sgbm_block_sz_ * sgbm_block_sz_);
  sgbm_disp12_     = this->declare_parameter<int>("sgbm_disp12_max_diff", 1);
  sgbm_uniqueness_ = this->declare_parameter<int>("sgbm_uniqueness_ratio", 10);
  sgbm_speckle_ws_ = this->declare_parameter<int>("sgbm_speckle_window_size", 50);
  sgbm_speckle_rg_ = this->declare_parameter<int>("sgbm_speckle_range", 2);
  sgbm_mode_       = this->declare_parameter<int>("sgbm_mode", (int)cv::StereoSGBM::MODE_SGBM_3WAY);

  RCLCPP_INFO(this->get_logger(), "Calibration: path='%s' string_len=%zu",
            calib_json_path_.c_str(), calib_json_string_.size());

  loadCalibrationOrThrow_();

  sgbm_ = cv::StereoSGBM::create(
    sgbm_min_disp_, sgbm_num_disp_, sgbm_block_sz_,
    sgbm_P1_, sgbm_P2_, sgbm_disp12_, 0,
    sgbm_uniqueness_, sgbm_speckle_ws_, sgbm_speckle_rg_, sgbm_mode_);

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
  if (publish_debug_disparity_) {
    for (int s = 0; s < num_stereo_pairs_; ++s) {
      const std::string topic = disparity_topic_prefix_ + "/stereo" + std::to_string(s + 1);
      pub_disp_[s] = this->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(1));
      RCLCPP_INFO(this->get_logger(), "Publishing disparity debug: %s", topic.c_str());
    }
  }


  const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, output_rate_hz_));
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&MultiStereoPlanningCloudNode::onTimer_, this));

  RCLCPP_INFO(this->get_logger(),
    "Started. num_cameras=%d num_stereo_pairs=%d output=%s@%.2fHz frame=%s",
    num_cameras_, num_stereo_pairs_, output_topic_.c_str(), output_rate_hz_, imu_frame_.c_str());
}

// ---------- calib ----------
void MultiStereoPlanningCloudNode::loadCalibrationOrThrow_()
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
    T_imu_cam_[i] = se3FromImuToCamEntry_(Tlist.at(i));
    T_cam_imu_[i] = T_imu_cam_[i].inverse();

    std::array<int,2> r = {res.at(i).at(0).get<int>(), res.at(i).at(1).get<int>()};
    const auto& intr_i = intr.at(i).at("intrinsics");
    K_[i] = pinholeFromEntry_(intr_i, r);
  }

  for (int s = 0; s < num_stereo_pairs_; ++s) {
    computeBaselineForPair_(s);
  }

  std::string bl;
  for (int s = 0; s < num_stereo_pairs_; ++s) {
    bl += " s" + std::to_string(s+1) + "=" + std::to_string(baseline_m_[s]);
  }
  RCLCPP_INFO(this->get_logger(), "Loaded calibration (%d cams). Baselines:%s",
              num_cameras_, bl.c_str());
}

void MultiStereoPlanningCloudNode::computeBaselineForPair_(int stereo_id)
{
  const int camL = 2 * stereo_id;
  const int camR = 2 * stereo_id + 1;

  // T_camL_camR = (camL->imu) * (imu->camR)
  const Eigen::Matrix3d R_camL_imu = T_cam_imu_[camL].R();
  //const Eigen::Matrix3d R_imu_camR = T_imu_cam_[camR].R();

  Eigen::Vector3d t = T_cam_imu_[camL].t + R_camL_imu * T_imu_cam_[camR].t;

  const double tx = t.x();
  const double tn = t.norm();
  baseline_m_[stereo_id] = (std::abs(tx) > 1e-4) ? std::abs(tx) : tn;
}

// ---------- callbacks ----------
void MultiStereoPlanningCloudNode::onImage_(int stereo_id, bool is_left,
                                           const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  const rclcpp::Time now_steady = steady_clock.now();

  std::lock_guard<std::mutex> lk(mtx_);
  if (is_left) {
    stereo_buf_[stereo_id].left = msg;
    last_rx_wall_time_left_[stereo_id] = now_steady;
  } else {
    stereo_buf_[stereo_id].right = msg;
    last_rx_wall_time_right_[stereo_id] = now_steady;
  }
}


void MultiStereoPlanningCloudNode::onTimer_()
{
  // Usa un clock monotonic (non ROS-time), così "stale" funziona anche se rosbag stoppa o /clock sparisce
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  const rclcpp::Time now_steady = steady_clock.now();

  // Snapshot veloce dei buffer + rx times
  std::array<StereoBuffer, kMaxStereo> snap;
  std::array<rclcpp::Time, kMaxStereo> rxL, rxR;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    snap = stereo_buf_;
    rxL  = last_rx_wall_time_left_;
    rxR  = last_rx_wall_time_right_;
  }

  // 1) Missing data?
  for (int s = 0; s < num_stereo_pairs_; ++s) {
    if (!snap[s].left || !snap[s].right) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting stereo%d images... haveL=%d haveR=%d (L='%s' R='%s')",
        s + 1, snap[s].left ? 1 : 0, snap[s].right ? 1 : 0,
        topic_left_[s].c_str(), topic_right_[s].c_str());
      return;
    }
  }

  // 2) Stale timeout: se il bag si ferma, i buffer restano "pieni" e tu riprocessi sempre.
  // Qui invece: se non arrivano nuovi frame per stale_timeout_ms_ -> clear e stop.
  for (int s = 0; s < num_stereo_pairs_; ++s) {
    // se rxL/rxR non sono mai stati settati (stamp 0), evita stale falso positivo
    if (rxL[s].nanoseconds() == 0 || rxR[s].nanoseconds() == 0) continue;

    const double ageL_ms = (now_steady - rxL[s]).seconds() * 1000.0;
    const double ageR_ms = (now_steady - rxR[s]).seconds() * 1000.0;

    if (ageL_ms > stale_timeout_ms_ || ageR_ms > stale_timeout_ms_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Stereo%d stale: ageL=%.1fms ageR=%.1fms > %.1fms. Clearing buffers.",
        s + 1, ageL_ms, ageR_ms, stale_timeout_ms_);

      std::lock_guard<std::mutex> lk(mtx_);
      stereo_buf_[s].left.reset();
      stereo_buf_[s].right.reset();
      return;
    }
  }

  // 3) Gate: se NON sono arrivati nuovi frame rispetto all'ultimo processing -> NON fare nulla
  bool any_new = false;
  for (int s = 0; s < num_stereo_pairs_; ++s) {
    const auto& Ls = snap[s].left->header.stamp;
    const auto& Rs = snap[s].right->header.stamp;

    const bool sameL = (Ls.sec == last_proc_left_stamp_[s].sec) &&
                       (Ls.nanosec == last_proc_left_stamp_[s].nanosec);
    const bool sameR = (Rs.sec == last_proc_right_stamp_[s].sec) &&
                       (Rs.nanosec == last_proc_right_stamp_[s].nanosec);

    if (!(sameL && sameR)) { any_new = true; break; }
  }

  if (!any_new) {
    // questo è il motivo per cui, quando stoppi il bag, NON deve più “elaborare”
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "No new frames -> skip.");
    return;
  }

  // 4) Skew check: riferimento = stereo1 left
  const auto t0 = snap[0].left->header.stamp;

  rclcpp::Duration max_skew = rclcpp::Duration::from_nanoseconds(0);
  int worst_s = -1;
  bool worst_is_right = false;
  rclcpp::Duration worst_dt = rclcpp::Duration::from_nanoseconds(0);

  for (int s = 0; s < num_stereo_pairs_; ++s) {
    const auto dtL = absDiff_(t0, snap[s].left->header.stamp);
    const auto dtR = absDiff_(t0, snap[s].right->header.stamp);

    if (dtL > worst_dt) { worst_dt = dtL; worst_s = s; worst_is_right = false; }
    if (dtR > worst_dt) { worst_dt = dtR; worst_s = s; worst_is_right = true;  }

    max_skew = std::max(max_skew, dtL);
    max_skew = std::max(max_skew, dtR);
  }

  const double max_skew_ms = max_skew.seconds() * 1000.0;
  if (max_skew_ms > max_stamp_skew_ms_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Stamp skew too large: %.2f ms > %.2f ms. Worst stereo%d %s dt=%.2f ms",
      max_skew_ms, max_stamp_skew_ms_,
      worst_s + 1, worst_is_right ? "R" : "L",
      worst_dt.seconds() * 1000.0);
    return;
  }

  // 5) Voxel map: reserve “moderato”, non 250k fisso
  const int rows = static_cast<int>(snap[0].left->height);
  const int cols = static_cast<int>(snap[0].left->width);
  const int step = std::max(1, pixel_step_);
  const size_t approx_samples =
    (size_t)((rows + step - 1) / step) * (size_t)((cols + step - 1) / step);

  std::unordered_map<int64_t, VoxelAcc> vox;
  vox.reserve((approx_samples / 8 + 2000) * (size_t)num_stereo_pairs_);

  // 6) Process + publish
  const rclcpp::Time t_start = this->now();

  for (int s = 0; s < num_stereo_pairs_; ++s) {
    processStereoPair_(snap[s].left, snap[s].right, /*camL=*/2*s, /*stereo_id=*/s, vox);
  }

  const rclcpp::Time t_proc_end = this->now();
  publishVoxelCloud_(vox, t0);
  const rclcpp::Time t_end = this->now();

  // 7) Aggiorna last processed SOLO dopo publish (così non “perdi” frame se skip per skew)
  for (int s = 0; s < num_stereo_pairs_; ++s) {
    last_proc_left_stamp_[s]  = snap[s].left->header.stamp;
    last_proc_right_stamp_[s] = snap[s].right->header.stamp;
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "Cloud: vox=%zu | process=%.1fms publish=%.1fms total=%.1fms",
    vox.size(),
    (t_proc_end - t_start).seconds() * 1000.0,
    (t_end - t_proc_end).seconds() * 1000.0,
    (t_end - t_start).seconds() * 1000.0);
}

void MultiStereoPlanningCloudNode::processStereoPair_(
  const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr& right_msg,
  int cam_left_index,
  int stereo_id,
  std::unordered_map<int64_t, VoxelAcc>& vox)
{
  // --- Convert robust to mono8
  cv::Mat left, right;
  try {
    left  = cv_bridge::toCvShare(left_msg, "mono8")->image;
    right = cv_bridge::toCvShare(right_msg, "mono8")->image;
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Stereo%d: cv_bridge mono8 failed: %s (encL='%s' encR='%s')",
      stereo_id + 1, e.what(), left_msg->encoding.c_str(), right_msg->encoding.c_str());
    return;
  }
  if (left.empty() || right.empty()) return;

  // --- Downscale
  const double scale = std::max(1e-6, downscale_);
  cv::Mat l_ds, r_ds;
  if (std::abs(scale - 1.0) > 1e-6) {
    cv::resize(left,  l_ds, cv::Size(), scale, scale, cv::INTER_AREA);
    cv::resize(right, r_ds, cv::Size(), scale, scale, cv::INTER_AREA);
  } else {
    l_ds = left;
    r_ds = right;
  }

  // --- Disparity (CV_16S, disp*16)
  cv::Mat disp16;
  sgbm_->compute(l_ds, r_ds, disp16);
  if (disp16.empty()) return;

  const int rows = disp16.rows;
  const int cols = disp16.cols;

  // --- Publish disparity debug (mono8 scaled) (cheap)
  if (publish_debug_disparity_ && pub_disp_[stereo_id]) {
    cv::Mat disp8;
    const double denom = std::max(1, sgbm_num_disp_);
    const double alpha = 255.0 / (denom * 16.0);
    disp16.convertTo(disp8, CV_8U, alpha);

    sensor_msgs::msg::Image msg;
    msg.header = left_msg->header;
    msg.height = static_cast<uint32_t>(disp8.rows);
    msg.width  = static_cast<uint32_t>(disp8.cols);
    msg.encoding = "mono8";
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(disp8.cols);
    msg.data.assign(disp8.datastart, disp8.dataend);
    pub_disp_[stereo_id]->publish(msg);
  }

  // --- Intrinsics scaled
  PinholeIntrinsics K = K_[cam_left_index];
  K.fx *= scale; K.fy *= scale; K.cx *= scale; K.cy *= scale;

  const double baseline = baseline_m_[stereo_id];
  const SE3& T_cam_imu  = T_cam_imu_[cam_left_index];

  const int step = std::max(1, pixel_step_);
  const double inv_leaf = 1.0 / std::max(1e-6, voxel_leaf_);

  // --- Adaptive disparity min from depth_max_
  // Keep points up to depth_max_ => disparity must be >= fx*B/depth_max
  const double disp_min_from_zmax = (K.fx * baseline) / std::max(1e-3, depth_max_);
  const double disp_min = std::max(0.5, 0.8 * disp_min_from_zmax);   // 0.8 margin
  const double disp_max = std::max(disp_min + 0.5, (K.fx * baseline) / std::max(1e-3, depth_min_));

  // --- Light texture filter on left (Sobel x)
  cv::Mat gx16;
  cv::Sobel(l_ds, gx16, CV_16S, 1, 0, 3);
  const int grad_thr = 10;  // conservative (raise to filter more)

  // --- Light disparity consistency filter
  const double disp_consistency_thr = 2.0; // px (raise to keep more)

  // --- Small helpers
  auto dispAt = [&](int u, int v) -> double {
    if (u < 0 || u >= cols || v < 0 || v >= rows) return -1.0;
    const int16_t raw = disp16.at<int16_t>(v, u);
    if (raw <= 0) return -1.0;
    return double(raw) / 16.0;
  };

  size_t cand = 0, kept = 0;

  // --- Main loop
  for (int v = 1; v < rows - 1; v += step) {
    const int16_t* dptr = disp16.ptr<int16_t>(v);
    for (int u = 1; u < cols - 1; u += step) {
      const int16_t d_raw = dptr[u];
      if (d_raw <= 0) continue;

      const double disp = double(d_raw) / 16.0;
      if (disp < disp_min || disp > disp_max) continue;

      // texture gate
      if (std::abs(gx16.at<int16_t>(v, u)) < grad_thr) continue;

      // local disparity consistency (simple 1D check)
      const double dL = dispAt(u - 1, v);
      const double dR = dispAt(u + 1, v);
      if (dL > 0 && dR > 0) {
        const double davg = 0.5 * (dL + dR);
        if (std::abs(disp - davg) > disp_consistency_thr) continue;
      }

      // back-project
      const double z = (K.fx * baseline) / disp;
      if (z < depth_min_ || z > depth_max_) continue;

      const double x = (double(u) - K.cx) * z / K.fx;
      const double y = (double(v) - K.cy) * z / K.fy;

      const Eigen::Vector3d p_imu = T_cam_imu.transform(Eigen::Vector3d(x, y, z));

      const int ix = int(std::floor(p_imu.x() * inv_leaf));
      const int iy = int(std::floor(p_imu.y() * inv_leaf));
      const int iz = int(std::floor(p_imu.z() * inv_leaf));

      const int64_t key = packKey_(ix, iy, iz);
      auto& acc = vox[key];
      acc.sum += p_imu;
      acc.count += 1;

      ++kept;
      ++cand;
    }
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "Stereo%d: disp_min=%.2f disp_max=%.2f | kept=%zu",
    stereo_id + 1, disp_min, disp_max, kept);
}


void MultiStereoPlanningCloudNode::publishVoxelCloud_(const std::unordered_map<int64_t, VoxelAcc>& vox,
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

  if (i != vox.size()) {
    cloud.width = static_cast<uint32_t>(i);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step * cloud.height);
  }

  pub_cloud_->publish(cloud);
}

} // namespace multicam_depth

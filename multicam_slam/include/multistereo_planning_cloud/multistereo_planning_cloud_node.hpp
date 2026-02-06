#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <iomanip>

#include <mutex>
#include <unordered_map>
#include <array>
#include <string>

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

struct VoxelAcc
{
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  int count = 0;
};

class MultiStereoPlanningCloudNode : public rclcpp::Node
{
public:
  explicit MultiStereoPlanningCloudNode(const rclcpp::NodeOptions& options);

private:
  static constexpr int kMaxCams   = 8;
  static constexpr int kMaxStereo = 4;

  // ---- parameters
  std::string calib_json_path_;
  std::string calib_json_string_;
  std::string imu_frame_;
  std::string output_topic_;

  int num_cameras_ = 6;
  int num_stereo_pairs_ = 3;

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

  double disp_min_px_{4.0};                 // soglia disparità minima (in pixel)
  double disp_local_thr_px_{1.5};           // tolleranza locale (px)
  int    disp_local_support_{2};            // quanti vicini devono “concordare”

  // ---- calibration
  std::array<PinholeIntrinsics, kMaxCams> K_{};
  std::array<SE3, kMaxCams> T_imu_cam_{};
  std::array<SE3, kMaxCams> T_cam_imu_{};
  std::array<double, kMaxStereo> baseline_m_{};

  // --- per evitare di processare sempre lo stesso frame ---
  std::array<builtin_interfaces::msg::Time, kMaxStereo> last_proc_left_stamp_{};
  std::array<builtin_interfaces::msg::Time, kMaxStereo> last_proc_right_stamp_{};

  // --- opzionale: detect “stale” usando wall time (sempre valido anche senza /clock) ---
  std::array<rclcpp::Time, kMaxStereo> last_rx_wall_time_left_{};
  std::array<rclcpp::Time, kMaxStereo> last_rx_wall_time_right_{};
  double stale_timeout_ms_{500.0};   // param

  // ---- ROS
  std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, kMaxStereo> sub_left_{};
  std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, kMaxStereo> sub_right_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool publish_debug_disparity_ = false;
  std::string disparity_topic_prefix_ = "debug/disparity";

  std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, kMaxStereo> pub_disp_{};


  // ---- stereo
  cv::Ptr<cv::StereoSGBM> sgbm_;

  // ---- image buffers
  struct StereoBuffer
  {
    sensor_msgs::msg::Image::ConstSharedPtr left;
    sensor_msgs::msg::Image::ConstSharedPtr right;
  };

  std::mutex mtx_;
  std::array<StereoBuffer, kMaxStereo> stereo_buf_{};

  // ---- methods
  void onImage_(int stereo_id, bool is_left, const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void onTimer_();

  void loadCalibrationOrThrow_();
  void computeBaselineForPair_(int stereo_id);

  void processStereoPair_(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                          const sensor_msgs::msg::Image::ConstSharedPtr& right_msg,
                          int cam_left_index,
                          int stereo_id,
                          std::unordered_map<int64_t, VoxelAcc>& vox);

  void publishVoxelCloud_(const std::unordered_map<int64_t, VoxelAcc>& vox,
                          const builtin_interfaces::msg::Time& stamp);

  static rclcpp::Duration absDiff_(const builtin_interfaces::msg::Time& a,
                                  const builtin_interfaces::msg::Time& b);

  static SE3 se3FromImuToCamEntry_(const nlohmann::json& j);
  static PinholeIntrinsics pinholeFromEntry_(const nlohmann::json& intr_j, const std::array<int,2>& res);
  static int64_t packKey_(int ix, int iy, int iz);
};

} // namespace multicam_depth

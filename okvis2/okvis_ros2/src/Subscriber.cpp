// Subscriber.cpp (ROS2) - robust version
#include <cmath>
#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <limits>

#include <glog/logging.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>

#include <okvis/ros2/Subscriber.hpp>

namespace okvis {

static constexpr double DEFAULT_SYNC_THR_S = 0.01;

Subscriber::Subscriber(std::shared_ptr<rclcpp::Node> node,
                       okvis::ViInterface* viInterface,
                       okvis::Publisher* publisher,
                       const okvis::ViParameters& parameters)
: viInterface_(viInterface),
  publisher_(publisher),
  parameters_(parameters)
{
  setNodeHandle(node);
}

Subscriber::~Subscriber()
{
  shutdown();
}

void Subscriber::setNodeHandle(std::shared_ptr<rclcpp::Node> node)
{
  node_ = node;

  const size_t N = parameters_.nCameraSystem.numCameras();
  imageSubscribers_.resize(N);

  img_buf_.clear();
  img_buf_.resize(N);

  // Tuning knobs (feel free to expose as params)
  sync_thr_s_       = DEFAULT_SYNC_THR_S;
  target_hz_        = 10.0;     // start conservative
  max_buf_per_cam_  = 120;      // prevent unbounded growth

  // --- IMU callback group dedicated ---
  cbg_imu_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // --- image transport ---
  imgTransport_ = std::make_unique<image_transport::ImageTransport>(node_);

  const int img_queue = 30 * static_cast<int>(N);

  // IMPORTANT: subscribe to RELATIVE topics, so remaps like "cam0/image_raw:=..." work.
  for (size_t i = 0; i < N; ++i) {
    const std::string topic = "cam" + std::to_string(i) + "/image_raw";  // RELATIVE
    imageSubscribers_[i] = imgTransport_->subscribe(
      topic,
      img_queue,
      std::bind(&Subscriber::imageCallback, this, std::placeholders::_1, static_cast<unsigned int>(i)));
  }

  // IMU subscription (relative topic: "imu0")
  rclcpp::SubscriptionOptions imu_opts;
  imu_opts.callback_group = cbg_imu_;

  subImu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu0",  // RELATIVE -> your remap "imu0:=/uav50/device1/..." will work
    rclcpp::SensorDataQoS().keep_last(5000),
    std::bind(&Subscriber::imuCallback, this, std::placeholders::_1),
    imu_opts);

  // Start sync thread
  running_.store(true);
  last_accepted_ns_ = 0;
  sync_thread_ = std::thread(&Subscriber::syncLoop, this);

  LOG(INFO) << "[Subscriber] started: cams=" << N
            << " sync_thr=" << sync_thr_s_ << "s target_hz=" << target_hz_;
}

void Subscriber::shutdown()
{
  const bool was_running = running_.exchange(false);
  if (was_running) {
    buf_cv_.notify_all();
    if (sync_thread_.joinable()) sync_thread_.join();
  }

  for (auto & s : imageSubscribers_) s.shutdown();
  subImu_.reset();

  {
    std::lock_guard<std::mutex> lk(buf_mtx_);
    for (auto & b : img_buf_) b.clear();
  }
}

void Subscriber::dropOldestUnlocked(size_t cam, size_t n)
{
  auto & b = img_buf_.at(cam);
  while (n-- && !b.empty()) {
    b.erase(b.begin());
  }
}

// ---------- IMAGE CALLBACK (ultra-light) ----------
void Subscriber::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                               unsigned int cameraIndex)
{
  if (cameraIndex >= img_buf_.size()) return;

  okvis::Time t(msg->header.stamp.sec, msg->header.stamp.nanosec);
  const uint64_t t_ns = t.toNSec();

  const cv::Mat raw(msg->height, msg->width, CV_8UC1,
                    const_cast<uint8_t*>(msg->data.data()), msg->step);

  // Clone to decouple lifetime from ROS message
  cv::Mat img = raw.clone();

  {
    std::lock_guard<std::mutex> lk(buf_mtx_);
    auto & b = img_buf_[cameraIndex];
    b[t_ns] = std::move(img);

    if (b.size() > max_buf_per_cam_) {
      dropOldestUnlocked(cameraIndex, b.size() - max_buf_per_cam_);
    }
  }

  buf_cv_.notify_one();
}

// ---------- SYNC LOOP (robust, no map::at) ----------
void Subscriber::syncLoop()
{
  const size_t N = img_buf_.size();
  if (N == 0) return;

  const double min_dt_s = 1.0 / std::max(0.5, target_hz_);
  const uint64_t min_dt_ns = static_cast<uint64_t>(min_dt_s * 1e9);
  const uint64_t thr_ns = static_cast<uint64_t>(sync_thr_s_ * 1e9);

  while (running_.load()) {
    // wait for data / timeout
    {
      std::unique_lock<std::mutex> ul(buf_mtx_);
      buf_cv_.wait_for(ul, std::chrono::milliseconds(5));
    }

    // Try produce ONE synced set per loop (prevents bursts)
    okvis::Time t_to_add;
    std::map<size_t, cv::Mat> images_out;
    bool accept = false;

    {
      std::lock_guard<std::mutex> lk(buf_mtx_);

      // Need at least cam0
      if (img_buf_[0].empty()) continue;

      const uint64_t t_ref_ns = img_buf_[0].begin()->first;

      // For each camera store the chosen iterator (so we never need map::at)
      std::vector<std::map<uint64_t, cv::Mat>::iterator> chosen_it(N);

      // Find per-cam closest to t_ref_ns within threshold
      for (size_t cam = 0; cam < N; ++cam) {
        auto & b = img_buf_[cam];
        if (b.empty()) {
          goto not_synced;
        }

        auto it = b.lower_bound(t_ref_ns);

        auto best = b.end();
        int64_t best_dt = std::numeric_limits<int64_t>::max();

        auto consider = [&](decltype(it) jt) {
          if (jt == b.end()) return;
          int64_t adt = std::llabs((int64_t)jt->first - (int64_t)t_ref_ns);
          if (adt < best_dt) { best_dt = adt; best = jt; }
        };

        consider(it);
        if (it != b.begin()) consider(std::prev(it));

        if (best == b.end() || best_dt > (int64_t)thr_ns) {
          // If this cam's oldest is already newer than t_ref+thr, then t_ref is too old -> drop cam0 oldest
          if (b.begin()->first > t_ref_ns + thr_ns) {
            img_buf_[0].erase(img_buf_[0].begin());
          }
          goto not_synced;
        }

        chosen_it[cam] = best;
      }

      // Synced set found
      // Throttle on t_ref
      if (last_accepted_ns_ == 0 || (t_ref_ns - last_accepted_ns_) >= min_dt_ns) {
        accept = true;
        last_accepted_ns_ = t_ref_ns;

        t_to_add.fromNSec(t_ref_ns);
        for (size_t cam = 0; cam < N; ++cam) {
          images_out[cam] = chosen_it[cam]->second; // copy Mat header (data shared)
        }
      }

      // ALWAYS purge up to (and including) chosen_it per cam
      for (size_t cam = 0; cam < N; ++cam) {
        auto & b = img_buf_[cam];
        auto end = chosen_it[cam];
        ++end; // include used element
        b.erase(b.begin(), end);
      }

not_synced:
      (void)0;
    } // unlock

    // Call SLAM outside lock
    if (accept && !images_out.empty()) {
      if (!viInterface_->addImages(t_to_add, images_out)) {
        LOG(WARNING) << "addImages rejected at t=" << t_to_add;
      }
    }
  }
}

// ---------- IMU ----------
void Subscriber::imuCallback(const sensor_msgs::msg::Imu& msg)
{
  okvis::Time ts(msg.header.stamp.sec, msg.header.stamp.nanosec);

  const Eigen::Vector3d acc(msg.linear_acceleration.x,
                            msg.linear_acceleration.y,
                            msg.linear_acceleration.z);

  const Eigen::Vector3d gyr(msg.angular_velocity.x,
                            msg.angular_velocity.y,
                            msg.angular_velocity.z);

  viInterface_->addImuMeasurement(ts, acc, gyr);
}

} // namespace okvis

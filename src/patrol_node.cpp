#include "cat_patrol_robot/patrol_node.hpp"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <sstream>

#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
using namespace std::chrono_literals;

namespace cat_patrol_robot
{

static double normalize_angle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

PatrolNode::PatrolNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("patrol_node", options)
{
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  image_topic_ = declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
  buzzer_topic_ = declare_parameter<std::string>("buzzer_topic", "Buzzer");
  joy_topic_ = declare_parameter<std::string>("joy_active_topic", "/JoyState");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  image_save_dir_ = declare_parameter<std::string>("image_save_dir", "/tmp/cat_patrol_images");
  mail_request_topic_ = declare_parameter<std::string>("mail_request_topic", "/cat_patrol/mail_request");
  mail_subject_ = declare_parameter<std::string>("mail_subject", "Cat patrol photos");
  mail_to_ = declare_parameter<std::string>("smtp_to_address", "user@example.com");
  cat_detected_topic_ = declare_parameter<std::string>("cat_detected_topic", "/cat_patrol/cat_detected");

  use_lidar_ = declare_parameter<bool>("use_lidar_obstacle_stop", false);
  obstacle_min_range_ = declare_parameter<double>("obstacle_min_range_m", 0.35);
  obstacle_sector_deg_ = declare_parameter<double>("obstacle_sector_deg", 60.0);
  linear_speed_ = declare_parameter<double>("linear_speed", 0.2);
  angular_speed_ = declare_parameter<double>("angular_speed", 0.6);
  patrol_drive_sec_ = declare_parameter<double>("patrol_drive_sec", 10.0);
  capture_frame_count_ = declare_parameter<int>("capture_frame_count", 12);
  capture_turn_speed_ = declare_parameter<double>("capture_turn_speed", 3.0);
  capture_rotate_sec_ = declare_parameter<double>("capture_rotate_sec", 3.0);
  return_timeout_sec_ = declare_parameter<double>("return_home_max_time_sec", 120.0);
  pos_tol_ = declare_parameter<double>("position_tolerance_m", 0.15);
  yaw_tol_ = declare_parameter<double>("yaw_tolerance_rad", 0.2);
  patrol_period_sec_ = declare_parameter<double>("patrol_period_sec", 1800.0);
  start_on_boot_ = declare_parameter<bool>("start_patrol_on_boot", true);
  loop_patrol_ = declare_parameter<bool>("loop_patrol", true);
  cat_detection_enabled_ = declare_parameter<bool>("cat_detection_enabled", false);
  approach_linear_ = declare_parameter<double>("approach_linear_speed", 0.1);
  approach_max_sec_ = declare_parameter<double>("approach_max_sec", 15.0);
  exit_after_one_cycle_ = declare_parameter<bool>("exit_after_one_cycle", false);
  use_joy_override_ = declare_parameter<bool>("use_joy_override", false);
  if (std::getenv("CAT_PATROL_EXIT_ONCE")) {
    exit_after_one_cycle_ = true;
  }

  const int period_ms = declare_parameter<int>("patrol_timer_period_ms", 50);

  std::filesystem::create_directories(image_save_dir_);

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  buzzer_pub_ = create_publisher<std_msgs::msg::Bool>(buzzer_topic_, 10);
  mail_pub_ = create_publisher<std_msgs::msg::String>(mail_request_topic_, 10);

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PatrolNode::image_cb, this, std::placeholders::_1));

  if (use_lidar_) {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PatrolNode::scan_cb, this, std::placeholders::_1));
  }

  if (use_joy_override_) {
    joy_sub_ = create_subscription<std_msgs::msg::Bool>(
      joy_topic_, 10, std::bind(&PatrolNode::joy_cb, this, std::placeholders::_1));
  }

  cat_sub_ = create_subscription<std_msgs::msg::Bool>(
    cat_detected_topic_, 10, std::bind(&PatrolNode::cat_detected_cb, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  patrol_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms), std::bind(&PatrolNode::patrol_timer_cb, this));

  state_enter_time_ = now();

  if (loop_patrol_) {
    idle_cycle_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(1.0, patrol_period_sec_)),
      [this]() {
        if (state_ == PatrolState::Idle) {
          RCLCPP_INFO(get_logger(), "Patrol period elapsed — starting cycle");
          transition_to(PatrolState::Patrol);
        }
      });
  }

  if (start_on_boot_) {
    state_ = PatrolState::WaitingForTf;
    RCLCPP_INFO(get_logger(), "Waiting for TF (odom -> base_footprint) before first patrol...");
  } else {
    transition_to(PatrolState::Idle);
  }
}

void PatrolNode::transition_to(PatrolState s)
{
  stop_robot();
  state_ = s;
  state_enter_time_ = now();

  if (s == PatrolState::Patrol) {
    cat_detected_.store(false);
    if (get_odom_pose(home_x_, home_y_, home_yaw_)) {
      RCLCPP_INFO(get_logger(), "Recorded home (odom): x=%.3f y=%.3f yaw=%.3f",
                  home_x_, home_y_, home_yaw_);
    } else {
      RCLCPP_WARN(get_logger(), "Could not read odom pose at patrol start — return-home may fail");
    }
    RCLCPP_INFO(get_logger(), "Driving forward for %.1f seconds", patrol_drive_sec_);
  }

  if (s == PatrolState::Capture) {
    saved_paths_.clear();
    frames_saved_ = 0;
    capture_phase_start_ = now();
    capture_phase_ = CapturePhase::Snap;
    RCLCPP_INFO(get_logger(), "Capture: will take %d images over 360 degrees", capture_frame_count_);
  }

  if (s == PatrolState::CatApproach) {
    approach_start_ = now();
    std_msgs::msg::Bool b;
    b.data = true;
    buzzer_pub_->publish(b);
  }
}

bool PatrolNode::get_odom_pose(double & x, double & y, double & yaw)
{
  try {
    geometry_msgs::msg::TransformStamped t =
      tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    x = t.transform.translation.x;
    y = t.transform.translation.y;
    tf2::Quaternion q(
      t.transform.rotation.x, t.transform.rotation.y,
      t.transform.rotation.z, t.transform.rotation.w);
    double roll{}, pitch{};
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "TF lookup failed: %s", ex.what());
    return false;
  }
}

void PatrolNode::publish_twist(double lin_x, double lin_y, double ang_z)
{
  geometry_msgs::msg::Twist t;
  t.linear.x = lin_x;
  t.linear.y = lin_y;
  t.angular.z = ang_z;
  cmd_pub_->publish(t);
}

void PatrolNode::stop_robot()
{
  publish_twist(0.0, 0.0, 0.0);
}

bool PatrolNode::obstacle_too_close()
{
  if (!use_lidar_) {
    return false;
  }
  return min_scan_range_.load() < static_cast<float>(obstacle_min_range_);
}

void PatrolNode::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  float min_r = 1e9f;
  const float half = static_cast<float>(obstacle_sector_deg_ * 0.5 * M_PI / 180.0);
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    float r = msg->ranges[i];
    if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
      continue;
    }
    const float ang = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
    if (std::abs(ang) <= half) {
      min_r = std::min(min_r, r);
    }
  }
  min_scan_range_ = min_r;
}

void PatrolNode::joy_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  joy_active_.store(msg->data);
}

void PatrolNode::cat_detected_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  cat_detected_.store(msg->data);
}

void PatrolNode::image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
{
  last_image_ = msg;
}

std::string PatrolNode::escape_json(const std::string & s) const
{
  std::string out;
  out.reserve(s.size() + 8);
  for (char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    out += c;
  }
  return out;
}

void PatrolNode::send_mail_request()
{
  std::ostringstream oss;
  oss << "{\"subject\":\"" << escape_json(mail_subject_) << "\","
      << "\"to\":\"" << escape_json(mail_to_) << "\","
      << "\"paths\":[";
  for (size_t i = 0; i < saved_paths_.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << "\"" << escape_json(saved_paths_[i]) << "\"";
  }
  oss << "]}";
  std_msgs::msg::String out;
  out.data = oss.str();
  mail_pub_->publish(out);
  RCLCPP_INFO(get_logger(), "Published mail request (%zu attachments)", saved_paths_.size());
}

void PatrolNode::patrol_timer_cb()
{
  if (joy_active_.load()) {
    stop_robot();
    return;
  }

  switch (state_) {
    case PatrolState::WaitingForTf: {
      double x, y, yaw;
      if (get_odom_pose(x, y, yaw)) {
        RCLCPP_INFO(get_logger(), "TF available — starting patrol");
        transition_to(PatrolState::Patrol);
      }
      break;
    }
    case PatrolState::Idle:
      break;
    case PatrolState::Patrol:
      if (cat_detection_enabled_ && cat_detected_.load()) {
        transition_to(PatrolState::CatApproach);
        return;
      }
      patrol_tick();
      break;
    case PatrolState::Capture:
      capture_tick();
      break;
    case PatrolState::ReturnHome:
      return_home_tick();
      break;
    case PatrolState::CatApproach:
      cat_approach_tick();
      break;
  }
}

void PatrolNode::patrol_tick()
{
  if (obstacle_too_close()) {
    stop_robot();
    return;
  }

  const double elapsed = (now() - state_enter_time_).seconds();
  const double leg = patrol_drive_sec_;
  const double total = leg * 4.0;

  if (elapsed >= total) {
    RCLCPP_INFO(get_logger(), "Patrol drive done — transitioning to Capture");
    stop_robot();
    transition_to(PatrolState::Capture);
    return;
  }

  int phase = static_cast<int>(elapsed / leg);
  double vx = (phase % 2 == 0) ? linear_speed_ : -linear_speed_;

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "Patrol %s: %.1f / %.1fs", (vx > 0 ? "FWD" : "BACK"), elapsed, total);

  publish_twist(vx, 0.0, 0.0);
}

void PatrolNode::capture_tick()
{
  const auto tnow = now();
  const double total_elapsed = (tnow - capture_phase_start_).seconds();
  if (total_elapsed > 90.0) {
    RCLCPP_WARN(get_logger(), "Capture phase timeout after %.0fs", total_elapsed);
    stop_robot();
    send_mail_request();
    transition_to(PatrolState::ReturnHome);
    return;
  }

  if (frames_saved_ >= capture_frame_count_) {
    stop_robot();
    RCLCPP_INFO(get_logger(), "Capture complete — %d images taken in %.1fs",
                frames_saved_, total_elapsed);
    send_mail_request();
    transition_to(PatrolState::ReturnHome);
    return;
  }

  const double rotate_sec = capture_rotate_sec_;
  const double settle_sec = 0.8;

  switch (capture_phase_) {
    case CapturePhase::Snap: {
      if (!last_image_ || last_image_->data.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "No image received yet on %s", image_topic_.c_str());
        return;
      }
      if (save_current_image()) {
        RCLCPP_INFO(get_logger(), "Captured [%d/%d]", frames_saved_, capture_frame_count_);
        if (frames_saved_ < capture_frame_count_) {
          capture_phase_ = CapturePhase::Rotate;
          capture_rotate_end_ = tnow + rclcpp::Duration::from_seconds(rotate_sec);
          RCLCPP_INFO(get_logger(), "Rotating for %.1fs at %.1f rad/s",
                      rotate_sec, capture_turn_speed_);
        }
      }
      break;
    }
    case CapturePhase::Rotate: {
      if (tnow >= capture_rotate_end_) {
        stop_robot();
        capture_phase_ = CapturePhase::Settle;
        capture_settle_end_ = tnow + rclcpp::Duration::from_seconds(settle_sec);
        RCLCPP_INFO(get_logger(), "Rotation done, settling %.1fs", settle_sec);
      } else {
        publish_twist(0.0, 0.0, capture_turn_speed_);
      }
      break;
    }
    case CapturePhase::Settle: {
      if (tnow >= capture_settle_end_) {
        capture_phase_ = CapturePhase::Snap;
      }
      break;
    }
  }
}

bool PatrolNode::save_current_image()
{
  if (!last_image_ || last_image_->width == 0 || last_image_->height == 0 ||
      last_image_->data.empty()) {
    return false;
  }

  cv::Mat bgr_image;
  try {
    const std::string & enc = last_image_->encoding;
    const int rows = static_cast<int>(last_image_->height);
    const int cols = static_cast<int>(last_image_->width);
    const int step = static_cast<int>(last_image_->step);

    if (enc == "rgb8") {
      cv::Mat rgb(rows, cols, CV_8UC3, const_cast<uint8_t*>(last_image_->data.data()), step);
      cv::cvtColor(rgb, bgr_image, cv::COLOR_RGB2BGR);
    } else if (enc == "bgr8") {
      cv::Mat src(rows, cols, CV_8UC3, const_cast<uint8_t*>(last_image_->data.data()), step);
      bgr_image = src.clone();
    } else if (enc == "mono8") {
      cv::Mat mono(rows, cols, CV_8UC1, const_cast<uint8_t*>(last_image_->data.data()), step);
      cv::cvtColor(mono, bgr_image, cv::COLOR_GRAY2BGR);
    } else if (enc == "16UC1" || enc == "mono16") {
      cv::Mat raw16(rows, cols, CV_16UC1, const_cast<uint8_t*>(last_image_->data.data()), step);
      cv::Mat mono8;
      raw16.convertTo(mono8, CV_8UC1, 255.0 / 1000.0);
      cv::cvtColor(mono8, bgr_image, cv::COLOR_GRAY2BGR);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
        "Unsupported encoding '%s'", enc.c_str());
      return false;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Image conversion: %s", e.what());
    return false;
  }
  if (bgr_image.empty()) {
    return false;
  }

  std::filesystem::path dir(image_save_dir_);
  std::ostringstream fname;
  fname << "snap_" << now().nanoseconds() << "_" << frames_saved_ << ".jpg";
  std::filesystem::path fp = dir / fname.str();
  if (cv::imwrite(fp.string(), bgr_image)) {
    saved_paths_.push_back(fp.string());
    frames_saved_++;
    return true;
  }
  return false;
}

void PatrolNode::return_home_tick()
{
  if ((now() - state_enter_time_).seconds() > return_timeout_sec_) {
    RCLCPP_WARN(get_logger(), "Return home timeout");
    stop_robot();
    transition_to(PatrolState::Idle);
    reset_idle_loop();
    if (exit_after_one_cycle_) {
      rclcpp::shutdown();
    }
    return;
  }

  double x, y, yaw;
  if (!get_odom_pose(x, y, yaw)) {
    stop_robot();
    return;
  }

  const double dx = home_x_ - x;
  const double dy = home_y_ - y;
  const double dist = std::hypot(dx, dy);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "RETURN dist_to_home=%.3f pos=(%.3f,%.3f) home=(%.3f,%.3f)",
    dist, x, y, home_x_, home_y_);

  if (dist < pos_tol_) {
    const double ey = normalize_angle(home_yaw_ - yaw);
    if (std::abs(ey) < yaw_tol_) {
      stop_robot();
      RCLCPP_INFO(get_logger(), "Returned home");
      transition_to(PatrolState::Idle);
      reset_idle_loop();
      if (exit_after_one_cycle_) {
        RCLCPP_INFO(get_logger(), "exit_after_one_cycle: shutting down ROS");
        rclcpp::shutdown();
      }
      return;
    }
    const double w = std::copysign(angular_speed_, ey);
    publish_twist(0.0, 0.0, w);
    return;
  }

  const double aim = std::atan2(dy, dx);
  const double turn = normalize_angle(aim - yaw);
  double w = std::clamp(turn * 2.0, -angular_speed_, angular_speed_);
  double v = std::clamp(linear_speed_ * 0.7, 0.0, linear_speed_);
  if (std::abs(turn) > yaw_tol_ * 2.0) {
    v = 0.0;
  }
  publish_twist(v, 0.0, w);
}

void PatrolNode::reset_idle_loop()
{
  std_msgs::msg::Bool b;
  b.data = false;
  buzzer_pub_->publish(b);
}

void PatrolNode::cat_approach_tick()
{
  if ((now() - approach_start_).seconds() > approach_max_sec_) {
    RCLCPP_WARN(get_logger(), "Cat approach timeout");
    std_msgs::msg::Bool b;
    b.data = false;
    buzzer_pub_->publish(b);
    transition_to(PatrolState::Patrol);
    return;
  }

  if (obstacle_too_close()) {
    stop_robot();
    return;
  }

  publish_twist(approach_linear_, 0.0, 0.0);

  if (!cat_detected_.load()) {
    stop_robot();
    std_msgs::msg::Bool b;
    b.data = false;
    buzzer_pub_->publish(b);
    transition_to(PatrolState::Patrol);
  }
}

}  // namespace cat_patrol_robot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cat_patrol_robot::PatrolNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

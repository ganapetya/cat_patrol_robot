#ifndef CAT_PATROL_ROBOT__PATROL_NODE_HPP_
#define CAT_PATROL_ROBOT__PATROL_NODE_HPP_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace cat_patrol_robot
{

enum class PatrolState
{
  WaitingForTf,
  Idle,
  Patrol,
  Capture,
  ReturnHome,
  CatApproach,
};

class PatrolNode : public rclcpp::Node
{
public:
  explicit PatrolNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void patrol_timer_cb();
  void image_cb(const sensor_msgs::msg::Image::SharedPtr msg);
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void joy_cb(const std_msgs::msg::Bool::SharedPtr msg);
  void cat_detected_cb(const std_msgs::msg::Bool::SharedPtr msg);

  void transition_to(PatrolState s);
  bool get_odom_pose(double & x, double & y, double & yaw);
  void publish_twist(double lin_x, double lin_y, double ang_z);
  void stop_robot();
  bool obstacle_too_close();

  void patrol_tick();

  void capture_tick();
  void return_home_tick();
  void cat_approach_tick();

  bool save_current_image();
  void send_mail_request();
  std::string escape_json(const std::string & s) const;
  void reset_idle_loop();

  // Parameters / topics
  std::string cmd_vel_topic_;
  std::string image_topic_;
  std::string scan_topic_;
  std::string buzzer_topic_;
  std::string joy_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string image_save_dir_;
  std::string mail_request_topic_;
  std::string mail_subject_;
  std::string mail_to_;
  std::string cat_detected_topic_;

  bool use_lidar_{false};
  double obstacle_min_range_{0.35};
  double obstacle_sector_deg_{60.0};
  double linear_speed_{0.2};
  double angular_speed_{0.6};
  double patrol_drive_sec_{10.0};
  int capture_frame_count_{12};
  double capture_turn_speed_{3.0};
  double capture_rotate_sec_{3.0};
  double return_timeout_sec_{120.0};
  double pos_tol_{0.12};
  double yaw_tol_{0.2};
  double patrol_period_sec_{1800.0};
  bool start_on_boot_{true};
  bool loop_patrol_{true};
  bool cat_detection_enabled_{false};
  double approach_linear_{0.1};
  double approach_max_sec_{15.0};
  bool exit_after_one_cycle_{false};
  bool use_joy_override_{false};

  // Home return
  double home_x_{0}, home_y_{0}, home_yaw_{0};

  // Capture: timed rotate-and-snap
  std::vector<std::string> saved_paths_;
  int frames_saved_{0};
  sensor_msgs::msg::Image::SharedPtr last_image_;
  rclcpp::Time capture_phase_start_;
  rclcpp::Time capture_rotate_end_;
  rclcpp::Time capture_settle_end_;
  enum class CapturePhase { Snap, Rotate, Settle };
  CapturePhase capture_phase_{CapturePhase::Snap};

  // Cat / laser / joy
  std::atomic<float> min_scan_range_{1000.0f};
  std::atomic_bool joy_active_{false};
  std::atomic_bool cat_detected_{false};

  // Cat approach phase
  rclcpp::Time approach_start_;

  PatrolState state_{PatrolState::Idle};
  rclcpp::Time state_enter_time_;
  rclcpp::TimerBase::SharedPtr patrol_timer_;
  rclcpp::TimerBase::SharedPtr idle_cycle_timer_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr buzzer_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mail_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cat_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace cat_patrol_robot

#endif  // CAT_PATROL_ROBOT__PATROL_NODE_HPP_

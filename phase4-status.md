# Phase 4 — Patrol Behavior (Nav2-native) Status & Reference

Companion to [`plan.md`](plan.md), [`phase3-status.md`](phase3-status.md),
[`phase2-status.md`](phase2-status.md), [`architecture.md`](architecture.md),
and [`hardware.md`](hardware.md).

Phase 4 = build a **Nav2-native patrol manager** that wakes on schedule,
drives through recorded waypoints, captures photos at each waypoint, returns
home, sends one email bundle, and goes back to sleep.

> **STATUS: In progress — first run attempted 2026-07-09, BLOCKED at wp5.**
> Package built and running; 10 waypoints + home captured on the new map;
> first cycle aborted when the Nav2 goal for wp5 failed. See §11 Session 1 for
> the full state and the resume plan. Use §11 as your running session log.

---

## 1. Purpose and scope

**Goal.** Replace open-loop patrol driving with a Nav2-action-driven manager:

- `SLEEPING → WAKING → PATROLLING(waypoint i) → CAPTURE(i) → … → RETURNING → SLEEPING`
- Keep a hook for `INVESTIGATING` (Phase 6).

**Why this phase matters (from `plan.md`).**
- This is the project’s core C++ architecture phase:
  FSM + action callbacks + callback-group concurrency.
- It reuses Phase 3’s `NavigateToPose` action client pattern from
  `src/nav_goal_client_node.cpp`.
- It carries forward the Strategy-style design from
  `include/cat_patrol_robot/patterns/patrol_pattern.hpp`, but now the
  motion primitive is Nav2 goals (not raw `/cmd_vel`).

**What changes vs Phase 3.**
- Phase 3 proved single-goal navigation; Phase 4 sequences many goals.
- Phase 4 adds scheduling (`patrol_period_sec`) and cycle orchestration.
- Photo capture at each waypoint becomes part of the patrol pipeline.
- Return-home becomes explicit Nav2 goal to recorded `home` pose.

**Scope.**
- One map (`living_room_v1`).
- 4–6 waypoints + 1 home pose.
- Single-room unattended patrol cycle.
- Wake-up scheduling by ROS timer only (systemd timer integration deferred to
  Phase 7 as planned).

---

## 2. Acceptance criteria

Phase 4 is done when all required items pass:

### Required
- [ ] `patrol_manager` package builds and launches cleanly.
- [ ] Node reaches `SLEEPING` after startup and wakes itself by timer.
- [ ] On wake, it sends Nav2 goals for all configured waypoints in order.
- [ ] At each waypoint, one photo is saved to disk and path appended to cycle list.
- [ ] After last waypoint, robot navigates to home pose.
- [ ] After arriving home, exactly one mail request is published with all photo paths.
- [ ] FSM returns to `SLEEPING` and repeats next cycle (if `loop_patrol=true`).
- [ ] No raw `/cmd_vel` publishes from `patrol_manager` (Nav2-only movement).
- [ ] Executor remains responsive during active navigation (no callback starvation).

### Nice-to-have (stretch)
- [ ] Publish patrol telemetry (current waypoint index, distance remaining,
      recovery count if available).
- [ ] Add `NavigateThroughPoses` variant for A/B test vs sequential goals.
- [ ] Add a tiny waypoint recorder helper node for faster map-pose capture.

---

## 3. What we need to create

| File | Purpose |
|---|---|
| `../patrol_manager/config/patrol_manager_params.yaml` | Phase 4 runtime params (schedule, topics, waypoints, home pose) |
| `../patrol_manager/launch/patrol_manager.launch.py` | Brings up patrol manager with params |
| `../patrol_manager/include/patrol_manager/patrol_strategy.hpp` | Strategy interface for patrol behaviors |
| `../patrol_manager/include/patrol_manager/waypoint_patrol_strategy.hpp` | Waypoint implementation strategy |
| `../patrol_manager/src/waypoint_patrol_strategy.cpp` | Strategy logic scaffold |
| `../patrol_manager/src/patrol_manager_node.cpp` | Main FSM + Nav2 action client + capture + mail trigger |
| `../patrol_manager/CMakeLists.txt` + `../patrol_manager/package.xml` | Build wiring and dependencies |
| `~/myscripts2/t5_patrol.sh` | Convenience launch wrapper (mirrors t1/t2/t2.5/t3/t4) |

Notes:
- Package location is workspace-level: `yahboomcar_ws/src/patrol_manager`.
- Reuse references from existing code:
  - Action client pattern: `cat_patrol_robot/src/nav_goal_client_node.cpp`
  - JPEG conversion/save logic: `cat_patrol_robot/src/patrol_node.cpp`
  - Strategy-style organization: `cat_patrol_robot/include/cat_patrol_robot/patterns/patrol_pattern.hpp`

---

## 4. Files to create (before first session)

### 4a. Create package scaffold

```bash
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src
ros2 pkg create patrol_manager \
  --build-type ament_cmake \
  --dependencies rclcpp rclcpp_action nav2_msgs geometry_msgs sensor_msgs std_msgs

mkdir -p /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/include/patrol_manager
mkdir -p /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/src
mkdir -p /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/config
mkdir -p /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/launch
```

### 4b. `../patrol_manager/config/patrol_manager_params.yaml`

This keeps **all Phase 4 behavior** in one editable YAML file,
including waypoint data (no extra parser dependency needed).

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/config/patrol_manager_params.yaml << 'EOF'
patrol_manager:
  ros__parameters:
    # --- Scheduling ---
    start_on_boot: true
    loop_patrol: true
    patrol_period_sec: 1800.0

    # --- Navigation action ---
    navigate_action_name: "navigate_to_pose"
    nav_goal_timeout_sec: 180.0
    settle_after_arrival_sec: 1.0

    # --- Topics ---
    image_topic: "/camera/color/image_raw"
    mail_request_topic: "/cat_patrol/mail_request"
    state_topic: "/patrol_manager/state"

    # --- Capture + mail ---
    image_save_dir: "/tmp/cat_patrol_images"
    mail_subject: "Cat patrol photos"
    smtp_to_address: "user@example.com"

    # --- Frame assumptions ---
    map_frame: "map"

    # --- Home pose in map frame (x, y, yaw radians) ---
    home_pose: [0.0, 0.0, 0.0]

    # --- Waypoints in map frame ---
    # Keep all 3 arrays same length.
    waypoint_names: ["wp1", "wp2", "wp3", "wp4"]
    waypoint_x: [ 1.20,  2.00,  1.80,  0.80]
    waypoint_y: [-0.40,  0.30,  1.10,  0.90]
    waypoint_yaw: [0.00, 1.57, 3.14, -1.57]
EOF
```

> Replace the sample pose values with your real room waypoints before Session 1.

### 4c. `../patrol_manager/launch/patrol_manager.launch.py`

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/launch/patrol_manager.launch.py << 'EOF'
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/config/patrol_manager_params.yaml',
        description='Full path to patrol manager params YAML',
    )

    patrol_node = Node(
        package='patrol_manager',
        executable='patrol_manager_node',
        name='patrol_manager',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        patrol_node,
    ])
EOF
```

### 4d. `../patrol_manager/include/patrol_manager/patrol_strategy.hpp`

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/include/patrol_manager/patrol_strategy.hpp << 'EOF'
#ifndef PATROL_MANAGER__PATROL_STRATEGY_HPP_
#define PATROL_MANAGER__PATROL_STRATEGY_HPP_

#include <string>
#include <vector>

namespace patrol_manager
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string name;
};

class PatrolStrategy
{
public:
  virtual ~PatrolStrategy() = default;
  virtual std::string name() const = 0;
  virtual void reset() = 0;
  virtual bool has_next() const = 0;
  virtual Pose2D next() = 0;
};

}  // namespace patrol_manager

#endif  // PATROL_MANAGER__PATROL_STRATEGY_HPP_
EOF
```

### 4e. `../patrol_manager/include/patrol_manager/waypoint_patrol_strategy.hpp`

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/include/patrol_manager/waypoint_patrol_strategy.hpp << 'EOF'
#ifndef PATROL_MANAGER__WAYPOINT_PATROL_STRATEGY_HPP_
#define PATROL_MANAGER__WAYPOINT_PATROL_STRATEGY_HPP_

#include <cstddef>
#include <vector>

#include "patrol_manager/patrol_strategy.hpp"

namespace patrol_manager
{

class WaypointPatrolStrategy : public PatrolStrategy
{
public:
  explicit WaypointPatrolStrategy(std::vector<Pose2D> waypoints);

  std::string name() const override;
  void reset() override;
  bool has_next() const override;
  Pose2D next() override;

private:
  std::vector<Pose2D> waypoints_;
  std::size_t index_{0};
};

}  // namespace patrol_manager

#endif  // PATROL_MANAGER__WAYPOINT_PATROL_STRATEGY_HPP_
EOF
```

### 4f. `../patrol_manager/src/waypoint_patrol_strategy.cpp`

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/src/waypoint_patrol_strategy.cpp << 'EOF'
#include "patrol_manager/waypoint_patrol_strategy.hpp"

#include <stdexcept>

namespace patrol_manager
{

WaypointPatrolStrategy::WaypointPatrolStrategy(std::vector<Pose2D> waypoints)
: waypoints_(std::move(waypoints))
{}

std::string WaypointPatrolStrategy::name() const
{
  return "waypoint_patrol";
}

void WaypointPatrolStrategy::reset()
{
  index_ = 0;
}

bool WaypointPatrolStrategy::has_next() const
{
  return index_ < waypoints_.size();
}

Pose2D WaypointPatrolStrategy::next()
{
  if (!has_next()) {
    throw std::runtime_error("WaypointPatrolStrategy::next called with no remaining waypoints");
  }
  return waypoints_[index_++];
}

}  // namespace patrol_manager
EOF
```

### 4g. `../patrol_manager/src/patrol_manager_node.cpp` (starter implementation)

This is a **Phase 4 starter**, intentionally compact. Build it, run one cycle,
then harden incrementally in Session 1–2.

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/src/patrol_manager_node.cpp << 'EOF'
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include "patrol_manager/waypoint_patrol_strategy.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::chrono_literals;

namespace patrol_manager
{

enum class State
{
  SLEEPING,
  WAKING,
  PATROLLING,
  CAPTURE,
  RETURNING,
  INVESTIGATING,
};

class PatrolManagerNode : public rclcpp::Node
{
public:
  PatrolManagerNode()
  : rclcpp::Node("patrol_manager")
  {
    // Parameters
    start_on_boot_ = declare_parameter<bool>("start_on_boot", true);
    loop_patrol_ = declare_parameter<bool>("loop_patrol", true);
    patrol_period_sec_ = declare_parameter<double>("patrol_period_sec", 1800.0);
    settle_after_arrival_sec_ = declare_parameter<double>("settle_after_arrival_sec", 1.0);

    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    mail_request_topic_ = declare_parameter<std::string>("mail_request_topic", "/cat_patrol/mail_request");
    state_topic_ = declare_parameter<std::string>("state_topic", "/patrol_manager/state");
    image_save_dir_ = declare_parameter<std::string>("image_save_dir", "/tmp/cat_patrol_images");
    mail_subject_ = declare_parameter<std::string>("mail_subject", "Cat patrol photos");
    mail_to_ = declare_parameter<std::string>("smtp_to_address", "user@example.com");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");

    const auto home_pose = declare_parameter<std::vector<double>>("home_pose", std::vector<double>{0.0, 0.0, 0.0});
    if (home_pose.size() == 3) {
      home_.x = home_pose[0];
      home_.y = home_pose[1];
      home_.yaw = home_pose[2];
      home_.name = "home";
    }

    const auto names = declare_parameter<std::vector<std::string>>("waypoint_names", std::vector<std::string>{});
    const auto xs = declare_parameter<std::vector<double>>("waypoint_x", std::vector<double>{});
    const auto ys = declare_parameter<std::vector<double>>("waypoint_y", std::vector<double>{});
    const auto yaws = declare_parameter<std::vector<double>>("waypoint_yaw", std::vector<double>{});

    if (!(names.size() == xs.size() && xs.size() == ys.size() && ys.size() == yaws.size())) {
      throw std::runtime_error("waypoint arrays must have equal sizes");
    }
    std::vector<Pose2D> waypoints;
    waypoints.reserve(names.size());
    for (std::size_t i = 0; i < names.size(); ++i) {
      waypoints.push_back(Pose2D{xs[i], ys[i], yaws[i], names[i]});
    }
    strategy_ = std::make_unique<WaypointPatrolStrategy>(std::move(waypoints));

    std::filesystem::create_directories(image_save_dir_);

    // Callback groups
    fsm_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    io_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // I/O
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = io_group_;
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr msg) { last_image_ = msg; },
      sub_opts);

    mail_pub_ = create_publisher<std_msgs::msg::String>(mail_request_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);

    // NOTE: the action client shares the MutuallyExclusive fsm_group_ so that
    // goal/feedback/result callbacks are serialized with the FSM tick. Without
    // this, result callbacks land in the node default group and race the tick
    // on state_/strategy_/cycle_image_paths_ under MultiThreadedExecutor.
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose", fsm_group_);

    // Timers
    fsm_timer_ = create_wall_timer(100ms, std::bind(&PatrolManagerNode::tick, this), fsm_group_);
    wake_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(patrol_period_sec_)),
      [this]() { wake_requested_ = true; },
      fsm_group_);

    state_ = start_on_boot_ ? State::WAKING : State::SLEEPING;
    publish_state();

    RCLCPP_INFO(get_logger(), "PatrolManager started. strategy=%s waypoints=%zu",
                strategy_->name().c_str(), names.size());
  }

private:
  static geometry_msgs::msg::PoseStamped to_pose(const Pose2D & p, const std::string & frame, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = frame;
    out.header.stamp = stamp;
    out.pose.position.x = p.x;
    out.pose.position.y = p.y;
    out.pose.orientation.z = std::sin(p.yaw / 2.0);
    out.pose.orientation.w = std::cos(p.yaw / 2.0);
    return out;
  }

  void tick()
  {
    switch (state_) {
      case State::SLEEPING:
        if (wake_requested_) {
          wake_requested_ = false;
          transition(State::WAKING);
        }
        break;

      case State::WAKING:
        cycle_image_paths_.clear();
        strategy_->reset();
        // Non-blocking readiness check — never block inside the FSM timer (see §9).
        // Stay in WAKING and re-check next tick until the server is up.
        if (!action_client_->action_server_is_ready()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "navigate_to_pose action server unavailable; staying in WAKING");
          break;
        }
        send_next_waypoint_goal();
        break;

      case State::PATROLLING:
        // Waiting for action result callback to transition.
        break;

      case State::CAPTURE:
        if (save_current_image()) {
          if (strategy_->has_next()) {
            send_next_waypoint_goal();
          } else {
            transition(State::RETURNING);
            send_home_goal();
          }
        }
        break;

      case State::RETURNING:
        // Waiting for home-goal result callback.
        break;

      case State::INVESTIGATING:
        // Reserved for Phase 6.
        break;
    }
  }

  void send_next_waypoint_goal()
  {
    if (!strategy_->has_next()) {
      transition(State::RETURNING);
      send_home_goal();
      return;
    }

    const auto wp = strategy_->next();
    auto goal = NavigateToPose::Goal();
    goal.pose = to_pose(wp, map_frame_, now());

    transition(State::PATROLLING);
    RCLCPP_INFO(get_logger(), "Navigating to waypoint '%s' x=%.2f y=%.2f yaw=%.2f",
                wp.name.c_str(), wp.x, wp.y, wp.yaw);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.feedback_callback =
      [this](GoalHandle::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "distance remaining: %.2f m", fb->distance_remaining);
      };

    opts.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          this->transition(State::CAPTURE);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Waypoint goal failed (code=%d)", static_cast<int>(result.code));
          this->transition(State::SLEEPING);
        }
      };

    action_client_->async_send_goal(goal, opts);
  }

  void send_home_goal()
  {
    auto goal = NavigateToPose::Goal();
    goal.pose = to_pose(home_, map_frame_, now());

    RCLCPP_INFO(get_logger(), "Returning home x=%.2f y=%.2f yaw=%.2f", home_.x, home_.y, home_.yaw);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          send_mail_request();
          transition(State::SLEEPING);
          if (!loop_patrol_) {
            RCLCPP_INFO(get_logger(), "loop_patrol=false, cycle complete");
          }
        } else {
          RCLCPP_ERROR(get_logger(), "Home goal failed (code=%d)", static_cast<int>(result.code));
          transition(State::SLEEPING);
        }
      };

    action_client_->async_send_goal(goal, opts);
  }

  bool save_current_image()
  {
    if (!last_image_ || last_image_->data.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No image available yet");
      return false;
    }

    try {
      // Manual sensor_msgs::Image -> cv::Mat conversion (same approach as
      // patrol_node.cpp). Avoids a cv_bridge dependency/ABI risk on this build.
      const std::string & enc = last_image_->encoding;
      const int rows = static_cast<int>(last_image_->height);
      const int cols = static_cast<int>(last_image_->width);
      const int step = static_cast<int>(last_image_->step);
      cv::Mat bgr_image;
      if (enc == "rgb8") {
        cv::Mat rgb(rows, cols, CV_8UC3, const_cast<uint8_t*>(last_image_->data.data()), step);
        cv::cvtColor(rgb, bgr_image, cv::COLOR_RGB2BGR);
      } else if (enc == "bgr8") {
        cv::Mat src(rows, cols, CV_8UC3, const_cast<uint8_t*>(last_image_->data.data()), step);
        bgr_image = src.clone();
      } else if (enc == "mono8") {
        cv::Mat mono(rows, cols, CV_8UC1, const_cast<uint8_t*>(last_image_->data.data()), step);
        cv::cvtColor(mono, bgr_image, cv::COLOR_GRAY2BGR);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
          "Unsupported encoding '%s'", enc.c_str());
        return false;
      }
      if (bgr_image.empty()) {
        return false;
      }
      std::ostringstream fn;
      fn << "snap_" << now().nanoseconds() << ".jpg";
      const auto fp = (std::filesystem::path(image_save_dir_) / fn.str()).string();
      if (cv::imwrite(fp, bgr_image)) {
        cycle_image_paths_.push_back(fp);
        RCLCPP_INFO(get_logger(), "Captured: %s", fp.c_str());
        return true;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "save_current_image exception: %s", e.what());
    }
    return false;
  }

  void send_mail_request()
  {
    std::ostringstream oss;
    oss << "{\"subject\":\"" << mail_subject_ << "\",";
    oss << "\"to\":\"" << mail_to_ << "\",";
    oss << "\"paths\":[";
    for (size_t i = 0; i < cycle_image_paths_.size(); ++i) {
      if (i > 0) {oss << ",";}
      oss << "\"" << cycle_image_paths_[i] << "\"";
    }
    oss << "]}";

    std_msgs::msg::String msg;
    msg.data = oss.str();
    mail_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published mail request with %zu images", cycle_image_paths_.size());
  }

  void transition(State s)
  {
    state_ = s;
    publish_state();
  }

  void publish_state()
  {
    std_msgs::msg::String s;
    switch (state_) {
      case State::SLEEPING: s.data = "SLEEPING"; break;
      case State::WAKING: s.data = "WAKING"; break;
      case State::PATROLLING: s.data = "PATROLLING"; break;
      case State::CAPTURE: s.data = "CAPTURE"; break;
      case State::RETURNING: s.data = "RETURNING"; break;
      case State::INVESTIGATING: s.data = "INVESTIGATING"; break;
    }
    state_pub_->publish(s);
  }

private:
  // Parameters / config
  bool start_on_boot_{true};
  bool loop_patrol_{true};
  double patrol_period_sec_{1800.0};
  double settle_after_arrival_sec_{1.0};
  std::string image_topic_;
  std::string mail_request_topic_;
  std::string state_topic_;
  std::string image_save_dir_;
  std::string mail_subject_;
  std::string mail_to_;
  std::string map_frame_;

  // State
  State state_{State::SLEEPING};
  bool wake_requested_{false};
  Pose2D home_;
  std::vector<std::string> cycle_image_paths_;

  // Concurrency
  rclcpp::CallbackGroup::SharedPtr fsm_group_;
  rclcpp::CallbackGroup::SharedPtr io_group_;

  // ROS entities
  rclcpp::TimerBase::SharedPtr fsm_timer_;
  rclcpp::TimerBase::SharedPtr wake_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mail_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  sensor_msgs::msg::Image::SharedPtr last_image_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  // Strategy
  std::unique_ptr<PatrolStrategy> strategy_;
};

}  // namespace patrol_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<patrol_manager::PatrolManagerNode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
EOF
```

### 4h. CMakeLists + package.xml (new package)

`../patrol_manager/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(patrol_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

include_directories(include)

add_library(waypoint_patrol_strategy src/waypoint_patrol_strategy.cpp)
ament_target_dependencies(waypoint_patrol_strategy rclcpp)

add_executable(patrol_manager_node src/patrol_manager_node.cpp)
target_link_libraries(patrol_manager_node waypoint_patrol_strategy ${OpenCV_LIBS})
ament_target_dependencies(patrol_manager_node
  rclcpp rclcpp_action nav2_msgs geometry_msgs sensor_msgs std_msgs
)

install(TARGETS
  waypoint_patrol_strategy
  patrol_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
```

`../patrol_manager/package.xml` key dependencies:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>patrol_manager</name>
  <version>0.0.0</version>
  <description>Nav2-native patrol manager for waypoint cycles, capture, and return-home.</description>
  <maintainer email="jetson@todo.todo">jetson</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>nav2_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>libopencv-dev</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```



### 4i. `~/myscripts2/t5_patrol.sh`

```bash
cat > ~/myscripts2/t5_patrol.sh << 'EOF'
#!/usr/bin/env bash
export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds.xml
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch patrol_manager patrol_manager.launch.py
EOF
chmod +x ~/myscripts2/t5_patrol.sh
```

### 4j. Build

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --symlink-install --packages-select patrol_manager
source install/setup.bash

# verify
ros2 pkg executables patrol_manager | grep patrol_manager_node
```

---

## 5. Architecture

```
Jetson
──────────────────────────────────────────────────────────────────────────
t1.sh  yahboomcar_bringup (driver, EKF, robot_state_publisher, joy safety)
t2.sh  lidar
t2.5.sh scan_front_filter     /scan -> /scan_filtered
t3_amcl.sh localization       map_server + amcl + lifecycle_manager_localization
t4_nav.sh  navigation         planner/controller/behaviors/bt_navigator

NEW (Phase 4)
t5_patrol.sh patrol_manager
  ├─ FSM: SLEEPING / WAKING / PATROLLING / CAPTURE / RETURNING
  ├─ Strategy: WaypointPatrolStrategy (ordered map-frame poses)
  ├─ Action client -> /navigate_to_pose (Nav2 bt_navigator)
  ├─ Sub /camera/color/image_raw
  ├─ Save JPEGs -> /tmp/cat_patrol_images
  ├─ Pub /cat_patrol/mail_request (JSON with attachment paths)
  └─ Pub /patrol_manager/state (operator observability)

Action flow:
  patrol_manager --goal--> bt_navigator
                <--feedback--
                <--result----
```

---

## 6. Pre-flight checklist

### 6a. One-time before first session

- [ ] Build `patrol_manager` and verify executable appears.
- [ ] Confirm launch args:
      `ros2 launch patrol_manager patrol_manager.launch.py --show-args`
- [ ] Fill real waypoint/home values in `patrol_manager_params.yaml`.
- [ ] Verify camera topic live:
      `ros2 topic hz /camera/color/image_raw`
- [ ] Verify mail node pipeline (already used by existing package) is active.

### 6b. Every session

- [ ] Battery above your safe threshold before autonomous cycle.
- [ ] Environment clear enough for planned waypoints.
- [ ] Bring up `t1` + `t2` + `t2.5` + `t3` + `t4` first.
- [ ] Confirm localization converged before starting patrol.
- [ ] Keep joystick ready for safety override.

---

## 7. Session procedure

### Step A — Jetson terminals (in order)

1. Bringup:
   `bash ~/myscripts2/t1.sh`
2. LiDAR:
   `bash ~/myscripts2/t2.sh`
3. Scan filter:
   `bash ~/myscripts2/t2.5.sh`
4. Localization:
   `bash ~/myscripts2/t3_amcl.sh`
5. Navigation:
   `bash ~/myscripts2/t4_nav.sh`
6. Patrol manager:
   `bash ~/myscripts2/t5_patrol.sh`

### Step B — RViz on host

- Show: Map, TF, LaserScan, Global/Local costmaps, Path, RobotModel.
- Ensure AMCL pose is reasonable before patrol begins.

### Step C — Validate patrol manager state machine

- Watch state topic:
  `ros2 topic echo /patrol_manager/state`
- Expected transitions for one full cycle:
  `SLEEPING -> WAKING -> PATROLLING -> CAPTURE -> ... -> RETURNING -> SLEEPING`

### Step D — Verify one full cycle

- Confirm each waypoint produces one `goal succeeded` in patrol logs.
- Confirm one photo file appears per waypoint in `image_save_dir`.
- Confirm one mail request message is published at cycle end.
- Confirm robot returns to configured home pose and stops.

### Step E — Verify unattended repeat

- Wait one full `patrol_period_sec` interval.
- Confirm automatic wake and second cycle start.

---

## 8. Pitfalls and recovery

- **Action server unavailable at wake**
  - Symptom: repeated “navigate_to_pose action server unavailable”.
  - Fix: start `t4_nav.sh` first; verify Nav2 lifecycle active.

- **No images captured**
  - Symptom: CAPTURE state loops with “No image available yet”.
  - Fix: verify camera topic name and frame rate; check Astra node is alive.

- **Waypoint arrays mismatch**
  - Symptom: node throws on startup.
  - Fix: keep `waypoint_names/x/y/yaw` equal length.

- **Robot cannot reach a waypoint**
  - Symptom: goal aborted repeatedly.
  - Fix: move waypoint to free space in map; check costmaps and map alignment.

- **Callback starvation / laggy state updates**
  - Symptom: delayed state changes while nav feedback is busy.
  - Fix: keep FSM timer + action result callback in the MutuallyExclusive group,
    image sub in Reentrant, run `MultiThreadedExecutor(2+)`.

- **Regression to old chassis behavior**
  - Symptom: angular.z behaves like strafe (or vice versa).
  - Fix: re-check hardware wheel orientation and baseline bringup behavior from
    Phase 3 resolution notes.

---

## 9. Concept anchors (Phase 4 learning focus)

- **FSM as orchestration layer**
  - The FSM does not drive motors directly.
  - It decides *what phase should run next*.

- **Action callbacks as event stream**
  - Goal response/feedback/result become asynchronous events.
  - Avoid blocking waits inside timer callbacks.

- **Callback groups + executor**
  - `MutuallyExclusive` (`fsm_group_`): FSM tick **and** action goal/feedback/result
    callbacks. Because the result callback mutates FSM state (`state_`, `strategy_`,
    `cycle_image_paths_`), it must be serialized with the tick — not Reentrant.
  - `Reentrant` (`io_group_`): image subscriber only.
  - `MultiThreadedExecutor`: allows both groups to progress concurrently.

- **Strategy pattern reuse**
  - `PatrolStrategy` keeps room for future patrol styles:
    random walk, perimeter sweep, threat-driven routing.
  - `WaypointPatrolStrategy` is the first concrete strategy.

---

## 10. Commands cheat-sheet

```bash
# Build only patrol_manager
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --symlink-install --packages-select patrol_manager
source install/setup.bash

# Run patrol manager directly
ros2 run patrol_manager patrol_manager_node --ros-args \
  --params-file ~/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/config/patrol_manager_params.yaml

# Run launch wrapper
ros2 launch patrol_manager patrol_manager.launch.py

# Observe state transitions
ros2 topic echo /patrol_manager/state

# Observe mail payload publication
ros2 topic echo /cat_patrol/mail_request

# Check active action server
ros2 action list | grep navigate_to_pose

# Check camera stream
ros2 topic hz /camera/color/image_raw

# Check created photos
ls -lt /tmp/cat_patrol_images | head
```

---

## 11. Phase 4 session log

### Session 1 — 2026-07-09

**Accomplished this session**
- Applied review fixes #1–#4 to this doc's code blocks (action client in
  `fsm_group_`; non-blocking `action_server_is_ready()`; manual `cv::Mat`
  conversion instead of `cv_bridge`; prose reconciled).
- Created the `patrol_manager` package at workspace level
  (`yahboomcar_ws/src/patrol_manager`) with the fixed code. Builds clean;
  `ros2 pkg executables patrol_manager` → `patrol_manager_node`. **No rebuild
  needed to change params** (launch loads the src YAML by absolute path).
- New helper scripts in `~/myscripts2/`:
  - `t5_patrol.sh`  — launch patrol_manager
  - `t6_camera.sh`  — Astra color+depth ONLY (sources `software/library_ws`;
    `uvc_product_id:=0x050f`). The staged t1–t4 do NOT start the camera.
  - `t7_mail.sh`    — `mail_node` standalone (inherits `CAT_PATROL_SMTP_*`
    from `~/.bashrc`: user `peter.botnet@gmail.com`, host smtp.gmail.com).
  - `getpose.sh`    — prints map→base_footprint as `x / y / yaw(rad)`.

**Map situation (IMPORTANT — resolved, not a mismatch)**
- The original `living_room_v1` map was wrong; remapped on the host
  (slam_toolbox in Humble Docker) → `my_map.pgm/.yaml`.
- The new map was installed on the bot by **overwriting the `living_room_v1`
  name** (kept the filename so no config change). Confirmed: `maps/living_room_v1.pgm`
  is 45655 bytes, mtime 2026-07-09 06:30 == host `my_map.pgm` size. `amcl_params.yaml`
  `yaml_filename` still points at `living_room_v1.yaml` and correctly serves the
  NEW map. **Waypoints below are valid against the loaded map.**
- Earlier "map appears 2 s then goes grey" was two `/map` publishers — a
  slam_toolbox instance still running alongside map_server. Fixed by stopping slam.

**Waypoints configured:** 10 + home, captured live by driving on the new map
(written to `patrol_manager/config/patrol_manager_params.yaml`):
| wp | x | y | yaw |
|----|------|------|------|
| 1 | 1.585 | -0.120 | -0.115 |
| 2 | 3.151 | -0.511 | -0.272 |
| 3 | 5.133 | -0.672 | 0.135 |
| 4 | 6.402 | -0.476 | 0.150 |
| 5 | 6.478 | -1.132 | 2.904 |
| 6 | 3.644 | -0.392 | 3.036 |
| 7 | 3.183 | 1.502 | 1.855 |
| 8 | 2.987 | 2.178 | 2.992 |
| 9 | 3.344 | 0.709 | -1.312 |
| 10 | 1.954 | -0.277 | -2.966 |
| home | 0.331 | -0.086 | 0.043 |

First-run config: `loop_patrol=false`, `smtp_to_address=peter.botnet@gmail.com`,
`start_on_boot=true`.

**Full cycle result:** ❌ BLOCKED — Nav2 goal for **wp5** failed (aborted).
- Robot ended at **x=6.766, y=-1.405, yaw=3.074** — *past* wp5, jammed into the
  far corner. wp4 (6.402,-0.476) and wp5 (6.478,-1.132) are both at the far
  end (x≈6.4–6.5); wp5 also demands a ~166° turnaround (yaw 2.904) hard against
  the far wall.
- **Photos captured:** wp1–wp4 reached and should have saved to
  `/tmp/cat_patrol_images` (verify count next session). Not confirmed this session.
- **Home return:** did NOT happen — on a waypoint-goal failure the current FSM
  goes straight to `SLEEPING` and abandons the cycle (no home, no email).

**Prime suspects for the wp5 abort (investigate next session, in order)**
1. **wp5 too close to the far wall / inside costmap inflation** → planner or
   controller can't reach goal tolerance. Most likely.
2. **Tight turnaround (166°) in a confined corner** → recoveries exhausted → abort.
3. Global/local costmap or inflation radius too aggressive for that corner.

**Resume plan for next session**
1. Bring up t1→t4 + t6 (camera) + t7 (mail). Confirm map is steady (single
   `/map` publisher — `ros2 topic info /map -v`), AMCL converged.
2. Inspect the abort: rerun and watch `t4_nav` logs + RViz global/local costmap
   at wp5. Note the exact result code the patrol logs print
   ("Waypoint goal failed (code=…)": 4=ABORTED, 5=CANCELED).
3. Fix options (pick based on what the costmap shows):
   - **Re-capture wp5** ~30–40 cm further from the wall and/or with a friendlier
     orientation (drive there, `getpose.sh`, edit the arrays).
   - Send wp5 goal by clicking it in RViz first to confirm Nav2 can reach it
     standalone before blaming patrol_manager.
   - Loosen `xy_goal_tolerance` / `yaw_goal_tolerance` in `nav2_params.yaml`, or
     reduce inflation near that corner.
4. **Code hardening to consider (from §14d):** on a waypoint failure, instead of
   abandoning the whole cycle (→SLEEPING), either (a) retry the goal N times, or
   (b) skip the failed waypoint and continue. Currently one bad goal kills the
   run and skips home+email. This is the biggest robustness gap.

**Notes:** Everything up to wp5 worked — package, FSM, Nav2 action client,
per-waypoint navigation, and state transitions are all functioning. The blocker
is a single unreachable/hard waypoint plus the abandon-on-failure policy, not a
structural problem.

### Session 2 — ___

- Scheduler repeat cycle: PASS / blockers ___
- Callback-group/executor behavior under load: ___
- Notes: ___

### Final results

- patrol_period_sec used: ___
- waypoint count: ___
- capture success rate per cycle: ___
- unattended cycle: PASS / blockers ___
- Phase 4 acceptance criteria: all met / blockers ___

---

## 12. Open questions / forward links

- **Phase 5 data flywheel.** Are captured waypoint photos sufficient quality
  for early cat-detector dataset bootstrapping?
- **Phase 6 interruption model.** On cat detection, do we cancel active goal
  immediately and resume from same waypoint index afterward?
- **NavigateThroughPoses vs sequential NavigateToPose.** Is stop-and-capture at
  each waypoint easier with sequential goals (usually yes for this project).
- **Phase 7 scheduling hardening.** Move timer wake-up to systemd timers once
  patrol behavior is stable.

---

## 13. Learning guide — Phase 4 deep dive

### 13a. Data-flow diagram

```text
   patrol_manager (FSM)
        │
        ├─ subscribe /camera/color/image_raw
        │         └─ save JPEG -> /tmp/cat_patrol_images/*.jpg
        │
        ├─ action client /navigate_to_pose
        │         ├─ goal -> bt_navigator
        │         ├─ feedback <- distance_remaining
        │         └─ result <- succeeded/aborted/canceled
        │
        ├─ publish /patrol_manager/state
        └─ publish /cat_patrol/mail_request (JSON with file paths)
```

### 13b. Component responsibilities

- **`patrol_manager_node`**
  - Owns FSM + schedule + action client + image capture + mail publish.
- **`WaypointPatrolStrategy`**
  - Owns waypoint ordering and iteration.
- **Nav2 stack (`t4_nav`)**
  - Owns actual path planning/control/recovery.
- **Mail node (existing pipeline)**
  - Owns SMTP sending and attachment handling.

### 13c. Why callback-group split matters

Without split, a long callback (or accidental blocking wait) can delay FSM
transitions, image updates, and action feedback handling. The split makes the
node’s behavior predictable under load.

### 13d. What to verify in RViz during Phase 4

- Path appears to each waypoint.
- Local/global costmaps stay healthy.
- Robot stops near waypoint pose before capture.
- Return-home path is valid at cycle end.

---

## 14. C++ lessons checklist (Phase 4)

### 14a. FSM and async orchestration
- [ ] Encode state transitions explicitly.
- [ ] Keep transition side effects small and deterministic.
- [ ] Handle action result codes (`SUCCEEDED` / `ABORTED` / `CANCELED`) distinctly.

### 14b. Executor + callback groups
- [ ] `MutuallyExclusive` group for FSM timer **and** action result callback
      (serialize all FSM-state mutations).
- [ ] `Reentrant` group for the image subscription only.
- [ ] Validate behavior with `MultiThreadedExecutor` thread count changes.

### 14c. Strategy pattern continuation
- [ ] Keep `PatrolStrategy` interface minimal.
- [ ] Add one future strategy stub without touching FSM logic.

### 14d. Practical hardening (next)
- [ ] Add goal timeout/cancel path with `async_cancel_goal`.
- [ ] Add retry policy for transient nav failures.
- [ ] Add capture timeout/fallback if camera stream stalls.
- [ ] Add cycle summary metrics (durations, success counts) in logs.

#include "cat_patrol_robot/patterns/classic_pattern.hpp"

namespace cat_patrol_robot
{

void ClassicPattern::on_start(PatrolContext & ctx)
{
  start_time_ = ctx.now();
  RCLCPP_INFO(ctx.get_logger(),
    "[classic] Driving forward/back for %.1f s per leg (4 legs)",
    ctx.patrol_drive_sec);
}

PatrolSignal ClassicPattern::tick(PatrolContext & ctx)
{
  if (ctx.lidar_obstacle_ahead()) {
    ctx.stop_robot();
    return PatrolSignal::Continue;
  }

  const double elapsed = (ctx.now() - start_time_).seconds();
  const double leg = ctx.patrol_drive_sec;
  const double total = leg * 4.0;

  if (elapsed >= total) {
    ctx.stop_robot();
    RCLCPP_INFO(ctx.get_logger(), "[classic] Patrol drive done");
    return PatrolSignal::DoneNextCapture;
  }

  int phase = static_cast<int>(elapsed / leg);
  double vx = (phase % 2 == 0) ? ctx.linear_speed : -ctx.linear_speed;

  RCLCPP_INFO_THROTTLE(ctx.get_logger(), *rclcpp::Clock::make_shared(), 2000,
    "[classic] %s: %.1f / %.1fs", (vx > 0 ? "FWD" : "BACK"), elapsed, total);

  ctx.publish_twist(vx, 0.0, 0.0);
  return PatrolSignal::Continue;
}

}  // namespace cat_patrol_robot

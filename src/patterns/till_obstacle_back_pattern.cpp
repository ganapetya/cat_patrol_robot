// =============================================================================
// till_obstacle_back_pattern.cpp — Implementation of the 4-phase patrol
// =============================================================================
// This file contains the METHOD BODIES for TillObstacleBackPattern.
// The class is DECLARED in the matching .hpp header file; the compiler
// needs both to build a complete program.
//
// THE FULL SEQUENCE:
//   1. DriveForward   — linear.x = speed → roll straight until depth camera
//                       reports an obstacle within depth_obstacle_min_m.
//   2. DriveBackHome  — linear.x = -speed → reverse until odometry says
//                       we're within pos_tol of the recorded home position.
//   3. CaptureImages  — spin in place taking N photos at evenly-spaced yaw
//                       targets (closed-loop, NOT timed).  Beep on each snap.
//                       After last photo, rotate back to start yaw (full 360°).
//                       Then email the photos via send_mail_request().
//   4. TurnAround     — rotate 180° (π radians) so the robot faces the
//                       opposite direction from where it started, then stop.
// =============================================================================

#include "cat_patrol_robot/patterns/till_obstacle_back_pattern.hpp"

#include <cmath>  // M_PI, std::abs, std::copysign, std::hypot, std::clamp

namespace cat_patrol_robot
{

// ===========================================================================
// normalize_angle — wrap any angle to the range (-π, +π]
// ===========================================================================
// MATH CONCEPT:
//   Angles in radians can exceed ±π.  For example, if you rotate 350°
//   counter-clockwise, the raw angle is ~6.1 radians.  But that's equivalent
//   to -10° (i.e., -0.17 rad).  Navigation code needs the SHORTEST turn
//   direction, so we normalize first.
//
//   This function keeps subtracting or adding 2π (one full turn) until
//   the angle falls within (-π, +π].
//
// C++ CONCEPT: "static" FREE FUNCTION
//   The "static" keyword here means "internal linkage" — this function is
//   only visible within this .cpp file.  Other .cpp files can have their
//   own static normalize_angle() without conflict.  This is the C++ way
//   of saying "this is a private helper, not part of the public API."
// ===========================================================================
static double normalize_angle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

// ===========================================================================
// on_start — called once when the patrol begins
// ===========================================================================
void TillObstacleBackPattern::on_start(PatrolContext & ctx)
{
  frames_saved_ = 0;
  saved_paths_.clear();
  turn_target_set_ = false;
  enter_phase(Phase::DriveForward, ctx);
  RCLCPP_INFO(ctx.get_logger(),
    "[till_obstacle_back] Pattern started — driving forward until obstacle");
}

// ===========================================================================
// on_stop — called when the patrol is interrupted (e.g., joystick override)
// ===========================================================================
void TillObstacleBackPattern::on_stop(PatrolContext & ctx)
{
  ctx.stop_robot();  // Safety: always zero velocity when stopping
}

// ===========================================================================
// enter_phase — transition to a new phase
// ===========================================================================
// DESIGN PATTERN: SINGLE TRANSITION POINT
//   ALL phase changes go through this one function.  This guarantees:
//   - The robot always stops between phases (safety)
//   - The phase timer always resets
//   - Phase-specific initialization always runs
//   Having a single transition point makes bugs much easier to find.
// ===========================================================================
void TillObstacleBackPattern::enter_phase(Phase p, PatrolContext & ctx)
{
  ctx.stop_robot();              // Safety: stop wheels before switching
  phase_ = p;                    // Update current phase
  phase_start_ = ctx.now();      // Record when this phase started

  // Phase-specific setup
  if (p == Phase::CaptureImages) {
    // Reset capture sub-state to the beginning
    capture_step_ = CaptureStep::Snap;

    // Record the robot's current heading — this is the "0° mark" for
    // the panoramic sweep.  All photo positions are computed relative to
    // this starting yaw.
    double x, y, yaw;
    if (ctx.get_odom_pose(x, y, yaw)) {
      capture_start_yaw_ = yaw;
      RCLCPP_INFO(ctx.get_logger(),
        "[till_obstacle_back] Capture start — recording yaw=%.3f rad", yaw);
    }
  }
}

// ===========================================================================
// tick — main dispatch (called 20 times per second by the node)
// ===========================================================================
// C++ CONCEPT: SWITCH STATEMENT
//   "switch (phase_)" is like a series of "if / else if" checks, but the
//   compiler can optimize it into a jump table — O(1) dispatch instead of
//   O(N) comparisons.  Each "case" label is a possible value.  The
//   "return" exits the switch and the function at the same time.
// ===========================================================================
PatrolSignal TillObstacleBackPattern::tick(PatrolContext & ctx)
{
  switch (phase_) {
    case Phase::DriveForward:   return tick_drive_forward(ctx);
    case Phase::DriveBackHome:  return tick_drive_back_home(ctx);
    case Phase::CaptureImages:  return tick_capture(ctx);
    case Phase::TurnAround:     return tick_turn_around(ctx);
    case Phase::Done:           return PatrolSignal::DoneIdle;
  }
  return PatrolSignal::DoneIdle;  // Unreachable, but keeps the compiler happy
}

// ===========================================================================
// Phase 1: DriveForward — drive straight until obstacle detected
// ===========================================================================
// ROBOTICS CONCEPT: DEPTH-CAMERA OBSTACLE DETECTION
//   The depth camera (e.g., Orbbec Astra) outputs a 2D image where each
//   pixel's value is the DISTANCE to whatever that pixel sees.  The node's
//   depth_image_cb() finds the minimum distance in a central region of the
//   depth image.  If that minimum is below depth_obstacle_min_m (e.g., 0.5 m),
//   depth_obstacle_ahead() returns true.
//
//   This is simpler and cheaper than LiDAR, but only covers the camera's
//   field of view (typically 60-90° horizontal).  LiDAR covers 360° but
//   costs more and needs a separate sensor.
// ===========================================================================
PatrolSignal TillObstacleBackPattern::tick_drive_forward(PatrolContext & ctx)
{
  // How long have we been in this phase?
  const double elapsed = (ctx.now() - phase_start_).seconds();

  // Safety timeout: if we've been driving for too long without hitting
  // anything, assume something is wrong and proceed to the next phase.
  if (elapsed > forward_timeout_sec) {
    RCLCPP_WARN(ctx.get_logger(),
      "[till_obstacle_back] Forward timeout (%.0fs) — switching to reverse", elapsed);
    enter_phase(Phase::DriveBackHome, ctx);
    return PatrolSignal::Continue;
  }

  // Check if the depth camera sees something close ahead
  if (ctx.depth_obstacle_ahead()) {
    RCLCPP_INFO(ctx.get_logger(),
      "[till_obstacle_back] Obstacle detected after %.1fs — reversing to home", elapsed);
    enter_phase(Phase::DriveBackHome, ctx);
    return PatrolSignal::Continue;
  }

  // ROS 2 CONCEPT: RCLCPP_INFO_THROTTLE
  //   Like RCLCPP_INFO but only actually prints once per N milliseconds.
  //   Without throttling, logging at 20 Hz would flood the terminal with
  //   400 lines every 20 seconds — unreadable.  The 2000 means "print
  //   at most once every 2 seconds."
  RCLCPP_INFO_THROTTLE(ctx.get_logger(), *rclcpp::Clock::make_shared(), 2000,
    "[till_obstacle_back] FWD %.1fs", elapsed);

  // Drive forward: linear.x = speed, no strafe (y=0), no turn (z=0)
  ctx.publish_twist(ctx.linear_speed, 0.0, 0.0);
  return PatrolSignal::Continue;
}

// ===========================================================================
// Phase 2: DriveBackHome — reverse to the starting position
// ===========================================================================
// ROBOTICS CONCEPT: ODOMETRY-BASED NAVIGATION
//   The robot knows its position via ODOMETRY: wheel encoders count how
//   many times each wheel has rotated, and from that, the robot calculates
//   how far it has traveled.  This position is published as a TF transform
//   from "odom" frame to "base_footprint" frame.
//
//   Odometry DRIFTS over time (wheel slip, uneven surfaces), so it's only
//   accurate over short distances.  For a patrol that drives a few meters
//   and comes back, it's good enough.  For building-scale navigation,
//   you'd need SLAM (Simultaneous Localization And Mapping) with a map.
//
// MATH CONCEPT: std::hypot(dx, dy)
//   Computes √(dx² + dy²) — the Euclidean (straight-line) distance between
//   two points.  It's more numerically stable than writing sqrt(dx*dx+dy*dy)
//   because it handles very large and very small values without overflow.
// ===========================================================================
PatrolSignal TillObstacleBackPattern::tick_drive_back_home(PatrolContext & ctx)
{
  const double elapsed = (ctx.now() - phase_start_).seconds();

  if (elapsed > drive_back_timeout_sec) {
    RCLCPP_WARN(ctx.get_logger(),
      "[till_obstacle_back] Drive-back timeout (%.0fs)", elapsed);
    enter_phase(Phase::CaptureImages, ctx);
    return PatrolSignal::Continue;
  }

  // Get current position from odometry
  double x, y, yaw;
  if (!ctx.get_odom_pose(x, y, yaw)) {
    ctx.stop_robot();  // No pose available — stop and retry next tick
    return PatrolSignal::Continue;
  }

  // Distance from current position to home
  const double dist = std::hypot(ctx.home_x - x, ctx.home_y - y);

  RCLCPP_INFO_THROTTLE(ctx.get_logger(), *rclcpp::Clock::make_shared(), 2000,
    "[till_obstacle_back] BACK dist_home=%.3f", dist);

  // Are we close enough to home?
  if (dist < ctx.pos_tol) {
    RCLCPP_INFO(ctx.get_logger(),
      "[till_obstacle_back] Reached home — starting capture");
    enter_phase(Phase::CaptureImages, ctx);
    return PatrolSignal::Continue;
  }

  // Drive backward (negative linear.x)
  ctx.publish_twist(-ctx.linear_speed, 0.0, 0.0);
  return PatrolSignal::Continue;
}

// ===========================================================================
// Phase 3: CaptureImages — 360° panoramic photo sweep
// ===========================================================================
// ROBOTICS CONCEPT: CLOSED-LOOP YAW CONTROL
//   Instead of "spin for N seconds" (open-loop, unreliable), we use
//   the odometry heading (yaw) as FEEDBACK.  The algorithm:
//
//   1. Record start_yaw when entering this phase.
//   2. For photo i (0-indexed), compute absolute target:
//        target = normalize_angle(start_yaw + i * (2π / N))
//      With N=12, each step is 2π/12 = 30° = 0.524 radians.
//   3. Use a PROPORTIONAL CONTROLLER to rotate to the target:
//        error = normalize_angle(target - current_yaw)
//        speed = clamp(error * Kp, -max, +max)
//      This means: turn fast when far from target, slow when close.
//   4. After the LAST photo (i = N-1), rotate back to start_yaw to
//      complete a full 360°.
//
// CONTROL THEORY: PROPORTIONAL CONTROLLER (P-controller)
//   The simplest feedback controller: output = Kp * error.
//     - Kp (proportional gain) = 2.0 in our case
//     - error = target_yaw - current_yaw
//     - output = angular velocity (rad/s)
//
//   When error is large (far from target): output is large → fast rotation
//   When error is small (near target): output is small → slow, precise approach
//
//   We add two refinements:
//     - clamp(output, -max, +max): prevent commanding unreasonable speeds
//     - min speed floor (0.15 rad/s): below this, the motor might STALL
//       (not enough torque to overcome friction).  std::copysign preserves
//       the direction while enforcing the minimum magnitude.
// ===========================================================================
PatrolSignal TillObstacleBackPattern::tick_capture(PatrolContext & ctx)
{
  const auto tnow = ctx.now();
  const double total_elapsed = (tnow - phase_start_).seconds();

  // Overall safety timeout for the entire capture phase
  if (total_elapsed > 120.0) {
    RCLCPP_WARN(ctx.get_logger(),
      "[till_obstacle_back] Capture timeout after %.0fs", total_elapsed);
    ctx.stop_robot();
    ctx.set_buzzer(false);
    ctx.send_mail_request();    // Email whatever we captured
    enter_phase(Phase::TurnAround, ctx);
    return PatrolSignal::Continue;
  }

  // Turn off the per-snap beep after 150 ms.
  // The beep was turned ON in the Snap case below; here we turn it OFF
  // after a short delay to produce an audible "blip."
  if (capture_buzzer_on_ && tnow >= buzzer_off_time_) {
    ctx.set_buzzer(false);
    capture_buzzer_on_ = false;
  }

  // MATH: compute the angular spacing between photos.
  // For 12 photos: step_angle = 2π/12 ≈ 0.524 rad ≈ 30°
  const double step_angle = 2.0 * M_PI / ctx.capture_frame_count;

  // How long to wait after stopping rotation, before taking the next photo.
  // This lets the camera image stabilize (no motion blur).
  const double settle_sec = 0.8;

  // Yaw tolerance for "close enough to target" during rotation.
  // Tighter than navigation tolerance (0.2 rad) because we want evenly-
  // spaced photos.  0.10 rad ≈ 5.7°.
  const double capture_yaw_tol = 0.10;

  // -----------------------------------------------------------------------
  // Capture sub-state machine: Snap → Rotate → Settle → Snap → ...
  // -----------------------------------------------------------------------
  switch (capture_step_) {

    // =====================================================================
    // CaptureStep::Snap — take a photo
    // =====================================================================
    case CaptureStep::Snap: {
      // COMPLETION CHECK: all N photos taken AND the final rotation back
      // to start_yaw is done (we arrive back here after Rotate → Settle).
      if (frames_saved_ >= ctx.capture_frame_count) {
        ctx.stop_robot();
        ctx.set_buzzer(false);
        RCLCPP_INFO(ctx.get_logger(),
          "[till_obstacle_back] Capture complete — %d images, full 360° in %.1fs",
          frames_saved_, total_elapsed);
        ctx.send_mail_request();
        enter_phase(Phase::TurnAround, ctx);
        return PatrolSignal::Continue;
      }

      // Try to save the camera's latest color frame as a JPEG
      if (ctx.save_current_image()) {
        frames_saved_++;

        // Audible feedback: short beep so you know a photo was taken
        ctx.set_buzzer(true);
        capture_buzzer_on_ = true;
        // rclcpp::Duration::from_seconds(0.15) creates a 150-millisecond duration
        buzzer_off_time_ = tnow + rclcpp::Duration::from_seconds(0.15);

        RCLCPP_INFO(ctx.get_logger(),
          "[till_obstacle_back] Captured [%d/%d]", frames_saved_, ctx.capture_frame_count);

        // Decide WHERE to rotate to next
        if (frames_saved_ < ctx.capture_frame_count) {
          // Not the last photo: rotate to the next evenly-spaced position
          //   Photo 1 → start + 1*step, Photo 2 → start + 2*step, etc.
          capture_target_yaw_ = normalize_angle(
            capture_start_yaw_ + frames_saved_ * step_angle);
        } else {
          // Last photo (index N-1) was just taken.  Rotate back to the
          // starting yaw to complete a FULL 360°.  Without this, we'd
          // only reach (N-1)*step = 330° for N=12.
          capture_target_yaw_ = capture_start_yaw_;
        }

        // Enter the Rotate sub-step
        capture_step_ = CaptureStep::Rotate;
        capture_rotate_start_ = tnow;  // For per-step timeout
        RCLCPP_INFO(ctx.get_logger(),
          "[till_obstacle_back] Rotating to yaw=%.2f (step %.0f°)",
          capture_target_yaw_, step_angle * 180.0 / M_PI);
      } else {
        // save_current_image() returned false — camera not providing data?
        // Log a warning (throttled so we don't spam) to help diagnose.
        RCLCPP_WARN_THROTTLE(ctx.get_logger(), *rclcpp::Clock::make_shared(), 3000,
          "[till_obstacle_back] save_current_image returned false — "
          "no color frame on image topic?");
      }
      break;
    }

    // =====================================================================
    // CaptureStep::Rotate — spin toward the target yaw
    // =====================================================================
    case CaptureStep::Rotate: {
      // Read current heading from odometry
      double x, y, yaw;
      if (!ctx.get_odom_pose(x, y, yaw)) {
        ctx.stop_robot();  // No TF → stop and retry next tick
        break;
      }

      // Per-step safety timeout: if one rotation takes more than 15 seconds,
      // something is wrong (stuck, TF jumping, etc.).  Skip to Settle.
      const double rotate_elapsed = (tnow - capture_rotate_start_).seconds();
      if (rotate_elapsed > 15.0) {
        RCLCPP_WARN(ctx.get_logger(),
          "[till_obstacle_back] Rotation step timeout (%.0fs) — skipping", rotate_elapsed);
        ctx.stop_robot();
        capture_step_ = CaptureStep::Settle;
        capture_settle_end_ = tnow + rclcpp::Duration::from_seconds(settle_sec);
        break;
      }

      // CONTROL MATH:
      //   error = normalize_angle(target - current)
      //   This gives the SHORTEST-PATH signed error:
      //     positive error → need to turn counter-clockwise (ROS convention)
      //     negative error → need to turn clockwise
      const double err = normalize_angle(capture_target_yaw_ - yaw);

      if (std::abs(err) < capture_yaw_tol) {
        // Close enough to target — stop and settle
        ctx.stop_robot();
        capture_step_ = CaptureStep::Settle;
        capture_settle_end_ = tnow + rclcpp::Duration::from_seconds(settle_sec);
        RCLCPP_INFO(ctx.get_logger(),
          "[till_obstacle_back] Reached target yaw (err=%.3f rad)", err);
      } else {
        // Proportional controller: w = Kp * error, clamped to max speed
        //
        // std::clamp(value, min, max) restricts value to [min, max].
        // Introduced in C++17.  Returns min if value < min, max if value > max,
        // otherwise returns value unchanged.
        double w = std::clamp(err * 2.0,
          -ctx.capture_turn_speed, ctx.capture_turn_speed);

        // Motor stall prevention: if the computed speed is very small,
        // the motor might not have enough torque to actually move.
        // std::copysign(magnitude, sign_source) gives a value with
        // the magnitude of the first argument and the sign of the second.
        //   std::copysign(0.15, -0.05) → -0.15
        //   std::copysign(0.15,  0.05) →  0.15
        if (std::abs(w) < 0.15) {
          w = std::copysign(0.15, w);
        }

        // Send rotation command: no forward motion, no strafe, just turn
        ctx.publish_twist(0.0, 0.0, w);
      }
      break;
    }

    // =====================================================================
    // CaptureStep::Settle — wait for camera to stabilize
    // =====================================================================
    // After the robot stops rotating, the camera image is still slightly
    // blurry from residual vibration and auto-exposure adjustment.
    // Waiting 0.8 seconds produces a sharper photo.
    case CaptureStep::Settle: {
      if (tnow >= capture_settle_end_) {
        capture_step_ = CaptureStep::Snap;  // Ready for next photo!
      }
      // No motion command needed — the robot is already stopped.
      break;
    }
  }

  return PatrolSignal::Continue;
}

// ===========================================================================
// Phase 4: TurnAround — rotate 180° and finish
// ===========================================================================
// After all photos are taken and emailed, the robot faces roughly the same
// direction as when capture started (because we did a full 360°).  We turn
// 180° so the robot faces the OPPOSITE direction — ready for the next patrol
// cycle to drive forward into new territory.
//
// MATH: "normalize_angle(yaw + M_PI)" adds 180° (π radians) to the current
// heading and wraps to (-π, +π].  This is the target heading.
//
// The error (target - current) is fed into a simple bang-bang controller:
//   std::copysign(angular_speed, error)
// This turns at full speed in the correct direction until the error is
// within yaw_tol (0.2 rad ≈ 11°).  Less precise than proportional control
// but fine for a 180° turn where overshoot is acceptable.
// ===========================================================================
PatrolSignal TillObstacleBackPattern::tick_turn_around(PatrolContext & ctx)
{
  double x, y, yaw;
  if (!ctx.get_odom_pose(x, y, yaw)) {
    ctx.stop_robot();
    return PatrolSignal::Continue;
  }

  // On the first tick of this phase, compute the target heading (once).
  // "turn_target_set_" prevents recomputing it every tick.
  if (!turn_target_set_) {
    turn_target_yaw_ = normalize_angle(yaw + M_PI);  // Current heading + 180°
    turn_target_set_ = true;
    RCLCPP_INFO(ctx.get_logger(),
      "[till_obstacle_back] Turning 180° — target yaw=%.2f", turn_target_yaw_);
  }

  // 30-second timeout: if the turn takes this long, something is stuck
  const double elapsed = (ctx.now() - phase_start_).seconds();
  if (elapsed > 30.0) {
    RCLCPP_WARN(ctx.get_logger(), "[till_obstacle_back] Turn timeout");
    ctx.stop_robot();
    enter_phase(Phase::Done, ctx);
    return PatrolSignal::Continue;
  }

  const double err = normalize_angle(turn_target_yaw_ - yaw);
  if (std::abs(err) < ctx.yaw_tol) {
    // Close enough — done!
    ctx.stop_robot();
    RCLCPP_INFO(ctx.get_logger(), "[till_obstacle_back] Turn complete — pattern done");
    enter_phase(Phase::Done, ctx);
    return PatrolSignal::DoneIdle;  // Tell the node: "I'm finished, go idle"
  }

  // Bang-bang controller: full speed in the direction of error
  const double w = std::copysign(ctx.angular_speed, err);
  ctx.publish_twist(0.0, 0.0, w);
  return PatrolSignal::Continue;
}

}  // namespace cat_patrol_robot

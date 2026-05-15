#ifndef CAT_PATROL_ROBOT__PATTERNS__CLASSIC_PATTERN_HPP_
#define CAT_PATROL_ROBOT__PATTERNS__CLASSIC_PATTERN_HPP_

#include "cat_patrol_robot/patterns/patrol_pattern.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cat_patrol_robot
{

class ClassicPattern : public PatrolPattern
{
public:
  void on_start(PatrolContext & ctx) override;
  PatrolSignal tick(PatrolContext & ctx) override;
  std::string name() const override { return "classic"; }

private:
  rclcpp::Time start_time_;
};

}  // namespace cat_patrol_robot

#endif  // CAT_PATROL_ROBOT__PATTERNS__CLASSIC_PATTERN_HPP_

#include "behaviortree_cpp_v3/bt_factory.h"

#include "rc_nav2_bt_nodes/compute_approach_goal.hpp"
#include "rc_nav2_bt_nodes/compute_approach_goals.hpp"
#include "rc_nav2_bt_nodes/near_goal_stopped_condition.hpp"
#include "rc_nav2_bt_nodes/reverse_steer_recovery.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rc_nav2_bt_nodes::NearGoalStoppedCondition>("NearGoalStopped");
  factory.registerNodeType<rc_nav2_bt_nodes::ComputeApproachGoal>("ComputeApproachGoal");
  factory.registerNodeType<rc_nav2_bt_nodes::ComputeApproachGoals>("ComputeApproachGoals");
  factory.registerNodeType<rc_nav2_bt_nodes::ReverseSteerRecovery>("ReverseSteerRecovery");
}

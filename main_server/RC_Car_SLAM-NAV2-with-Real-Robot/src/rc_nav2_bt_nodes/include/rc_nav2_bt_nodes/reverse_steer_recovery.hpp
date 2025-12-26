#ifndef RC_NAV2_BT_NODES__REVERSE_STEER_RECOVERY_HPP_
#define RC_NAV2_BT_NODES__REVERSE_STEER_RECOVERY_HPP_

#include <atomic>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rc_nav2_bt_nodes
{

class ReverseSteerRecovery : public BT::StatefulActionNode
{
public:
  ReverseSteerRecovery(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  ~ReverseSteerRecovery() override;

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool initPublisher();
  bool computeManeuver();
  void publishCmd(double linear, double angular);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Time start_time_;
  rclcpp::Time phase_start_time_;
  double phase_duration_{0.0};
  bool two_phase_{false};
  int phase_{0};
  double steer_sign_{1.0};
  double reverse_speed_{0.2};
  double max_steer_{0.8};
};

}  // namespace rc_nav2_bt_nodes

#endif  // RC_NAV2_BT_NODES__REVERSE_STEER_RECOVERY_HPP_

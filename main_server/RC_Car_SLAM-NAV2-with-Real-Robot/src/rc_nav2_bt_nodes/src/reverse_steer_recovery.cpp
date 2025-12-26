#include "rc_nav2_bt_nodes/reverse_steer_recovery.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

namespace rc_nav2_bt_nodes
{

ReverseSteerRecovery::ReverseSteerRecovery(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf),
  tf_buffer_(rclcpp::Clock::make_shared()),
  tf_listener_(tf_buffer_)
{
  node_ = rclcpp::Node::make_shared("reverse_steer_recovery");
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  executor_thread_ = std::thread([this]() {executor_.spin();});
}

ReverseSteerRecovery::~ReverseSteerRecovery()
{
  executor_.cancel();
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

BT::PortsList ReverseSteerRecovery::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<std::string>("global_frame", "map", "Global frame"),
    BT::InputPort<std::string>("base_frame", "rc_car/base_link_rot", "Robot base frame"),
    BT::InputPort<double>("next_point_radius", 0.3, "Nearest point radius (m)"),
    BT::InputPort<double>("angle_threshold", 0.52, "Angle threshold (rad)"),
    BT::InputPort<double>("reverse_distance", 0.3, "Reverse distance (m)"),
    BT::InputPort<double>("reverse_speed", 0.2, "Reverse speed (m/s)"),
    BT::InputPort<double>("max_steer", 0.8, "Max steering command (rad/s)"),
    BT::InputPort<std::string>("cmd_vel_topic", "/rc_car/cmd_vel", "cmd_vel topic")
  };
}

bool ReverseSteerRecovery::initPublisher()
{
  if (cmd_pub_) {
    return true;
  }

  std::string cmd_vel_topic;
  if (!getInput("cmd_vel_topic", cmd_vel_topic)) {
    return false;
  }

  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS(10));
  return true;
}

void ReverseSteerRecovery::publishCmd(double linear, double angular)
{
  if (!cmd_pub_) {
    return;
  }
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear;
  cmd.angular.z = angular;
  cmd_pub_->publish(cmd);
}

bool ReverseSteerRecovery::computeManeuver()
{
  std::string global_frame;
  std::string base_frame;
  double next_point_radius = 0.3;
  double angle_threshold = 0.52;
  double reverse_distance = 0.3;

  getInput("global_frame", global_frame);
  getInput("base_frame", base_frame);
  getInput("next_point_radius", next_point_radius);
  getInput("angle_threshold", angle_threshold);
  getInput("reverse_distance", reverse_distance);
  getInput("reverse_speed", reverse_speed_);
  getInput("max_steer", max_steer_);

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(global_frame, base_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    RCLCPP_WARN(node_->get_logger(), "TF lookup failed for %s -> %s", global_frame.c_str(), base_frame.c_str());
    return false;
  }

  const double robot_x = tf.transform.translation.x;
  const double robot_y = tf.transform.translation.y;
  const double robot_yaw = yawFromQuat(tf.transform.rotation);
  const double forward_x = std::cos(robot_yaw);
  const double forward_y = std::sin(robot_yaw);

  nav_msgs::msg::Path path;
  bool has_path = getInput("path", path) && !path.poses.empty();
  geometry_msgs::msg::PoseStamped target;

  if (has_path) {
    double closest = std::numeric_limits<double>::max();
    double best_proj = -std::numeric_limits<double>::max();
    bool found_forward = false;

    for (const auto & pose : path.poses) {
      const double dx = pose.pose.position.x - robot_x;
      const double dy = pose.pose.position.y - robot_y;
      const double dist = std::hypot(dx, dy);
      if (dist < closest) {
        closest = dist;
      }
      if (dist <= next_point_radius) {
        const double proj = dx * forward_x + dy * forward_y;
        if (proj > 0.0 && proj > best_proj) {
          best_proj = proj;
          target = pose;
          found_forward = true;
        }
      }
    }

    if (!found_forward) {
      geometry_msgs::msg::PoseStamped goal;
      if (getInput("goal", goal)) {
        target = goal;
        RCLCPP_INFO(node_->get_logger(),
          "No forward path point within %.2f m (closest %.2f m). Using goal for recovery direction.",
          next_point_radius, closest);
      } else {
        RCLCPP_INFO(node_->get_logger(),
          "No forward path point within %.2f m (closest %.2f m). Using nearest path point.",
          next_point_radius, closest);
      }
    } else {
      RCLCPP_INFO(node_->get_logger(),
        "Selected forward path point within %.2f m (proj=%.3f).",
        next_point_radius, best_proj);
    }
  } else {
    if (!getInput("goal", target)) {
      RCLCPP_WARN(node_->get_logger(), "No path or goal provided to recovery.");
      return false;
    }
  }

  if (!path.header.frame_id.empty() && path.header.frame_id != global_frame) {
    try {
      geometry_msgs::msg::TransformStamped tf_to_global =
        tf_buffer_.lookupTransform(global_frame, path.header.frame_id, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped transformed;
      tf2::doTransform(target, transformed, tf_to_global);
      target = transformed;
    } catch (const tf2::TransformException &) {
      RCLCPP_WARN(node_->get_logger(),
        "Failed to transform target pose from %s to %s",
        path.header.frame_id.c_str(), global_frame.c_str());
      return false;
    }
  } else if (!target.header.frame_id.empty() && target.header.frame_id != global_frame) {
    try {
      geometry_msgs::msg::TransformStamped tf_to_global =
        tf_buffer_.lookupTransform(global_frame, target.header.frame_id, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped transformed;
      tf2::doTransform(target, transformed, tf_to_global);
      target = transformed;
    } catch (const tf2::TransformException &) {
      RCLCPP_WARN(node_->get_logger(),
        "Failed to transform goal pose from %s to %s",
        target.header.frame_id.c_str(), global_frame.c_str());
      return false;
    }
  }

  const double dx = target.pose.position.x - robot_x;
  const double dy = target.pose.position.y - robot_y;
  if (std::hypot(dx, dy) < 1e-4) {
    RCLCPP_WARN(node_->get_logger(), "Target too close; skipping recovery.");
    return false;
  }

  const double path_yaw = std::atan2(dy, dx);
  const double angle_diff = normalizeAngle(path_yaw - robot_yaw);

  two_phase_ = std::abs(angle_diff) <= angle_threshold;
  steer_sign_ = (angle_diff >= 0.0) ? 1.0 : -1.0;

  if (!two_phase_) {
    // Reverse steer away from the path direction.
    steer_sign_ *= -1.0;
  }

  const double total_time = reverse_distance / std::max(reverse_speed_, 1e-3);
  phase_duration_ = two_phase_ ? (total_time * 0.5) : total_time;
  phase_ = 0;

  RCLCPP_INFO(node_->get_logger(),
    "Recovery mode: %s, angle_diff=%.3f rad, steer=%.2f, reverse_speed=%.2f",
    two_phase_ ? "two-phase" : "single-phase",
    angle_diff, steer_sign_ * max_steer_, reverse_speed_);

  return true;
}

BT::NodeStatus ReverseSteerRecovery::onStart()
{
  if (!initPublisher()) {
    return BT::NodeStatus::FAILURE;
  }

  if (!computeManeuver()) {
    return BT::NodeStatus::FAILURE;
  }

  start_time_ = node_->get_clock()->now();
  phase_start_time_ = start_time_;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReverseSteerRecovery::onRunning()
{
  const auto now = node_->get_clock()->now();
  const double phase_elapsed = (now - phase_start_time_).seconds();

  if (two_phase_ && phase_ == 0 && phase_elapsed >= phase_duration_) {
    phase_ = 1;
    phase_start_time_ = now;
    return BT::NodeStatus::RUNNING;
  }

  if ((!two_phase_ && phase_elapsed >= phase_duration_) ||
    (two_phase_ && phase_ == 1 && phase_elapsed >= phase_duration_))
  {
    publishCmd(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  const double phase_sign = (two_phase_ && phase_ == 1) ? -steer_sign_ : steer_sign_;
  publishCmd(-reverse_speed_, phase_sign * max_steer_);
  return BT::NodeStatus::RUNNING;
}

void ReverseSteerRecovery::onHalted()
{
  publishCmd(0.0, 0.0);
}

}  // namespace rc_nav2_bt_nodes

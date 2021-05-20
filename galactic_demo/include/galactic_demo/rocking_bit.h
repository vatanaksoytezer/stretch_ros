/*******************************************************
 * Copyright (C) 2021 Picknik Robotics
 *
 * This file can not be copied and/or distributed without the express
 * permission of Picknik Robotics.
 *******************************************************/

#ifndef GALACTIC_DEMO_ROCKING_BIT_H
#define GALACTIC_DEMO_ROCKING_BIT_H

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace galactic_demo
{
class RockingBit : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  explicit RockingBit(const rclcpp::NodeOptions & options);
  void send_goal(const FollowJointTrajectory::Goal& goal);

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_ptr_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

  double arm_swing_ {0.1};
  double swing_duration_ {10.0};
  double base_pos_ {-1.0};
  bool stopped_ {false};
  bool released_ {false};

  void js_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void stop_rockin(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response>      response);
  void goal_response_callback(std::shared_future<GoalHandle::SharedPtr> future);
  void result_callback(const GoalHandle::WrappedResult & result);
};
}  // namespace galactic_demo

#endif  // GALACTIC_DEMO_ROCKING_BIT_H

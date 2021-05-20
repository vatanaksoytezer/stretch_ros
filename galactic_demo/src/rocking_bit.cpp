/*******************************************************
 * Copyright (C) 2021 Picknik Robotics
 *
 * This file can not be copied and/or distributed without the express
 * permission of Picknik Robotics.
 *******************************************************/

#include <galactic_demo/rocking_bit.h>

namespace galactic_demo
{
using namespace std::placeholders;

trajectory_msgs::msg::JointTrajectoryPoint makePoint(double t, double position,
                                                     double velocity = 0.0, double acceleration = 0.0)
{
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions.push_back(position);
  pt.velocities.push_back(velocity);
  pt.accelerations.push_back(acceleration);
  pt.time_from_start = rclcpp::Duration::from_seconds(t);
  return pt;
}

RockingBit::RockingBit(const rclcpp::NodeOptions & options)
  : Node("trajectory_action_client", options)
{
    service_ptr_ = create_service<std_srvs::srv::Trigger>("stop_rockin", std::bind(&RockingBit::stop_rockin,
                                                                                   this, _1, _2));
    client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(this,
                                                                      "/stretch_controller/follow_joint_trajectory");
    subscription_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
                                                                      std::bind(&RockingBit::js_cb, this, _1));
}

void RockingBit::stop_rockin(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    RCLCPP_INFO(this->get_logger(), "Stop");
    stopped_ = true;
    response->success = true;
    response->message = "Stopped.";
}

void RockingBit::js_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (base_pos_ >= 0.0)
    {
        return;
    }

    double total = 0.0;
    for (unsigned int i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i].rfind("joint_arm_l") == 0)
        {
           total += msg->position[i];
        }
    }
    base_pos_ = total;
    RCLCPP_INFO(this->get_logger(), "Arm position: %.2f", base_pos_);

    // Send Initial
    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names.push_back("wrist_extension");
    goal.trajectory.points.push_back(makePoint(0.0, base_pos_));
    goal.trajectory.points.push_back(makePoint(swing_duration_ * 0.5, base_pos_ + arm_swing_));
    goal.trajectory.points.push_back(makePoint(swing_duration_ * 1.5, base_pos_ - arm_swing_));
    send_goal(goal);

    subscription_.reset();
}

void RockingBit::send_goal(const FollowJointTrajectory::Goal& goal)
{
    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&RockingBit::goal_response_callback, this, _1);
    send_goal_options.result_callback = std::bind(&RockingBit::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal, send_goal_options);
}

void RockingBit::goal_response_callback(std::shared_future<GoalHandle::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void RockingBit::result_callback(const GoalHandle::WrappedResult & result)
{
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    if (released_)
    {
        RCLCPP_INFO(this->get_logger(), "Fully complete");
        rclcpp::shutdown();
        return;
    }
    auto goal = FollowJointTrajectory::Goal();
    if (!stopped_)
    {
        // Continue with full swings
        RCLCPP_INFO(this->get_logger(), "Swing Again");
        goal.trajectory.joint_names.push_back("wrist_extension");
        goal.trajectory.points.push_back(makePoint(0.0, base_pos_ - arm_swing_));
        goal.trajectory.points.push_back(makePoint(swing_duration_, base_pos_ + arm_swing_));
        goal.trajectory.points.push_back(makePoint(swing_duration_ * 2.0, base_pos_ - arm_swing_));
    }
    else
    {
        // open hand
        RCLCPP_INFO(this->get_logger(), "Stopped");
        released_ = true;
        goal.trajectory.joint_names.push_back("wrist_extension");
        goal.trajectory.joint_names.push_back("joint_gripper_finger_left");
        goal.trajectory.joint_names.push_back("joint_gripper_finger_right");

        // use helper to set the duration/wrist pose
        goal.trajectory.points.push_back(makePoint(0.0, base_pos_ - arm_swing_));
        goal.trajectory.points.push_back(makePoint(swing_duration_ * 0.5, base_pos_));
        goal.trajectory.points.push_back(makePoint(swing_duration_ * 0.5 + 3.0, base_pos_));
        goal.trajectory.points.push_back(makePoint(swing_duration_ * 0.5 + 6.0, 0.0));

        // One position for each half of the gripper
        goal.trajectory.points[0].positions.push_back(0.0);
        goal.trajectory.points[0].positions.push_back(0.0);

        goal.trajectory.points[1].positions.push_back(0.0);
        goal.trajectory.points[1].positions.push_back(0.0);

        goal.trajectory.points[2].positions.push_back(0.1);
        goal.trajectory.points[2].positions.push_back(0.1);

        goal.trajectory.points[3].positions.push_back(0.1);
        goal.trajectory.points[3].positions.push_back(0.1);

        for (auto& point : goal.trajectory.points)
        {
            point.velocities.clear();
            point.accelerations.clear();
        }
    }
    send_goal(goal);
}

}  // namespace galactic_demo

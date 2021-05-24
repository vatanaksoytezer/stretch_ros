/*******************************************************
 * Copyright (C) 2021 Picknik Robotics
 *
 * This file can not be copied and/or distributed without the express
 * permission of Picknik Robotics.
 *******************************************************/

#ifndef GALACTIC_DEMO_STRETCH_TASK_H
#define GALACTIC_DEMO_STRETCH_TASK_H

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>

namespace galactic_demo
{
class StretchTask
{
public:
  StretchTask(const rclcpp::Node::SharedPtr &node);
  bool plan();
  bool execute();
protected:
  moveit::task_constructor::TaskUniquePtr task_;
  moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner_;
  moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner_;

  const std::string group_name_ { "mobile_base_arm" };
  const std::string end_effector_name_ { "gripper" };
  const std::string hand_group_name_ { "gripper" };
  const std::string hand_frame_ { "link_grasp_center" };

  const std::string hand_open_pose_ { "open" };
  const std::string hand_close_pose_ { "closed" };
  const std::string frame_id_ { "odom" };
};
}  // namespace galactic_demo

#endif  // GALACTIC_DEMO_STRETCH_TASK_H

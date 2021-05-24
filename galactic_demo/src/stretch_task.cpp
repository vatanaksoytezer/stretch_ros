/*******************************************************
 * Copyright (C) 2021 Picknik Robotics
 *
 * This file can not be copied and/or distributed without the express
 * permission of Picknik Robotics.
 *******************************************************/

#include <galactic_demo/stretch_task.h>

namespace galactic_demo
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("stretch_task");

StretchTask::StretchTask(const rclcpp::Node::SharedPtr &node)
  : sampling_planner_(std::make_unique<moveit::task_constructor::solvers::PipelinePlanner>(node)),
    cartesian_planner_(std::make_unique<moveit::task_constructor::solvers::CartesianPath>())
{
  task_ = std::make_unique<moveit::task_constructor::Task>();
  task_->loadRobotModel(node);
  task_->setProperty("group", group_name_);
  task_->setProperty("eef", end_effector_name_);
  task_->setProperty("hand", hand_group_name_);
  task_->setProperty("ik_frame", hand_frame_);
}


bool StretchTask::plan()
{
  RCLCPP_INFO(LOGGER, "Start searching for task solutions");
  task_->enableIntrospection();
  try
  {
    task_->plan(5);
  }
  catch (const moveit::task_constructor::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed: " << e);
    return false;
  }
  if (task_->numSolutions() == 0)
  {
    RCLCPP_ERROR(LOGGER, "Planning failed");
    return false;
  }
  return true;
}

bool StretchTask::execute()
{
  RCLCPP_INFO(LOGGER, "Executing solution trajectory");
  moveit_msgs::msg::MoveItErrorCodes execute_result = task_->execute(*task_->solutions().front());

  if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << execute_result.val);
    return false;
  }

  return true;
}
}  // namespace galactic_demo

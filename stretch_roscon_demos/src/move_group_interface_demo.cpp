/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

/* #include <moveit_visual_tools/moveit_visual_tools.h>  This has not been ported to ros2 yet */
#include <rviz_visual_tools/rviz_visual_tools.hpp>
/* this is a standin for moveit_visual_tools visual_tools.prompt */
#include <moveit/macros/console_colors.h>
void prompt(const std::string& message)
{
  printf(MOVEIT_CONSOLE_COLOR_GREEN "\n%s" MOVEIT_CONSOLE_COLOR_RESET, message.c_str());
  fflush(stdout);
  while (std::cin.get() != '\n' && rclcpp::ok())
    ;
}

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_interface_demo");

void poseMsgToEigen(const geometry_msgs::msg::Pose& msg, Eigen::Isometry3d& out)
{
  Eigen::Translation3d translation(msg.position.x, msg.position.y, msg.position.z);
  Eigen::Quaterniond quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  if ((quaternion.x() == 0) && (quaternion.y() == 0) && (quaternion.z() == 0) && (quaternion.w() == 0))
  {
    RCLCPP_WARN(LOGGER, "Empty quaternion found in pose message. Setting to neutral orientation.");
    quaternion.setIdentity();
  }
  else
  {
    quaternion.normalize();
  }
  out = translation * quaternion;
}

void initializeMoveGroup(moveit::planning_interface::MoveGroupInterface& move_group)
{
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(10);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_demo", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, "mobile_base_arm");

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


  // Setting Initial Planning Parameters
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  initializeMoveGroup(move_group);
  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  prompt("Press 'Enter' to start the demo");

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  move_group.setStartStateToCurrentState();
  
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.position.x = -0.058876442274373234;
  target_pose1.position.y = -0.6686726215135715;
  target_pose1.position.z = 0.9947614781407926;
  target_pose1.orientation.x = 2.3418130500298247e-06;
  target_pose1.orientation.y = -1.9460595924624177e-07;
  target_pose1.orientation.z = 0.007628164292894081;
  target_pose1.orientation.w = 0.9999709051287435;
  Eigen::Isometry3d approx_target = Eigen::Isometry3d::Identity();
  poseMsgToEigen(target_pose1, approx_target);
  
  move_group.setApproximateJointValueTarget(approx_target, move_group.getEndEffectorLink());

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are planning and asking move_group
  // to actually move the robot. 
  // If we wanted to plan only we would use move.group.plan(plan) instead
  bool success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Plan (pose goal) %s", success ? "" : "FAILED");


  prompt("Press 'Enter' to continue the demo");

  move_group.setStartStateToCurrentState();
  
  target_pose1.position.x = -0.358876442274373234;
  target_pose1.position.y = -0.6686726215135715;
  target_pose1.position.z = 0.7947614781407926;
  target_pose1.orientation.x = 2.3418130500298247e-06;
  target_pose1.orientation.y = -1.9460595924624177e-07;
  target_pose1.orientation.z = 0.007628164292894081;
  target_pose1.orientation.w = 0.9999709051287435;
  poseMsgToEigen(target_pose1, approx_target);
  
  move_group.setApproximateJointValueTarget(approx_target, move_group.getEndEffectorLink());

  prompt("Press 'Enter' to end the demo");
  rclcpp::shutdown();
  return 0;
}

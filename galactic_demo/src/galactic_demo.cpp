/*******************************************************
 * Copyright (C) 2021 Picknik Robotics
 *
 * This file can not be copied and/or distributed without the express
 * permission of Picknik Robotics.
 *******************************************************/

#include <rclcpp/rclcpp.hpp>


namespace galactic_demo
{
}  // namespace galactic_demo


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("galactic_demo", "", options);
  std::thread spinning_thread([node] { rclcpp::spin(node); });

  /*

  auto psm = loadPlanningSceneMonitor(node);
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(psm);
    scene->processCollisionObjectMsg(createTable(
        "table_source",
        tf2::toMsg(Eigen::Isometry3d(Eigen::Translation3d(1, 1, 0)))));
    scene->processCollisionObjectMsg(createTable(
        "table_target",
        tf2::toMsg(Eigen::Isometry3d(Eigen::Translation3d(1, -1, 0)))));
    scene->processCollisionObjectMsg(createObject(
        tf2::toMsg(Eigen::Isometry3d(Eigen::Translation3d(1, 1, 0.25)))));
  }  // Unlock PlanningScene
  psm->triggerSceneUpdateEvent(
      planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  rclcpp::sleep_for(500ms);

  PickPlaceTask::Parameters pick_place_parameters;
  //  pick_place_parameters.arm_group_name = "stretch_arm";
  pick_place_parameters.hand_group_name = "gripper";
  pick_place_parameters.end_effector_name = "gripper";
  pick_place_parameters.hand_frame = "link_grasp_center";
  pick_place_parameters.object_name = "object";
  pick_place_parameters.hand_open_pose = "open";
  pick_place_parameters.hand_close_pose = "closed";
  pick_place_parameters.mobile_base_arm_group_name = "mobile_base_arm";
  pick_place_parameters.place_pose.header.frame_id = "odom";
  pick_place_parameters.place_pose.pose =
      tf2::toMsg(Eigen::Isometry3d(Eigen::Translation3d(1, -1, 0.25 + 0.05)));

  PickPlaceTask pick_place_task(node, pick_place_parameters);
  if (!pick_place_task.plan()) {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to plan");
  }

  // Keep introspection alive
  spinning_thread.join();
  */
}

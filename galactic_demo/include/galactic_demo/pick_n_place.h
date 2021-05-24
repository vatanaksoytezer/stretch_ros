/*******************************************************
 * Copyright (C) 2021 Picknik Robotics
 *
 * This file can not be copied and/or distributed without the express
 * permission of Picknik Robotics.
 *******************************************************/

#ifndef GALACTIC_DEMO_PICK_N_PLACE_H
#define GALACTIC_DEMO_PICK_N_PLACE_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace galactic_demo
{
class PickNPlace : public rclcpp::Node
{
public:
  explicit PickNPlace(const rclcpp::NodeOptions & options);
  void initialize();
protected:
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
};
}  // namespace galactic_demo

#endif  // GALACTIC_DEMO_PICK_N_PLACE_H

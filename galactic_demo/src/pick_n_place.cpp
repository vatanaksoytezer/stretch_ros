/*******************************************************
 * Copyright (C) 2021 Picknik Robotics
 *
 * This file can not be copied and/or distributed without the express
 * permission of Picknik Robotics.
 *******************************************************/

#include <galactic_demo/pick_n_place.h>
#include <galactic_demo/stretch_task.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/task_constructor/stages.h>

namespace galactic_demo
{
using namespace std::chrono_literals;  // ms operator

moveit_msgs::msg::CollisionObject createBox(const std::string& object_name, double width, double depth, double height,
                                            const geometry_msgs::msg::Pose& pose)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = "odom";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions.resize(
    geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::BOX>()
  );
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::BOX_X) = width;
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::BOX_Y) = depth;
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::BOX_Z) = height;
  object.primitive_poses.push_back(pose);
  object.primitive_poses.resize(1);
  object.primitive_poses.back().position.z += 0.5 * height;  // align surface with world
  return object;
}

moveit_msgs::msg::CollisionObject createBox(const std::string& object_name, double width, double depth, double height,
                                            double x, double y, double z, double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return createBox(object_name, width, depth, height, pose);
}

using moveit::task_constructor::Stage;

class PickTask : public StretchTask
{
public:
  PickTask(const rclcpp::Node::SharedPtr &node, const std::string& object_name)
    : StretchTask(node)
  {
    Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
    {
      auto current_state = std::make_unique<moveit::task_constructor::stages::CurrentState>("current state");
      current_state_ptr = current_state.get();
      task_->add(std::move(current_state));
    }

    /** Open Hand **/
    {
      auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("open hand", sampling_planner_);
      stage->setGroup(hand_group_name_);
      stage->setGoal(hand_open_pose_);
      task_->add(std::move(stage));
    }

    /** Move To Object's Pose **/
    {
      /*auto stage = std::make_unique<moveit::task_constructor::stages::Connect>
        ("move to object pose",
         //moveit::task_constructor::stages::Connect::GroupPlannerVector{
        // {group_name_, sampling_planner_}}
        );
      stage->properties().configureInitFrom(Stage::PARENT);
      task_->add(std::move(stage));*/
    }

    /** Pick Object **/
    {
      auto grasp = std::make_unique<moveit::task_constructor::SerialContainer>("pick object");
      task_->properties().exposeTo(grasp->properties(), {"eef", "hand", "group", "ik_frame"});
      grasp->properties().configureInitFrom(Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

      /** Approach Object **/
      {
        auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("approach object",
                                                                                      cartesian_planner_);
        stage->properties().configureInitFrom(Stage::PARENT, {"group"});
        stage->setIKFrame(hand_frame_);
        stage->setMinMaxDistance(0.1, 0.15);
        stage->properties().set("marker_ns", "approach_object");

        // Set hand forward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "odom";
        vec.vector.z = -1.0;
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
      }

      /** Generate Grasp Pose **/
      {
        // Sample grasp pose
        auto stage = std::make_unique<moveit::task_constructor::stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose");
        stage->setPreGraspPose(hand_open_pose_);
        stage->setObject(object_name);
        stage->setAngleDelta(M_PI / 12);
        stage->setMonitoredStage(current_state_ptr); // Hook into current state

        // Compute IK
        auto wrapper = std::make_unique<moveit::task_constructor::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(hand_frame_);
        wrapper->properties().configureInitFrom(Stage::PARENT, {"eef", "group"});
        wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
        grasp->insert(std::move(wrapper));
      }

      /** Allow Collision (hand object) **/
      {
        auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>(
                       "allow collision (hand,object)");
        stage->allowCollisions(object_name,
                               task_->getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
                               true);
        grasp->insert(std::move(stage));
      }

      /** Close Hand **/
      {
        auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("close hand", sampling_planner_);
        stage->setGroup(hand_group_name_);
        stage->setGoal(hand_close_pose_);
        grasp->insert(std::move(stage));
      }

      /** Attach Object **/
      {
        auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("attach object");
        stage->attachObject(object_name, hand_frame_);
        grasp->insert(std::move(stage));
      }

      /** Allow collision (object support) **/
      {
        auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>(
                       "allow collision (object,support)");
        stage->allowCollisions({object_name}, "table_source", true);
        grasp->insert(std::move(stage));
      }

      /** Lift object **/
      {
        auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("lift object",
                     cartesian_planner_);
        stage->properties().configureInitFrom(Stage::PARENT, {"group"});
        stage->setMinMaxDistance(0.01, 0.1);
        stage->setIKFrame(hand_frame_);
        stage->properties().set("marker_ns", "lift_object");

        // Set upward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "odom";
        vec.vector.z = 1.0;
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
      }

      /** Forbid collision (object support) **/
      {
        auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>(
                       "forbid collision (object,surface)");
        stage->allowCollisions({object_name}, "table_source", false);
        grasp->insert(std::move(stage));
      }

      // Add grasp container to task
      task_->add(std::move(grasp));
    }
  }
};


class PlaceTask : public StretchTask
{
public:
  PlaceTask(const rclcpp::Node::SharedPtr &node, const std::string& object_name,
            const geometry_msgs::msg::PoseStamped& place_pose)
    : StretchTask(node)
  {
    Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
    {
      auto current_state = std::make_unique<moveit::task_constructor::stages::CurrentState>("current state");
      current_state_ptr = current_state.get();
      task_->add(std::move(current_state));
    }

    /** Move to Place **/
    {
      auto stage = std::make_unique<moveit::task_constructor::stages::Connect>(
                     "move to place",
                     moveit::task_constructor::stages::Connect::GroupPlannerVector
      {
        {group_name_, sampling_planner_}});
      stage->setTimeout(5.0);
      stage->properties().configureInitFrom(Stage::PARENT);
      task_->add(std::move(stage));
    }

    /** Place Object **/
    {
      auto place = std::make_unique<moveit::task_constructor::SerialContainer>("place object");
      task_->properties().exposeTo(place->properties(), {"eef", "hand", "group"});
      place->properties().configureInitFrom(Stage::PARENT,
      {"eef", "hand", "group"});

      /** Lower Object **/
      {
        auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("lower object",
                     cartesian_planner_);
        stage->properties().set("marker_ns", "lower_object");
        stage->setIKFrame(hand_frame_);
        stage->properties().configureInitFrom(Stage::PARENT, {"group"});
        stage->setMinMaxDistance(.01, .1);

        // Set downward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "odom";
        vec.vector.z = -1.0;
        stage->setDirection(vec);
        place->insert(std::move(stage));
      }

      /** Generate Place Pose **/
      {
        // Generate Place Pose
        auto stage =
          std::make_unique<moveit::task_constructor::stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(Stage::PARENT, {"ik_frame"});
        stage->properties().set("marker_ns", "place_pose");
        stage->setObject(object_name);

        // Set target pose
        stage->setPose(place_pose);
        stage->setMonitoredStage(current_state_ptr); // Hook into attach_object_stage

        // Compute IK
        auto wrapper = std::make_unique<moveit::task_constructor::stages::ComputeIK>("place pose IK",
                       std::move(stage));
        wrapper->setMaxIKSolutions(2);
        wrapper->setIKFrame(hand_frame_);
        wrapper->properties().configureInitFrom(Stage::PARENT, {"eef", "group"});
        wrapper->properties().configureInitFrom(Stage::INTERFACE,
        {"target_pose"});
        place->insert(std::move(wrapper));
      }

      /** Open Hand **/
      {
        auto stage =
          std::make_unique<moveit::task_constructor::stages::MoveTo>("open hand", sampling_planner_);
        stage->setGroup(hand_group_name_);
        stage->setGoal(hand_open_pose_);
        place->insert(std::move(stage));
      }

      /** Forbid collision (hand, object) **/
      {
        auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>(
                       "forbid collision (hand,object)");
        stage->allowCollisions(
          object_name,
          task_->getRobotModel()
          ->getJointModelGroup(hand_group_name_)
          ->getLinkModelNamesWithCollisionGeometry(),
          false);
        place->insert(std::move(stage));
      }

      /** Detach Object **/
      {
        auto stage =
          std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("detach object");
        stage->detachObject(object_name, hand_frame_);
        place->insert(std::move(stage));
      }

      // Add place container to task
      task_->add(std::move(place));
    }
  }
};

PickNPlace::PickNPlace(const rclcpp::NodeOptions & options)
  : Node("pick_n_place", options)
{
}

void PickNPlace::initialize()
{
  auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(),
                                                                                   "robot_description");
  psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), robot_model_loader);

  if (psm_->getPlanningScene())
  {
    psm_->startStateMonitor();
    psm_->requestPlanningSceneState();
    // Wait for complete state to be received
    psm_->getStateMonitor()->waitForCurrentState(now());
    psm_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                       planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_TOPIC);
  }

  // Lock and Load Objects
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(psm_);
    scene->processCollisionObjectMsg(createBox("table_source", 0.2, 0.2, 0.25, 1, 1, 0));
    scene->processCollisionObjectMsg(createBox("table_target", 0.2, 0.2, 0.25, 1, -1, 0));
    scene->processCollisionObjectMsg(createBox("object", 0.04, 0.04, 0.1, 1, 1, 0.25));
  }

  psm_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  rclcpp::sleep_for(500ms);
}
}  // namespace galactic_demo

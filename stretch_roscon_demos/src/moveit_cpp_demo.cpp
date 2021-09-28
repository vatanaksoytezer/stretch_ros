#include <rclcpp/rclcpp.hpp>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <geometry_msgs/msg/point_stamped.h>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("moveit_cpp_demo", "", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  static const std::string PLANNING_GROUP = "stretch_arm";
  static const std::string LOGNAME = "moveit_cpp_demo";

  /* Otherwise robot with zeros joint_states */
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(LOGGER, "Starting MoveItCpp Pick and Place Demo ...");

  auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
  moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

  auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto robot_start_state = planning_components->getStartState();
  auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

  // Start the demo

  // Planning with MoveItCpp
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // There are multiple ways to set the start and the goal states of the plan
  // they are illustrated in the following plan examples
  //
  // Plan #1
  // ^^^^^^^
  //
  // We can set the start state of the plan to the current state of the robot
  planning_components->setStartStateToCurrentState();

  // The first way to set the goal of the plan is by using geometry_msgs::PoseStamped ROS message type as follow
  geometry_msgs::msg::PoseStamped target_pose1;
  target_pose1.header.frame_id = "odom";
  target_pose1.pose.position.x = -0.058876442274373234;
  target_pose1.pose.position.y = -0.6686726215135715;
  target_pose1.pose.position.z = 0.9947614781407926;
  target_pose1.pose.orientation.x = 2.3418130500298247e-06;
  target_pose1.pose.orientation.y = -1.9460595924624177e-07;
  target_pose1.pose.orientation.z = 0.007628164292894081;
  target_pose1.pose.orientation.w = 0.9999709051287435;

  planning_components->setGoal(target_pose1, "stretch_arm");

  // Now, we call the PlanningComponents to compute the plan and visualize it.
  // Note that we are just planning
  auto plan_solution1 = planning_components->plan();

  if (plan_solution1)
  {
    planning_components->execute(); // Execute the plan
  }

  // Plan #3
  // ^^^^^^^
  //
  // We can also set the goal of the plan using
  // moveit::core::RobotState
  /*
  auto target_state = *robot_start_state;
  geometry_msgs::msg::Pose target_pose2;
  target_pose2.orientation.w = 1.0;
  target_pose2.position.x = 0.55;
  target_pose2.position.y = -0.05;
  target_pose2.position.z = 0.8;

  target_state.setFromIK(joint_model_group_ptr, target_pose2);

  planning_components->setGoal(target_state);

  // We will reuse the old start that we had and plan from it.
  auto plan_solution3 = planning_components->plan();
  if (plan_solution3)
  {
    moveit::core::RobotState robot_state(robot_model_ptr);
    moveit::core::robotStateMsgToRobotState(plan_solution3.start_state, robot_state);
    planning_components->execute(); // Execute the plan
  }
  */

  // Plan #4
  // ^^^^^^^
  //
  // We can set the start state of the plan to the current state of the robot
  // We can set the goal of the plan using the name of a group states
  // for panda robot we have one named robot state for "panda_arm" planning group called "ready"
  // see `panda_arm.xacro
  // <https://github.com/ros-planning/panda_moveit_config/blob/melodic-devel/config/panda_arm.xacro#L13>`_

  /*
  planning_components->setStartStateToCurrentState();
  planning_components->setGoal("ready");
  // Again we will reuse the old start that we had and plan from it.
  auto plan_solution4 = planning_components->plan();
  if (plan_solution4)
  {
    moveit::core::RobotState robot_state(robot_model_ptr);
    moveit::core::robotStateMsgToRobotState(plan_solution4.start_state, robot_state);
    planning_components->execute(); // Execute the plan
  }
  */

  // Plan #5
  // ^^^^^^^
  //
  // We can also generate motion plans around objects in the collision scene.
  //
  // First we create the collision object
  /*
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "panda_link0";
  collision_object.id = "box";

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = { 0.1, 0.4, 0.1 };

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.4;
  box_pose.position.y = 0.0;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Add object to planning scene
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(collision_object);
  }  // Unlock PlanningScene
  planning_components->setStartStateToCurrentState();
  planning_components->setGoal("extended");

  auto plan_solution5 = planning_components->plan();
  if (plan_solution5)
  {
    planning_components->execute(); // Execute the plan
  }
  */

  RCLCPP_INFO(LOGGER, "Shutting down.");
  rclcpp::shutdown();
  return 0;
}

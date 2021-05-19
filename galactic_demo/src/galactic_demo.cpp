/*******************************************************
 * Copyright (C) 2021 Picknik Robotics
 *
 * This file can not be copied and/or distributed without the express
 * permission of Picknik Robotics.
 *******************************************************/

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace galactic_demo
{
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

class TrajectoryActionClient : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  explicit TrajectoryActionClient(const rclcpp::NodeOptions & options)
  : Node("trajectory_action_client", options)
  {
    client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(this,
                                                                      "/stretch_controller/follow_joint_trajectory");
    timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&TrajectoryActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names.push_back("wrist_extension");
    goal.trajectory.points.push_back(makePoint(0.0, base_pos_));
    goal.trajectory.points.push_back(makePoint(swing_duration_ * 0.5, base_pos_ + arm_swing_));
    goal.trajectory.points.push_back(makePoint(swing_duration_ * 1.5, base_pos_ - arm_swing_));

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&TrajectoryActionClient::goal_response_callback, this, _1);
    send_goal_options.result_callback = std::bind(&TrajectoryActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal, send_goal_options);
  }

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  double arm_swing_ {0.1};
  double swing_duration_ {10.0};
  double base_pos_ {0.1};

  void goal_response_callback(std::shared_future<GoalHandle::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void result_callback(const GoalHandle::WrappedResult & result)
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
    RCLCPP_INFO(this->get_logger(), result.result->error_string.c_str());
    rclcpp::shutdown();
  }
};

}  // namespace galactic_demo


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<galactic_demo::TrajectoryActionClient>(options));
  rclcpp::shutdown();

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
//RCLCPP_COMPONENTS_REGISTER_NODE(galactic_demo::TrajectoryActionClient)

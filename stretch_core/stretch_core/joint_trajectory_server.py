#! /usr/bin/env python
from __future__ import print_function

import traceback

from control_msgs.action import FollowJointTrajectory

from geometry_msgs.msg import Transform

from hello_helpers.gripper_conversion import GripperConversion
from hello_helpers.hello_misc import to_sec

import pyquaternion

import rclpy
from rclpy.action import ActionServer

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .action_exceptions import FollowJointTrajectoryException, GoalToleranceException
from .action_exceptions import InvalidGoalException, InvalidJointException
from .command_groups import GripperCommandGroup, HeadPanCommandGroup, HeadTiltCommandGroup
from .command_groups import LiftCommandGroup, MobileBaseCommandGroup, TelescopingCommandGroup, WristYawCommandGroup


def transform_to_triple(transform):
    x = transform.translation.x
    y = transform.translation.y

    quat = transform.rotation
    q = pyquaternion.Quaternion(x=quat.x, y=quat.y, z=quat.z, w=quat.w)
    yaw, pitch, roll = q.yaw_pitch_roll
    return x, y, yaw


def to_transform(d):
    t = Transform()
    t.translation.x = d['x']
    t.translation.y = d['y']
    quaternion = pyquaternion.Quaternion(axis=[0, 0, 1], angle=d['theta'])
    t.rotation.w = quaternion.w
    t.rotation.x = quaternion.x
    t.rotation.y = quaternion.y
    t.rotation.z = quaternion.z
    return t


def twist_to_pair(msg):
    return msg.linear.x, msg.angular.z


def merge_arm_joints(trajectory):
    new_trajectory = JointTrajectory()
    arm_indexes = []
    for index, name in enumerate(trajectory.joint_names):
        if 'joint_arm_l' in name:
            arm_indexes.append(index)
        else:
            new_trajectory.joint_names.append(name)

    # If individual arm joints are not present, the original trajectory is fine
    if not arm_indexes:
        return trajectory

    if 'wrist_extension' in trajectory.joint_names:
        raise InvalidJointException('Received a command for the wrist_extension joint and one or more '
                                    'telescoping_joints. These are mutually exclusive options. '
                                    f'The joint names in the received command = {trajectory.joint_names}')

    if len(arm_indexes) != 4:
        raise InvalidJointException('Commands with telescoping joints requires all telescoping joints to be present. '
                                    f'Only received len(arm_indexes) of 4 telescoping joints.')

    # Set up points and variables to track arm values
    total_extension = []
    arm_velocities = []
    arm_accelerations = []
    for point in trajectory.points:
        new_point = JointTrajectoryPoint()
        new_point.time_from_start = point.time_from_start
        new_trajectory.points.append(new_point)

        total_extension.append(0.0)
        arm_velocities.append([])
        arm_accelerations.append([])

    for index, name in enumerate(trajectory.joint_names):
        for point_index, point in enumerate(trajectory.points):
            x = point.positions[index]
            v = point.velocities[index] if index < len(point.velocities) else None
            a = point.accelerations[index] if index < len(point.accelerations) else None

            if index in arm_indexes:
                total_extension[point_index] += x
                if v is not None:
                    arm_velocities[point_index].append(v)
                if a is not None:
                    arm_accelerations[point_index].append(a)
            else:
                new_point = new_trajectory.points[point_index]
                new_point.positions.append(x)
                if v is not None:
                    new_point.velocities.append(v)
                if a is not None:
                    new_point.accelerations.append(a)

    # Now add the arm values
    new_trajectory.joint_names.append('wrist_extension')
    for point_index, new_point in enumerate(new_trajectory.points):
        new_point.positions.append(total_extension[point_index])
        vels = arm_velocities[point_index]
        accels = arm_accelerations[point_index]

        if vels:
            new_point.velocities.append(sum(vels) / len(vels))
        if accels:
            new_point.accelerations.append(sum(accels) / len(accels))

    return new_trajectory


def preprocess_gripper_trajectory(trajectory):
    gripper_joint_names = ['joint_gripper_finger_left', 'joint_gripper_finger_right', 'gripper_aperture']
    present_gripper_joints = list(set(gripper_joint_names) & set(trajectory.joint_names))

    # If no gripper joint names are present, no changes needed
    if not present_gripper_joints:
        return trajectory
    elif len(present_gripper_joints) == 2:
        if (gripper_joint_names[0] in present_gripper_joints and gripper_joint_names[1] in present_gripper_joints):
            # Make sure that all the points are the same
            left_index = trajectory.joint_names.index(gripper_joint_names[0])
            right_index = trajectory.joint_names.index(gripper_joint_names[1])
            for pt in trajectory.points:
                if pt.position[left_index] != pt.position[right_index]:
                    raise InvalidGoalException('Recieved a command that includes both the left and right gripper '
                                               'joints and their commanded positions are not the same. '
                                               f'{pt.position[left_index]} != {pt.position[right_index]}')
                # Due dilligence would also check the velocity/acceleration, but leaving for now

            # If all the points are the same, then we can safely eliminate one
            trajectory.joint_names = trajectory.joint_names[:right_index] + trajectory.joint_names[right_index + 1:]
            for pt in trajectory.points:
                pt.position = pt.position[:right_index] + pt.position[right_index + 1:]
                if pt.velocity:
                    pt.velocity = pt.velocity[:right_index] + pt.velocity[right_index + 1:]
                if pt.acceleration:
                    pt.acceleration = pt.acceleration[:right_index] + pt.acceleration[right_index + 1:]
            present_gripper_joints = gripper_joint_names[:1]
        else:
            raise InvalidJointException('Recieved a command that includes an odd combination of gripper joints: '
                                        f'{present_gripper_joints}')
    elif len(present_gripper_joints) != 1:
        raise InvalidJointException('Recieved a command that includes too many gripper joints: '
                                    f'{present_gripper_joints}')

    # Now convert the gripper joint to the proper units
    gripper_conversion = GripperConversion()
    # TODO:


class JointTrajectoryAction:

    def __init__(self, node, fail_out_of_range_goal):
        self.node = node
        self.fail_out_of_range_goal = fail_out_of_range_goal
        self.server = ActionServer(self.node, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory',
                                   self.execute_cb)

        r = self.node.robot
        wrist_yaw_range_ticks = r.end_of_arm.motors['wrist_yaw'].params['range_t']
        wrist_yaw_range_rad = (r.end_of_arm.motors['wrist_yaw'].ticks_to_world_rad(wrist_yaw_range_ticks[1]),
                               r.end_of_arm.motors['wrist_yaw'].ticks_to_world_rad(wrist_yaw_range_ticks[0]))
        gripper_range_ticks = r.end_of_arm.motors['stretch_gripper'].params['range_t']
        gripper_range_rad = (r.end_of_arm.motors['stretch_gripper'].ticks_to_world_rad(gripper_range_ticks[0]),
                             r.end_of_arm.motors['stretch_gripper'].ticks_to_world_rad(gripper_range_ticks[1]))
        gripper_range_robotis = (r.end_of_arm.motors['stretch_gripper'].world_rad_to_pct(gripper_range_rad[0]),
                                 r.end_of_arm.motors['stretch_gripper'].world_rad_to_pct(gripper_range_rad[1]))

        self.head_pan_cg = HeadPanCommandGroup(r,
                                               self.node.head_pan_calibrated_offset_rad,
                                               self.node.head_pan_calibrated_looked_left_offset_rad)
        self.head_tilt_cg = HeadTiltCommandGroup(r,
                                                 self.node.head_tilt_calibrated_offset_rad,
                                                 self.node.head_tilt_calibrated_looking_up_offset_rad,
                                                 self.node.head_tilt_backlash_transition_angle_rad)
        self.wrist_yaw_cg = WristYawCommandGroup(wrist_yaw_range_rad)
        self.gripper_cg = GripperCommandGroup(gripper_range_robotis)
        self.telescoping_cg = TelescopingCommandGroup(tuple(r.arm.params['range_m']),
                                                      self.node.wrist_extension_calibrated_retracted_offset_m)
        self.lift_cg = LiftCommandGroup(tuple(r.lift.params['range_m']))
        self.mobile_base_cg = MobileBaseCommandGroup(virtual_range_m=(-0.5, 0.5))

        self.command_groups = [self.telescoping_cg, self.lift_cg, self.mobile_base_cg, self.head_pan_cg,
                               self.head_tilt_cg, self.wrist_yaw_cg, self.gripper_cg]
        self.cg_map = {group.name: group for group in self.command_groups}

    def execute_cb(self, goal_handle):
        try:
            with self.node.robot_stop_lock:
                # Escape stopped mode to execute trajectory
                self.node.stop_the_robot = False
                self.node.robot_mode_rwlock.acquire_read()

            # Process the goal
            goal = goal_handle.request

            # Check for valid positions
            for i, pt in enumerate(goal.trajectory.points):
                if len(pt.positions) != len(goal.joint_names):
                    raise InvalidGoalException(f'Goal point with index {i} has {len(pt.positions)} positions '
                                               f'but should have {len(goal.joint_names)}')

            goal.trajectory = merge_arm_joints(goal.trajectory)
            goal.trajectory = preprocess_gripper_trajectory(goal.trajectory)

            # Check for invalid names
            for index, name in enumerate(goal.trajectory.joint_names):
                if name not in self.cg_map:
                    raise InvalidJointException(f'Cannot find joint "{name}"')
            multi_dof_joints = goal.multi_dof_trajectory.joint_names
            if len(multi_dof_joints) != 1 or multi_dof_joints[0] != 'position':
                raise InvalidJointException('Driver supports a single multi_dof joint named position. '
                                            f'Got {multi_dof_joints} instead.')
            for i, pt in enumerate(goal.multi_dof_trajectory.points):
                if len(pt.positions) != len(goal.multi_dof_joint_names):
                    raise InvalidGoalException(f'MultiDOF goal point with index {i} has {len(pt.positions)} positions '
                                               f'but should have {len(goal.joint_names)}')

            # TODO: Add parameter
            for pt in goal.trajectory.points:
                pt.accelerations = []

            # For now, ignore goal time and configuration tolerances.

            # Print the goal
            if goal.trajectory.points:
                dt = to_sec(goal.trajectory.points[-1].time_from_start)
                n_points = len(goal.trajectory.points)
            else:
                dt = to_sec(goal.multi_dof_trajectory.points[-1].time_from_start)
                n_points = len(goal.multi_dof_trajectory.points)

            n_joints = len(goal.trajectory.joint_names) + len(goal.multi_dof_trajectory.joint_names)
            self.log(f'New follow_joint_trajectory goal with {n_points} points, {n_joints} joints over {dt} seconds.')

            if self.node.robot_mode == 'manipulation':
                return self.execute_trajectory(goal_handle, goal)
            else:
                return self.execute_sequence(goal_handle, goal)
        except FollowJointTrajectoryException as e:
            self.error(e)
            goal_handle.abort()
            return FollowJointTrajectory.Result(error_code=e.CODE, error_string=str(e))
        except Exception as e:
            self.node.robot.stop_trajectory()
            self.error(traceback.format_exc())
            goal_handle.abort()
            return FollowJointTrajectory.Result(error_code=-10000, error_string=str(e))
        finally:
            self.node.robot_mode_rwlock.release_read()

    def execute_sequence(self, goal_handle, goal):
        ###################################################
        # Decide what to do based on the commanded joints.
        commanded_joint_names = goal.trajectory.joint_names
        command_groups = [self.telescoping_cg, self.lift_cg, self.mobile_base_cg, self.head_pan_cg,
                          self.head_tilt_cg, self.wrist_yaw_cg, self.gripper_cg]

        ###################################################
        # Try to reach each of the goals in sequence until
        # an error is detected or success is achieved.
        for pointi, point in enumerate(goal.trajectory.points):
            self.node.get_logger().debug(("joint_traj action: "
                                          "target point #{0} = <{1}>").format(pointi, point))

            robot_status = self.node.robot.get_status()  # uses lock held by robot
            for index, name in enumerate(goal.trajectory.joint_names):
                self.cg_map[name].set_goal(point, index, self.fail_out_of_range_goal,
                                           manipulation_origin=self.node.mobile_base_manipulation_origin)

            for name in goal.trajectory.joint_names:
                self.cg_map[name].init_execution(self.node.robot, robot_status, backlash_state=self.node.backlash_state)

            self.node.robot.push_command()

            goals_reached = [c.goal_reached() for c in command_groups]
            goal_start_time = self.node.get_clock().now()

            while not all(goals_reached):
                if (self.node.get_clock().now() - goal_start_time) > self.node.default_goal_timeout_duration:
                    raise GoalToleranceException(f'Time to execute the current goal point = <{point}> exceeded '
                                                 f'the default_goal_timeout = {self.node.default_goal_timeout_s}')

                # Check if a premption request has been received.
                with self.node.robot_stop_lock:
                    if self.node.stop_the_robot or self.goal_handle.is_cancel_requested:
                        self.server.set_preempted()
                        self.node.get_logger().debug("joint_traj action: PREEMPTION REQUESTED, "
                                                     "but not stopping current motions to allow smooth "
                                                     "interpolation between old and new commands."
                                                     )
                        self.node.stop_the_robot = False
                        self.node.robot_mode_rwlock.release_read()
                        return self.result

                robot_status = self.node.robot.get_status()
                named_errors = [c.update_execution(robot_status, success_callback=self.success_callback,
                                                   backlash_state=self.node.backlash_state)
                                for c in command_groups]
                if any(ret for ret in named_errors):
                    self.node.robot_mode_rwlock.release_read()
                    return self.result

                self.feedback_callback(commanded_joint_names, point, named_errors)
                goals_reached = [c.goal_reached() for c in command_groups]
                rclpy.spin_once(self.node)

            self.node.get_logger().debug("joint_traj action: Achieved target point.")

        goal_handle.succeed()
        return self.create_result(FollowJointTrajectory.Result.SUCCESSFUL, 'Achieved all target points.')

    def feedback_callback(self, commanded_joint_names, desired_point, named_errors):
        clean_named_errors = []
        for named_error in named_errors:
            if type(named_error) == tuple:
                clean_named_errors.append(named_error)
            elif type(named_error) == list:
                clean_named_errors += named_error
        clean_named_errors_dict = dict((k, v) for k, v in clean_named_errors)

        actual_point = JointTrajectoryPoint()
        error_point = JointTrajectoryPoint()
        for i, commanded_joint_name in enumerate(commanded_joint_names):
            error_point.positions.append(clean_named_errors_dict[commanded_joint_name])
            actual_point.positions.append(desired_point.positions[i] - clean_named_errors_dict[commanded_joint_name])

        self.node.get_logger().info("joint_traj action: sending feedback")
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.node.get_clock().now()
        feedback.joint_names = commanded_joint_names
        feedback.desired = desired_point
        feedback.actual = actual_point
        feedback.error = error_point
        self.goal_handle.publish_feedback(feedback)

    def success_callback(self, success_str):
        self.node.get_logger().info("joint_traj action: {0}".format(success_str))
        self.result.error_code = self.result.SUCCESSFUL
        self.result.error_string = success_str
        self.goal_handle.succeed()

    def log(self, o):
        self.node.get_logger().info(str(o))

    def warn(self, o):
        self.node.get_logger().warn(str(o))

    def error(self, o):
        self.node.get_logger().error(str(o))

    def create_result(self, error_code, msg):
        if error_code < 0:
            self.error(msg)
        else:
            self.log(msg)

        return FollowJointTrajectory.Result(error_code=error_code, error_string=str(msg))

    def execute_trajectory(self, goal_handle, goal):
        cgs = []
        self.node.robot.pull_status()
        robot_status = self.node.robot.status

        for index, name in enumerate(goal.trajectory.joint_names):

            cg = self.cg_map[name]
            comp = cg.get_component(self.node.robot)
            cgs.append(cg)

            # Set Initial waypoint
            state = cg.get_state(robot_status)
            goal.trajectory.points[0].positions[index] = state['pos']
            if index < len(goal.trajectory.points[0].velocities):
                goal.trajectory.points[0].velocities[index] = state['vel']

            comp.trajectory.clear_waypoints()
            for waypoint in goal.trajectory.points:
                t = to_sec(waypoint.time_from_start)
                x = waypoint.positions[index]
                v = waypoint.velocities[index] if index < len(waypoint.velocities) else None
                a = waypoint.accelerations[index] if index < len(waypoint.accelerations) else None
                self.log(f'{cg.name} {t} {x} {v} {a}')
                comp.trajectory.add_waypoint(t, x, v, a)

        if goal.multi_dof_trajectory.joint_names:
            index = 0
            cg = self.mobile_base_cg
            comp = cg.get_component(self.node.robot)
            cgs.append(cg)

            comp.trajectory.clear_waypoints()
            for waypoint in goal.multi_dof_trajectory.points:
                t = to_sec(waypoint.time_from_start)
                x = transform_to_triple(waypoint.transforms[index])
                v = twist_to_pair(waypoint.velocities[index]) if index < len(waypoint.velocities) else None
                a = twist_to_pair(waypoint.accelerations[index]) if index < len(waypoint.accelerations) else None
                self.log(f'{cg.name} {t} {x} {v} {a}')
                comp.trajectory.add_waypoint(t, x, v, a)
            comp.trajectory.complete_trajectory()

        start_time = self.node.get_clock().now()
        self.node.robot.start_trajectory()

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = goal.trajectory.joint_names
        feedback.multi_dof_joint_names = goal.multi_dof_trajectory.joint_names
        rate = self.node.create_rate(10)
        while rclpy.ok() and self.node.robot.is_trajectory_executing():
            robot_status = self.node.robot.status
            now = self.node.get_clock().now()
            feedback.header.stamp = now.to_msg()
            feedback.desired.time_from_start = (now - start_time).to_msg()
            feedback.actual.time_from_start = feedback.desired.time_from_start
            feedback.desired.positions = []
            feedback.actual.positions = []

            dt = to_sec(feedback.desired.time_from_start)
            for i, joint_name in enumerate(feedback.joint_names):
                cg = self.cg_map[joint_name]
                state = cg.get_state(robot_status)
                feedback.actual.positions.append(state['pos'])
                comp = cg.get_component(self.node.robot)
                des = comp.trajectory.evaluate_at(dt)
                feedback.desired.positions.append(des.position)

            if feedback.multi_dof_joint_names:
                cg = self.mobile_base_cg
                comp = cg.get_component(self.node.robot)
                feedback.multi_dof_actual.time_from_start = feedback.desired.time_from_start
                feedback.multi_dof_desired.time_from_start = feedback.desired.time_from_start
                state = cg.get_state(robot_status)
                feedback.multi_dof_actual.transforms = [to_transform(state)]

            goal_handle.publish_feedback(feedback)
            rate.sleep()

        self.node.robot.stop_trajectory()
        goal_handle.succeed()
        return self.create_result(FollowJointTrajectory.Result.SUCCESSFUL, 'Achieved all target points.')

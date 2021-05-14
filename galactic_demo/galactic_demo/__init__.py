"""Galactic Demo Python Code."""
from control_msgs.action import FollowJointTrajectory

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_srvs.srv import Trigger

from trajectory_msgs.msg import JointTrajectoryPoint


def goal_point(position, velocity=None, acceleration=None, seconds=None):
    p = JointTrajectoryPoint()
    p.positions = [position]
    if velocity is not None:
        p.velocities = [velocity]
    if acceleration is not None:
        p.accelerations = [acceleration]
    if seconds is not None:
        p.time_from_start = Duration(seconds=seconds).to_msg()
    return p


class TrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('trajectory_action_client')
        self.base_pos = None
        self.stopped = False
        self.released = False
        self.arm_swing = 0.1
        self.swing_duration = 5.0
        self.stop = self.create_service(Trigger, 'stop_rockin', self.stop_rockin)
        self._action_client = ActionClient(self, FollowJointTrajectory,
                                           '/stretch_controller/follow_joint_trajectory')
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 1)

    def joint_state_cb(self, msg):
        if self.base_pos is not None:
            return
        total = 0.0
        for name, position in zip(msg.name, msg.position):
            if name.startswith('joint_arm_l'):
                total += position
        self.base_pos = total
        self.log(f'Arm position: {self.base_pos}')

        # Send Initial
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['wrist_extension']
        goal.trajectory.points.append(goal_point(self.base_pos,                  0.0, 0.0, 0.0))  # noqa: E241
        goal.trajectory.points.append(goal_point(self.base_pos + self.arm_swing, 0.0, 0.0, self.swing_duration / 2))
        goal.trajectory.points.append(goal_point(self.base_pos - self.arm_swing, 0.0, 0.0, self.swing_duration * 1.5))
        self.send(goal)

        self.sub.destroy()

    def send(self, goal):
        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.log('Goal rejected :(')
            return

        self.log('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def stop_rockin(self, request, response):
        self.log('Stop')
        self.stopped = True
        response.success = True
        response.message = 'Stopped.'
        return response

    def get_result_callback(self, future):
        if self.released:
            self.log('Fully complete')
            rclpy.shutdown()
        goal = FollowJointTrajectory.Goal()
        if self.stopped:
            self.log('Stopped')
            self.released = True
            goal.trajectory.joint_names = ['wrist_extension']
            goal.trajectory.points.append(goal_point(self.base_pos - self.arm_swing, 0.0, 0.0, 0.0))
            goal.trajectory.points.append(goal_point(self.base_pos, 0.0, 0.0, self.swing_duration / 2))
        else:

            goal.trajectory.joint_names = ['wrist_extension']
            goal.trajectory.points.append(goal_point(self.base_pos - self.arm_swing, 0.0, 0.0, 0.0))
            goal.trajectory.points.append(goal_point(self.base_pos + self.arm_swing, 0.0, 0.0, self.swing_duration))
            goal.trajectory.points.append(goal_point(self.base_pos - self.arm_swing, 0.0, 0.0,
                                                     self.swing_duration * 2.0))
        self.send(goal)

    def log(self, o):
        self.get_logger().info(str(o))

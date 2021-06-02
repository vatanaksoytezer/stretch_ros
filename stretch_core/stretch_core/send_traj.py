from math import cos, radians, sin

from control_msgs.action import FollowJointTrajectory

from geometry_msgs.msg import Transform, Twist

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint


def to_transform(x=0.0, y=0.0, theta=0.0):
    tf = Transform()
    tf.translation.x = x
    tf.translation.y = y
    tf.rotation.z = sin(theta * 0.5)
    tf.rotation.w = cos(theta * 0.5)
    return tf


def to_twist(x=0.0, y=0.0, theta=0.0):
    cmd = Twist()
    cmd.linear.x = x
    cmd.linear.y = y
    cmd.angular.z = theta
    return cmd


class TrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('trajectory_action_client')
        self._action_client = ActionClient(self, FollowJointTrajectory,
                                           '/stretch_controller/follow_joint_trajectory')

        self.send_position = False

    def send_goal(self):
        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw', 'joint_head_tilt',
                                       'joint_head_pan', 'joint_gripper_finger_left']
        p0 = JointTrajectoryPoint()
        p0.positions = [0.92, 0.5, radians(36), radians(90), radians(0), 0.0]
        p0.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]
        goal.trajectory.points.append(p0)

        p1 = JointTrajectoryPoint()
        p1.positions = [0.2, 0.0, radians(90), radians(0), radians(0), 0.24]
        p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]
        p1.time_from_start = Duration(seconds=10).to_msg()
        goal.trajectory.points.append(p1)

        p2 = JointTrajectoryPoint()
        p2.positions = [0.92, 0.5, radians(36), radians(90), radians(0), 0.0]
        p2.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]
        p2.time_from_start = Duration(seconds=20).to_msg()
        goal.trajectory.points.append(p2)

        if self.send_position:
            goal.multi_dof_trajectory.joint_names = ['position']
            p0 = MultiDOFJointTrajectoryPoint()
            p0.transforms = [to_transform()]
            p0.velocities = [to_twist()]
            goal.multi_dof_trajectory.points.append(p0)

            p1 = MultiDOFJointTrajectoryPoint()
            p1.transforms = [to_transform(theta=radians(90))]
            p1.time_from_start = Duration(seconds=10).to_msg()
            p1.velocities = [to_twist()]
            goal.multi_dof_trajectory.points.append(p1)

            p2 = MultiDOFJointTrajectoryPoint()
            p2.transforms = [to_transform()]
            p2.time_from_start = Duration(seconds=20).to_msg()
            p2.velocities = [to_twist()]
            goal.multi_dof_trajectory.points.append(p2)

        # goal.goal_tolerance.append(JointTolerance())
        # goal.path_tolerance.append(JointTolerance())

        self.log('Waiting for client')
        self._action_client.wait_for_server()
        self.log('Sending goal')
        self._send_goal_future = self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.log('Goal rejected :(')
            return

        self.log('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.log(feedback_msg)

    def get_result_callback(self, future):
        result = future.result().result
        self.log('Result: {0}'.format(result))
        rclpy.shutdown()

    def log(self, s):
        self.get_logger().info(str(s))


def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()

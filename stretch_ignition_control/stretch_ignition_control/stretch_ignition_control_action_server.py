from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class FollowJointTrajectoryActionServer(Node):

    def __init__(self):
        super().__init__('follow_joint_trajectory_action_server')
        self._joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/stretch_controller/follow_joint_trajectory',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        self._joint_trajectory_publisher.publish(trajectory)
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)

    follow_joint_trajectory_action_server = FollowJointTrajectoryActionServer()
    rclpy.spin(follow_joint_trajectory_action_server)

if __name__ == '__main__':
    main()

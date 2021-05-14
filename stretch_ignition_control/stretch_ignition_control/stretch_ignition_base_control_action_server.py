from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time

class StretchBaseActionServer(Node):

    def __init__(self):
        super().__init__('multidof_joint_trajectory_action_server')
        self._twist_publisher = self.create_publisher(Twist, '/joint_trajectory', 10)
        self._action_server = ActionServer(
            self,
            MultiDOFJointTrajectory,
            '/stretch_controller/follow_joint_trajectory',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = MultiDOFJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        self._twist_publisher.publish(trajectory)
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)

    stretch_base_action_server = StretchBaseActionServer()
    rclpy.spin(stretch_base_action_server)

if __name__ == '__main__':
    main()

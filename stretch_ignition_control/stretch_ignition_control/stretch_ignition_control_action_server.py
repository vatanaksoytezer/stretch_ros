from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, server
from rclpy.node import Node
import time
from math import sqrt


class FollowJointTrajectoryActionServer(Node):

    def __init__(self):
        super().__init__('follow_joint_trajectory_action_server')
        print("Starting action server")
        self.odom = Odometry()
        self._joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        self._cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/stretch_controller/follow_joint_trajectory',
            self.execute_callback)
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def execute_callback(self, goal_handle: server.ServerGoalHandle):
        print(type(goal_handle))
        self.get_logger().info('Executing goal...')
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory # type: JointTrajectory
        multidof_trajectory = goal_handle.request.multi_dof_trajectory # type: MultiDOFJointTrajectory
        # print(multidof_trajectory)
        previous_time_from_start = 0.0
        self._joint_trajectory_publisher.publish(trajectory)
        print("Target pose:", multidof_trajectory.points[-1].transforms[0].translation, multidof_trajectory.points[-1].transforms[0].rotation)
        for point in multidof_trajectory.points:
            cmd_vel = Twist()
            current_time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            cmd_vel.linear.x = float(sqrt(point.velocities[0].linear.x * point.velocities[0].linear.x + point.velocities[0].linear.y * point.velocities[0].linear.y))
            cmd_vel.angular.z = float(point.velocities[0].angular.z)
            if(point.velocities[0].linear.x < 0):
                cmd_vel.linear.x = -cmd_vel.linear.x
            # cmd_vel = point.velocities[0]
            print(point)
            # print(cmd_vel)
            self._cmd_vel_publisher.publish(cmd_vel)
            execution_time = current_time_from_start - previous_time_from_start
            time.sleep(execution_time)
            previous_time_from_start = current_time_from_start
        
        cmd_vel = Twist()
        self._cmd_vel_publisher.publish(cmd_vel)
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    follow_joint_trajectory_action_server = FollowJointTrajectoryActionServer()
    rclpy.spin(follow_joint_trajectory_action_server)

if __name__ == '__main__':
    main()

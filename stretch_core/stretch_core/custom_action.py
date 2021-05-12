from control_msgs.action import FollowJointTrajectory

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

import stretch_body.robot

import yaml

class SimpleTraj(Node):
    def __init__(self):
        super().__init__('simple_traj')
        self.r = stretch_body.robot.Robot()
        self.a = self.r.arm
        self.r.startup()

        self._action_server = ActionServer(self, FollowJointTrajectory,
                                           '/stretch_controller/follow_joint_trajectory', self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.r.pull_status()

        vtime = [0.0, 10.0, 20.0]
        vpos = [self.a.status['pos'], 0.5, 0.0]
        vvel = [self.a.status['vel'], 0.0, 0.0]

        def start_trajectory(times, positions, velocities):
            self.a.trajectory.clear_waypoints()
            for waypoint in zip(times, positions, velocities):
                self.get_logger().info(str(waypoint))
                self.a.trajectory.add_waypoint(t_s=waypoint[0], x_m=waypoint[1], v_m=waypoint[2])
            self.r.start_trajectory()

        def sense_trajectory():
            self.a.pull_status()
            if self.a.status['motor']['trajectory_active']:
                self.a.push_trajectory()
                if self.a.traj_start_time is not None and self.a.traj_curr_time is not None:
                    return (self.a.traj_curr_time - self.a.traj_start_time, self.a.status['pos'])

        start_trajectory(vtime, vpos, vvel)
        c = 0
        while rclpy.ok():
            sense = sense_trajectory()
            if not sense:
                self.r.stop_trajectory()
                result = FollowJointTrajectory.Result()
                if c == 0:
                    self.get_logger().info(yaml.dump(self.a.status))
                    goal_handle.abort()
                    self.get_logger().info('...Goal failed')
                    result.error_string = 'Failure'
                else:
                    goal_handle.succeed()
                    self.get_logger().info('...Goal complete')
                    result.error_string = 'Success'
                return result

            self.get_logger().info(str(sense))
            c += 1


def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = SimpleTraj()
    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()

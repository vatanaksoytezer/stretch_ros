import collections

from control_msgs.action import FollowJointTrajectory

from hello_helpers.hello_misc import to_sec

from matplotlib.pyplot import ion, subplots

from numpy import array_equal

import rclpy
from rclpy.node import Node


class PlotTrajClient(Node):
    def __init__(self):
        super().__init__('trajectory_action_client')
        self.goal_id = None
        self.joint_names = []
        self.f = None
        self.lines = {}
        ion()
        self.sub = self.create_subscription(FollowJointTrajectory.Impl.FeedbackMessage(),
                                            '/stretch_controller/follow_joint_trajectory/_action/feedback',
                                            self.feedback_callback,
                                            1)
        self.log('Ready.')

    def feedback_callback(self, feedback_msg):
        goal_id = feedback_msg.goal_id.uuid
        if self.goal_id is None or not array_equal(self.goal_id, goal_id):
            self.joint_names = feedback_msg.feedback.joint_names
            if self.f is None:
                self.f, self.axes = subplots(len(self.joint_names))
                for i, name in enumerate(self.joint_names):
                    self.axes[i].title.set_text(name)
            self.data = collections.defaultdict(list)
            self.goal_id = goal_id
        desired = feedback_msg.feedback.desired
        actual = feedback_msg.feedback.actual

        t = to_sec(desired.time_from_start)
        self.data['t'].append(t)
        for i, name in enumerate(self.joint_names):
            self.data[f'{name}_desired'].append(desired.positions[i])
            self.data[f'{name}_actual'].append(actual.positions[i])

            for t in ['desired', 'actual']:
                k = f'{name}_{t}'
                if k not in self.lines:
                    self.lines[k] = self.axes[i].plot(self.data['t'], self.data[k], label=t)[0]
                    self.log(self.lines[k])
                    self.axes[i].legend()
                else:
                    self.lines[k].set_xdata(self.data['t'])
                    self.lines[k].set_ydata(self.data[k])
            self.axes[i].relim()
            self.axes[i].autoscale_view(True, True, True)
        self.f.canvas.draw()

    def log(self, s):
        self.get_logger().info(str(s))


def main(args=None):
    rclpy.init(args=args)
    client = PlotTrajClient()
    rclpy.spin(client)


if __name__ == '__main__':
    main()

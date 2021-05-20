from hello_helpers.hello_misc import to_sec

from matplotlib.pyplot import show, subplots

from moveit_msgs.msg import DisplayTrajectory

import rclpy
from rclpy.node import Node


class PlotPlannedClient(Node):
    def __init__(self):
        super().__init__('plot_planned')
        self.f = None
        self.lines = {}
        self.sub = self.create_subscription(DisplayTrajectory,
                                            '/display_planned_path',
                                            self.trajectory_cb,
                                            1)
        self.log('Ready.')

    def trajectory_cb(self, traj_msg):
        traj = traj_msg.trajectory[0].joint_trajectory

        self.f, self.axes = subplots(len(traj.joint_names))

        for i, name in enumerate(traj.joint_names):
            self.axes[i].title.set_text(name)

            x = []
            y = []
            for pt in traj.points:
                t = to_sec(pt.time_from_start)
                x.append(t)
                y.append(pt.positions[i])

            if name not in self.lines:
                self.lines[name] = self.axes[i].plot(x, y, 'o-')[0]
            else:
                self.lines[name].set_xdata(x)
                self.lines[name].set_ydata(y)
            self.axes[i].relim()
            self.axes[i].autoscale_view(True, True, True)
        show()

    def log(self, s):
        self.get_logger().info(str(s))


def main(args=None):
    rclpy.init(args=args)
    client = PlotPlannedClient()
    rclpy.spin(client)


if __name__ == '__main__':
    main()

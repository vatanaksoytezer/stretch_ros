import stretch_body.robot


def main(args=None):
    robot = stretch_body.robot.Robot()
    robot.startup()
    robot.head.get_joint('head_pan').stop_trajectory()
    robot.head.get_joint('head_tilt').stop_trajectory()
    robot.lift.stop_trajectory()
    robot.base.stop_trajectory()
    robot.arm.stop_trajectory()


if __name__ == '__main__':
    main()

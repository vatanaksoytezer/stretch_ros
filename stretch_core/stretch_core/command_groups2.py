import hello_helpers.hello_misc as hm

from .action_exceptions import InvalidGoalException


def get_goal(point, index, joint_range, fail_out_of_range_goal=True):
    """Create and validates the goal point for the joints in this command group.

    Parameters
    ----------
    point: trajectory_msgs.JointTrajectoryPoint
        the target point for all joints
    index: int
        index into point arrays
    joint_range: (float, float)
        the valid joint range
    fail_out_of_range_goal: bool
        whether to bound out-of-range goals to range or fail

    Returns
    -------
    dictionary with point, velocity, acceleration and contact_threshold set

    """
    goal_pos = point.positions[index]
    bounded_pos = hm.bound_ros_command(joint_range, goal_pos, fail_out_of_range_goal)
    if bounded_pos is None:
        raise InvalidGoalException(f'Received goal point that is out of bounds. '
                                   f'Range = {joint_range}, but goal point = {goal_pos}.')

    return {
        'position': bounded_pos,
        'velocity': point.velocities[index] if len(point.velocities) > index else None,
        'acceleration': point.accelerations[index] if len(point.accelerations) > index else None,
        'contact_threshold': point.effort[index] if len(point.effort) > index else None
    }


class BaseCommandGroup:
    def __init__(self, robot, joint_name, joint_range, acceptable_joint_error=0.015):
        """Create a simple command group to extend.

        Attributes
        ----------
        robot: Robot
            robot object
        name: str
            joint name
        range: tuple(float)
            acceptable joint bounds
        acceptable_joint_error: float
            how close to zero the error must reach

        """
        self.robot = robot
        self.name = joint_name
        self.range = joint_range
        self.acceptable_joint_error = acceptable_joint_error
        self.goal = None

    def get_component(self):
        raise NotImplementedError

    def get_state(self, robot_status):
        raise NotImplementedError


class HeadCommandGroup(BaseCommandGroup):
    def __init__(self, robot, motor_name, joint_name, calibrated_offset, calibrated_base_offset,
                 acceptable_joint_error):
        self.motor_name = motor_name
        motor = robot.head.motors[self.motor_name]
        range_ticks = motor.params['range_t']
        range_rad = (motor.ticks_to_world_rad(range_ticks[1]), motor.ticks_to_world_rad(range_ticks[0]))
        BaseCommandGroup.__init__(self, robot, joint_name, range_rad, acceptable_joint_error)
        self.calibrated_offset = calibrated_offset
        self.calibrated_base_offset = calibrated_base_offset

    def update_execution(self, robot_status, backlash_state):
        if backlash_state['head_pan_looked_left']:
            pan_backlash_correction = self.head_pan_calibrated_looked_left_offset
        else:
            pan_backlash_correction = 0.0
        pan_current = robot_status['head']['head_pan']['pos'] + \
            self.head_pan_calibrated_offset + pan_backlash_correction
        self.error = self.goal['position'] - pan_current
        return self.name, self.error

        return None

    def get_component(self):
        return self.robot.head.get_joint(self.motor_name)

    def get_state(self, robot_status):
        return robot_status['head'][self.motor_name]


class HeadPanCommandGroup(HeadCommandGroup):
    def __init__(self, robot, head_pan_calibrated_offset, head_pan_calibrated_looked_left_offset):
        HeadCommandGroup.__init__(self, robot, 'head_pan', 'joint_head_pan',
                                  head_pan_calibrated_offset, head_pan_calibrated_looked_left_offset, 0.15)


class HeadTiltCommandGroup(HeadCommandGroup):
    def __init__(self, robot, head_tilt_calibrated_offset,
                 head_tilt_calibrated_looking_up_offset,
                 head_tilt_backlash_transition_angle):
        HeadCommandGroup.__init__(self, robot, 'head_tilt', 'joint_head_tilt',
                                  head_tilt_calibrated_offset, head_tilt_calibrated_looking_up_offset, 0.52)
        self.head_tilt_backlash_transition_angle = head_tilt_backlash_transition_angle


class WristYawCommandGroup(BaseCommandGroup):
    def __init__(self, robot, range_rad):
        BaseCommandGroup.__init__(self, robot, 'joint_wrist_yaw', range_rad)

    def get_component(self):
        return self.robot.end_of_arm.motors['wrist_yaw']

    def get_state(self, robot_status):
        return robot_status['end_of_arm']['wrist_yaw']

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
import xacro
import yaml

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

# Mapping from entry in the stretch_body configuration to joints in the SRDF
CONFIGURATION_TRANSLATION = {
    'lift': ['lift'],
    'wrist_yaw': ['wrist_yaw'],
    'stretch_gripper': ['joint_gripper_finger_left', 'joint_gripper_finger_right'],
    'base': ['position/x', 'position/theta'],
    'arm': ['joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3'],
    'head': ['joint_head_pan', 'joint_head_tilt']
}

def load_joint_limits_from_config(mode='default'):
    """Translate the values from the robot configuration to params to be used by MoveIt."""
    params = {'joint_limits': {}}
    try:
        from stretch_body.device import Device
        d = Device()
        for config_name, joint_names in CONFIGURATION_TRANSLATION.items():
            config = d.robot_params[config_name]
            config_limits = config['motion'].get(mode, {})
            result = {}
            for name, abrev in [('velocity', 'vel'), ('acceleration', 'accel')]:
                cfg_name = f'{abrev}_m'
                if cfg_name in config_limits:
                    result[f'has_{name}_limits'] = True
                    result[f'max_{name}'] = config_limits[cfg_name]
                else:
                    result[f'has_{name}_limits'] = False
            for joint_name in joint_names:
                params['joint_limits'][joint_name] = dict(result)
    except (KeyError, ModuleNotFoundError):
        # We may reach here if HELLO_FLEET_ID or HELLO_FLEET_PATH is not set
        # or stretch_body.device is not on the PYTHONPATH
        # in which case we load the defaults
        print('Load from default')
        return load_yaml('stretch_moveit_config', 'config/default_joint_limits.yaml')
    return params

def generate_launch_description():

    # joystick_config_arg = DeclareLaunchArgument(
    #     "joystick_config",
    #     default_value="default_joystick.yaml",
    #     description="Joystick configuration file to load.",
    # )

    ignition_launch_py = PythonLaunchDescriptionSource(
        [get_package_share_directory("stretch_ignition"), "/launch/ignition.launch.py"]
    )
    ignition_launch = IncludeLaunchDescription(
        ignition_launch_py,
        # launch_arguments={
        #     "joint_limited": LaunchConfiguration("joint_limited"),
        # }.items(),
    )

    robot_description_config = xacro.process_file(os.path.join(get_package_share_directory('stretch_moveit_config'),
                                                            'config',
                                                            'stretch.xacro'),
    )
    robot_description = {'robot_description' : robot_description_config.toxml()}

    robot_description_semantic_config = load_file('stretch_moveit_config', 'config/stretch_description.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml('stretch_moveit_config', 'config/kinematics.yaml')

    joint_limits_yaml = {'robot_description_planning': load_joint_limits_from_config()}

    # Planning Functionality
    ompl_planning_pipeline_config = { 'ompl' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 0.1 } }
    ompl_planning_yaml = load_yaml('stretch_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    controllers_yaml = load_yaml('stretch_moveit_config', 'config/moveit_simple_controllers.yaml')
    # controllers_yaml = load_yaml('stretch_moveit_config', 'config/ros_controllers.yaml')
    moveit_controllers = { 'moveit_simple_controller_manager' : controllers_yaml,
                            'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    trajectory_execution = {'allow_trajectory_execution': True,
                            'moveit_manage_controllers': False,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 0.01}

    planning_scene_monitor_parameters = {"publish_planning_scene": True,
                                        "publish_geometry_updates": True,
                                        "publish_state_updates": True,
                                        "publish_transforms_updates": True}

    # Start the demo node
    moveit_cpp_node = Node(package='stretch_roscon_demos',
                            executable='moveit_cpp_demo',
                            output='screen',
                            parameters=[robot_description,
                                        robot_description_semantic,
                                        kinematics_yaml,
                                        joint_limits_yaml,
                                        ompl_planning_pipeline_config,
                                        trajectory_execution,
                                        moveit_controllers,
                                        planning_scene_monitor_parameters])

    # RViz
    rviz_config_file = get_package_share_directory('stretch_moveit_config') + "/launch/moveit_ignition.rviz"
    rviz_node = Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='log',
                    arguments=['-d', rviz_config_file],
                    parameters=[robot_description,
                                robot_description_semantic,
                                ompl_planning_pipeline_config,
                                kinematics_yaml])

    # controller_launch_py = PythonLaunchDescriptionSource(
    #     [
    #         get_package_share_directory("tigerx_config"),
    #         "/launch/pounce_controller.launch.py",
    #     ]
    # )
    # controller_launch = IncludeLaunchDescription(
    #     controller_launch_py,
    #     launch_arguments={
    #         "joint_limited": LaunchConfiguration("joint_limited"),
    #         "speed_factor": LaunchConfiguration("speed_factor"),
    #     }.items(),
    # )

    # joy_launch_py = PythonLaunchDescriptionSource(
    #     [get_package_share_directory("pounce_teleop"), "/launch/joy_teleop.launch.py"]
    # )
    # joy_launch = IncludeLaunchDescription(
    #     joy_launch_py,
    #     launch_arguments={
    #         "config_name": LaunchConfiguration("joystick_config")
    #     }.items(),
    # )

    return LaunchDescription(
        [
            # ignition_launch,
            moveit_cpp_node,
            # joystick_config_arg,
            # controller_launch,
            # joy_launch,
        ]
    )
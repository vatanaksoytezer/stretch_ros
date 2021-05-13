import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    robot_description_path =  os.path.join(get_package_share_directory("stretch_description"), "urdf", "stretch_description.xacro")
    robot_description_config = xacro.process_file(
        robot_description_path
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Ignition gazebo
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_worlds = get_package_share_directory('stretch_gazebo')
    world_dir = os.path.join(pkg_worlds, 'worlds', 'empty_world.sdf')
    world_str = "-r " + world_dir
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        # launch_arguments={'ign_args': '-r empty.sdf'}.items(),
        launch_arguments={'ign_args': world_str}.items(),
    )

    # RViz
    pkg_stretch_gazebo = get_package_share_directory('stretch_gazebo')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_stretch_gazebo, 'config', 'stretch_gazebo.rviz')],
        parameters=[]
    )

    # Spawn
    # gazebo_pkg_path =  os.path.join(get_package_share_directory("stretch_gazebo"), "models", "stretch_ignition", "model.sdf")
    gazebo_pkg_path =  os.path.join(get_package_share_directory("stretch_description"), "urdf", "stretch.sdf")
    spawn = Node(package='ros_ign_gazebo', executable='create',
                arguments=[
                    '-name', 'stretch',
                    '-file', gazebo_pkg_path
                    ],
                output='screen',
                )

    # Ign Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
                # Velocity commands (ROS2 -> IGN)
                '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                # JointTrajectory bridge (ROS2 -> IGN)
                '/joint_trajectory@trajectory_msgs/msg/JointTrajectory]ignition.msgs.JointTrajectory',
                # Odometry (IGN -> ROS2)
                '/model/stretch/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                # odom->base_link tf (IGN -> ROS2)
                '/model/stretch/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                # Clock (IGN -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                # Joint states (IGN -> ROS2)
                '/world/empty_world/model/stretch/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                # JointTrajectoryProgress bridge (IGN -> ROS2)
                '/joint_trajectory_progress@std_msgs/msg/Float32[ignition.msgs.Float',
                # Lidar bridge is broken in Ign for now (IGN -> ROS2)
                '/lidar@sensor_msgs/LaserScan[ignition.msgs.LaserScan',
                '/lidar/points@sensor_msgs/PointCloud2[ignition.msgs.PointCloudPacked',
                # IMU (IGN -> ROS2)
                '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                # Magnetometer (IGN -> ROS2)
                '/magnetometer@sensor_msgs/msg/MagneticField[ignition.msgs.Magnetometer',
                ],
        remappings=[
            ("/model/stretch/tf", "tf"),
            ("/world/empty_world/model/stretch/joint_state", "joint_states"),
            ("/model/stretch/odometry", "odom"),
            ("/imu", "imu/data"),
            ("/magnetometer", "mag"),
        ],
        output='screen'
    )

    # Controllers 
    # TODO: Use ros_ign_control when it is ready

    return LaunchDescription(
        [
            # Launch Arguments
            DeclareLaunchArgument(
                'use_sim_time',
                default_value=use_sim_time,
                description="If true, use simulated clock"),
            # Nodes and Launches
            gazebo,
            spawn,
            bridge,
            # rviz,
            robot_state_publisher,
            # ros2_control_node,
        ]
    )
import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

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

    # Launch Arguments
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="If true, use simulated clock"
    )

    # Ignition gazebo
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': '-r empty.sdf'}.items(),
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
    gazebo_pkg_path =  os.path.join(get_package_share_directory("stretch_gazebo"), "models", "stretch_ignition", "model.sdf")
    spawn = Node(package='ros_ign_gazebo', executable='create',
                arguments=[
                    '-name', 'robot',
                    '-file', gazebo_pkg_path
                    ],
                output='screen')

    # Ign Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/model/robot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/model/robot/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'
                ],
        remappings=[
            ("/model/robot/tf", "tf"),
            ("/model/robot/odometry", "odom")
        ],
        output='screen'
    )

    # Robot state publisher
    # robot_desc_path =  os.path.join(get_package_share_directory("stretch_gazebo"), "urdf", "stretch_gazebo.urdf.xacro")
    # robot_description_config = xacro.process_file(
    #     robot_desc_path
    # )
    # robot_description = {"robot_description": robot_description_config.toxml()}
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="both",
    #     parameters=[],
    # )

    return LaunchDescription([
        use_sim_time,
        gazebo,
        spawn,
        bridge,
        rviz,
        # robot_state_publisher,
    ])
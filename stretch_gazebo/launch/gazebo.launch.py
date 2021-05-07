import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

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

    # Import model
    gazebo_pkg_path =  os.path.join(get_package_share_directory("stretch_gazebo"), "urdf", "stretch_gazebo.urdf.xacro")
    robot_description_config = xacro.process_file(
        gazebo_pkg_path
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

    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    # Joint state publisher GUI (for Debugging)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
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
        parameters=[robot_description]
    )

    # Spawn
    spawn = Node(package='ros_ign_gazebo', executable='create',
                arguments=[
                    '-name', 'robot',
                    # 'file', gazebo_pkg_path
                    '-topic', '/robot_description'
                    ],
                output='screen')

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz,
        gazebo,
        spawn,
    ])
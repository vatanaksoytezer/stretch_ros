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
    gazebo_pkg_path =  os.path.join(get_package_share_directory("stretch_gazebo"), "urdf", "stretch.sdf")
    spawn = Node(package='ros_ign_gazebo', executable='create',
                arguments=[
                    '-name', 'robot',
                    '-file', gazebo_pkg_path
                    ],
                output='screen')

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn,
        bridge,
        rviz,
    ])
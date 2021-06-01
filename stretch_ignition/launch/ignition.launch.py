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
    pkg_stretch_ignition = get_package_share_directory('stretch_ignition')
    # TODO: Make world argument modular with bridge
    world_dir = os.path.join(pkg_stretch_ignition, 'worlds', 'empty_world.sdf')
    # world_dir = os.path.join(get_package_share_directory('aws_robomaker_small_house_world'), 'worlds', 'small_house.sdf')
    world_str = "-r " + world_dir
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': world_str}.items(),
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_stretch_ignition, 'rviz', 'stretch_ignition.rviz')],
        parameters=[]
    )

    # Spawn
    stretch_sdf_path =  os.path.join(get_package_share_directory("stretch_ignition"), "sdf", "stretch.sdf")
    spawn = Node(package='ros_ign_gazebo', executable='create',
                arguments=[
                    '-name', 'stretch',
                    '-file', stretch_sdf_path,
                    '-z', '0.1',
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
                '/world/default/model/stretch/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                # JointTrajectoryProgress bridge (IGN -> ROS2)
                '/joint_trajectory_progress@std_msgs/msg/Float32[ignition.msgs.Float',
                # Lidar (IGN -> ROS2)
                '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                # Base IMU (IGN -> ROS2)
                '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                # Base Magnetometer (IGN -> ROS2)
                '/magnetometer@sensor_msgs/msg/MagneticField[ignition.msgs.Magnetometer',
                # Wrist Accelerometer (IGN -> ROS2)
                '/wrist_imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                # Realsense IMU
                '/realsense_imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                # RGBD Camera (TODO: Port realsense) (IGN -> ROS2)
                '/world/default/model/stretch/link/link_realsense_optical/sensor/realsense_d435/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/world/default/model/stretch/link/link_realsense_optical/sensor/realsense_d435/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/world/default/model/stretch/link/link_realsense_optical/sensor/realsense_d435/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                '/world/default/model/stretch/link/link_realsense_optical/sensor/realsense_d435/camera_info@sensor_msgs/msg/CameraInfo[ignition::msgs::CameraInfo',
                ],
        remappings=[
            ("/model/stretch/tf", "tf"),
            ("/world/default/model/stretch/joint_state", "joint_states"),
            ("/world/default/model/stretch/link/link_realsense_optical/sensor/realsense_d435/image", "/realsense/color/image_raw"),
            ("/world/default/model/stretch/link/link_realsense_optical/sensor/realsense_d435/camera_info", "/realsense/depth/camera_info"),
            ("/world/default/model/stretch/link/link_realsense_optical/sensor/realsense_d435/points", "/realsense/depth/points"),
            ("/world/default/model/stretch/link/link_realsense_optical/sensor/realsense_d435/depth_image", "/realsense/depth/image_raw"),
            ("/model/stretch/odometry", "odom"),
            ("/imu", "imu/data"),
            ("/magnetometer", "mag"),
            ("/wrist_imu", "wrist_imu/data"),
            ("/realsense_imu", "realsense/imu/data"),
        ],
        output='screen'
    )

    # Sensor Static TFs (until we can give frame_id argument in ignition sensor plugins)
    lidar_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='lidar_static_transform_publisher',
                        output='log',
                        arguments=['0', '0.0', '0.1664', '0.0', '0.0', '0.0', 'base_link', 'stretch/link_laser/gpu_lidar'])
    imu_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='imu_static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'stretch/base_link/imu'])
    mag_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='mag_static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'stretch/base_link/magnetometer'])
    wrist_imu_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='wrist_imu_static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'link_wrist_yaw', 'stretch/link_wrist_yaw/wrist_imu'])
    realsense_imu_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='realsense_imu_static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'link_head_tilt', 'stretch/link_head_tilt/realsense_imu'])
    # TODO (vatanaksoytezer): Port realsense and remove rgbd
    rgbd_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='rgbd_static_transform_publisher',
                        output='log',
                        # arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_depth_optical_frame', 'stretch/camera_depth_optical_frame/rgbd_camera'])
                        arguments=['0.0', '0.0', '0.0', '1.5708', '-1.5708', '0', 'camera_depth_optical_frame', 'stretch/link_realsense_optical/realsense_d435'])
                        # arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'link_head_tilt', 'stretch/link_head_tilt/rgbd_camera'])

    # Controllers 
    # TODO (vatanaksoytezer): Use ros_ign_control when it is ready
    stretch_ignition_control_node = Node(
        package="stretch_ignition_control",
        executable="stretch_ignition_control_action_server",
        name="stretch_ignition_control",
        output="screen",
    )    

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
            robot_state_publisher,
            stretch_ignition_control_node,
            lidar_static_tf,
            imu_static_tf,
            mag_static_tf,
            wrist_imu_static_tf,
            rgbd_static_tf,
            realsense_imu_static_tf,
            rviz,
        ]
    )

#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('bruno_description'),
            'launch', 'description.launch.py')),
    )

    # publish_static_tf is disabled here: robot_state_publisher (above) already
    # publishes base_link -> laser_frame from the URDF, so this would otherwise
    # duplicate that transform with a second publisher.
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ldlidar_stl_ros2'),
            'launch', 'ld14p.launch.py')),
        launch_arguments={'publish_static_tf': 'false'}.items(),
    )

    base_node = Node(
        package='bruno_base',
        executable='bruno_base_node',
        name='bruno_base_node',
        output='screen',
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            # Left empty: nothing on Bruno publishes ground-truth odometry.
            # If this is left at rf2o's default ('/base_pose_ground_truth'),
            # GT_pose_initialized never becomes true and LaserCallBack silently
            # never processes a single scan - rf2o would never produce odometry.
            'init_pose_from_topic': '',
            'freq': 10.0,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(description_launch)
    ld.add_action(lidar_launch)
    ld.add_action(base_node)
    ld.add_action(rf2o_node)
    return ld

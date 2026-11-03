#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('bruno_bringup'),
            'launch', 'bruno_bringup.launch.py')),
    )

    slam_params_file = os.path.join(
        get_package_share_directory('bruno_bringup'),
        'config', 'slam_toolbox_params.yaml')

    # Reuses slam_toolbox's own launch file rather than starting the node
    # directly: async_slam_toolbox_node is a lifecycle node and needs the
    # configure/activate transition events this file already wires up, or
    # it stays inactive and never processes a single scan. use_sim_time is
    # explicitly forced to false - this launch file's own default is true,
    # which would stall it on real hardware (no /clock topic here).
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'),
            'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'false',
        }.items(),
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8765}],
    )

    ld = LaunchDescription()
    ld.add_action(bringup_launch)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(foxglove_bridge_node)
    return ld

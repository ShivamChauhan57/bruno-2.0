#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

'''
LD14P uses the same 230400-baud packet protocol as the LD19, so this
launch file reuses product_name: LDLiDAR_LD19 (no dedicated LD14 entry
exists in ldlidar::LDType).

Parameter Description:
---
- Set laser scan directon:
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

def generate_launch_description():
  # Set to false when base_link -> laser_frame is already published elsewhere
  # (e.g. by robot_state_publisher from a URDF, as in bruno_bringup) to avoid
  # two publishers broadcasting the same static transform.
  publish_static_tf_arg = DeclareLaunchArgument(
      'publish_static_tf', default_value='true')

  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='ldlidar_stl_ros2',
      executable='ldlidar_stl_ros2_node',
      name='LD14P',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'topic_name': 'scan'},
        {'frame_id': 'laser_frame'},
        {'port_name': '/dev/lidar'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0}
      ]
  )

  # base_link to laser_frame static tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_laser_frame',
    arguments=[
      '--x', '0.08', '--y', '0.0', '--z', '0.12',
      '--yaw', '0', '--pitch', '0', '--roll', '0',
      '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'
    ],
    condition=IfCondition(LaunchConfiguration('publish_static_tf'))
  )

  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(publish_static_tf_arg)
  ld.add_action(ldlidar_node)
  ld.add_action(base_link_to_laser_tf_node)

  return ld

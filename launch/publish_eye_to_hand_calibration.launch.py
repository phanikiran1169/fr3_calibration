# publish_eye_to_hand_calibration.launch.py
# Publishes a saved eye-to-hand calibration as a static TF from fr3_link0
# to front_camera_link. The saved transform ends at the optical frame; this
# launch composes it with the camera driver's optical->root transform from
# /tf_static so the camera's full frame tree is reachable from the robot.
# The camera driver must be running before this launch.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def _default_calibration_name():
    pkg_share = get_package_share_directory('fr3_calibration')
    config_path = os.path.join(pkg_share, 'config', 'calibration_eye_to_hand.yaml')
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    return cfg['calibration']['name']


def generate_launch_description():
    name_arg = DeclareLaunchArgument(
        'calibration_name',
        default_value=_default_calibration_name(),
        description='Name of the saved easy_handeye2 calibration to publish',
    )
    camera_root_arg = DeclareLaunchArgument(
        'camera_root_frame',
        default_value='front_camera_link',
        description='Camera root frame to attach to the robot base. The '
                    'camera driver must already publish a static TF from the '
                    'tracking base frame (front_camera_color_optical_frame) '
                    'to this frame.',
    )
    lookup_timeout_arg = DeclareLaunchArgument(
        'lookup_timeout',
        default_value='20.0',
        description='Seconds to wait for the camera driver static TFs before '
                    'giving up.',
    )

    publisher = Node(
        package='fr3_calibration',
        executable='calibration_publisher',
        name='calibration_publisher',
        output='screen',
        arguments=[
            '--calibration-name', LaunchConfiguration('calibration_name'),
            '--camera-root-frame', LaunchConfiguration('camera_root_frame'),
            '--lookup-timeout', LaunchConfiguration('lookup_timeout'),
        ],
    )

    return LaunchDescription([
        name_arg,
        camera_root_arg,
        lookup_timeout_arg,
        publisher,
    ])

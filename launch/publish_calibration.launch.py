# publish_calibration.launch.py
# fr3_calibration: publishes a saved easy_handeye2 calibration as a static TF.
# Run after a successful calibration; reads the .calib YAML from
# ~/.ros2/easy_handeye2/calibrations/<name>.calib (which is symlinked to
# ~/thesis/data/calib/easy_handeye2/calibrations/).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import yaml


def _default_calibration_name():
    pkg_share = get_package_share_directory('fr3_calibration')
    config_path = os.path.join(pkg_share, 'config', 'calibration.yaml')
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    return cfg['calibration']['name']


def generate_launch_description():
    name_arg = DeclareLaunchArgument(
        'calibration_name',
        default_value=_default_calibration_name(),
        description='Name of the saved easy_handeye2 calibration to publish',
    )

    easy_handeye2_share = get_package_share_directory('easy_handeye2')
    publish_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(easy_handeye2_share, 'launch', 'publish.launch.py')
        ),
        launch_arguments={
            'name': LaunchConfiguration('calibration_name'),
        }.items(),
    )

    return LaunchDescription([
        name_arg,
        publish_launch,
    ])

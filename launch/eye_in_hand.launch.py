# eye_in_hand.launch.py
# Eye-in-hand calibration session: realsense2_camera (wrist) + aruco_ros/single
# (marker detection) + easy_handeye2 (sampling and solving). The marker is
# fixed in the workspace; the camera is mounted on the FR3 flange.
#
# Frames (defaults from config/calibration.yaml):
#   robot_base_frame:      fr3_link0
#   robot_effector_frame:  fr3_link8
#   tracking_base_frame:   wrist_camera_color_optical_frame
#   tracking_marker_frame: aruco_marker_frame

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(name: str) -> dict:
    pkg_share = get_package_share_directory('fr3_calibration')
    with open(os.path.join(pkg_share, 'config', name)) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    calib_cfg = _load_yaml('calibration.yaml')
    marker_cfg = _load_yaml('marker.yaml')['aruco']

    calib_name_arg = DeclareLaunchArgument(
        'calibration_name',
        default_value=calib_cfg['calibration']['name'],
        description='easy_handeye2 calibration name (output filename '
                    '~/.ros2/easy_handeye2/calibrations/<name>.calib)',
    )
    marker_id_arg = DeclareLaunchArgument(
        'marker_id',
        default_value=str(marker_cfg['marker_id']),
        description='ArUco marker ID (must match the printed marker)',
    )
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value=str(marker_cfg['marker_size_m']),
        description='ArUco marker side length in metres '
                    '(read from config/marker.yaml; update there after measuring)',
    )

    # camera_namespace='/' makes topics /wrist_camera/... instead of
    # /wrist_camera/wrist_camera/... . serial_no pins this launch to the
    # wrist camera. The leading underscore forces string parsing of the
    # serial; numeric serials with leading zeros are otherwise rejected.
    serial_arg = DeclareLaunchArgument(
        'serial_no',
        default_value='_020522070946',
        description='RealSense serial number for the wrist camera.',
    )
    realsense_share = get_package_share_directory('realsense2_camera')
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_namespace': '/',
            'camera_name': 'wrist_camera',
            'serial_no': LaunchConfiguration('serial_no'),
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'enable_sync': 'true',
        }.items(),
    )

    # aruco_ros/single detects the marker and broadcasts the marker pose as
    # a TF (parent = camera_frame, child = marker_frame). The shipped
    # single.launch.py is hardcoded for stereo, so the node is instantiated
    # directly with our remaps. image_is_rectified=True is valid only if
    # CameraInfo.D is zeros for the published color stream.
    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': LaunchConfiguration('marker_size'),
            'marker_id': LaunchConfiguration('marker_id'),
            # Identical reference_frame and camera_frame skip the
            # camera->reference TF lookup; the marker pose is broadcast
            # directly in the camera optical frame.
            'reference_frame': calib_cfg['camera']['optical_frame'],
            'camera_frame': calib_cfg['camera']['optical_frame'],
            'marker_frame': calib_cfg['aruco']['marker_frame'],
        }],
        remappings=[
            ('/image', calib_cfg['camera']['image_topic']),
            ('/camera_info', calib_cfg['camera']['info_topic']),
        ],
    )

    # rqt_image_view shows the detection overlay and keeps aruco_ros active:
    # aruco_ros/single skips detection when no subscriber exists on its
    # non-TF output topics, so without a viewer no marker TF is broadcast.
    aruco_viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='aruco_result_viewer',
        arguments=['/aruco_single/result'],
        output='screen',
    )

    # easy_handeye2 samples robot and tracking TFs and solves AX=XB.
    easy_handeye2_share = get_package_share_directory('easy_handeye2')
    easy_handeye2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(easy_handeye2_share, 'launch', 'calibrate.launch.py')
        ),
        launch_arguments={
            'name': LaunchConfiguration('calibration_name'),
            'calibration_type': 'eye_in_hand',
            'robot_base_frame': calib_cfg['robot']['base_frame'],
            'robot_effector_frame': calib_cfg['robot']['effector_frame'],
            'tracking_base_frame': calib_cfg['camera']['optical_frame'],
            'tracking_marker_frame': calib_cfg['aruco']['marker_frame'],
        }.items(),
    )

    return LaunchDescription([
        calib_name_arg,
        marker_id_arg,
        marker_size_arg,
        serial_arg,
        realsense_launch,
        aruco_single,
        aruco_viewer,
        easy_handeye2_launch,
    ])

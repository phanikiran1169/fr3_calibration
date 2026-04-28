# eye_in_hand.launch.py
# fr3_calibration: brings up the full eye-in-hand calibration session.
# Wires together: realsense2_camera (RGB-D + camera_info), aruco_ros/single
# (marker detection + TF broadcast), and easy_handeye2 (sampling + solving).
#
# The camera is mounted on the FR3 flange. The ArUco marker is fixed in the
# workspace (taped to a rigid board on the table; must not move during the
# session).
#
# Frames (defaults from config/calibration.yaml):
#   robot_base_frame:      fr3_link0
#   robot_effector_frame:  fr3_link8
#   tracking_base_frame:   camera_color_optical_frame
#   tracking_marker_frame: aruco_marker_frame  (broadcast by aruco_ros/single)
#
# Marker geometry is read from config/marker.yaml (single source of truth).

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

    # 1. RealSense camera
    # rs_launch.py defaults camera_namespace='camera' AND camera_name='camera',
    # which composes topics as /camera/camera/... . We override camera_namespace
    # to '/' so topics come out as /camera/color/image_raw — matching what
    # fr3_data_collection/config/record_config.yaml expects.
    realsense_share = get_package_share_directory('realsense2_camera')
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_namespace': '/',
            'camera_name': 'camera',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'enable_sync': 'true',
        }.items(),
    )

    # 2. aruco_ros/single — detects the marker and broadcasts a TF
    #    parent = camera_frame (or reference_frame if non-empty)
    #    child  = marker_frame
    #
    #    aruco_ros/launch/single.launch.py is hardcoded for stereo cameras,
    #    so we instantiate the node directly here with our remaps.
    #
    #    image_is_rectified=True tells aruco_ros to use CameraInfo.P with zero
    #    distortion. RealSense color is generally published with distortion
    #    already applied to the image, but verify on first run by checking that
    #    /camera/color/camera_info has D = [0, 0, 0, 0, 0]. If D is non-zero,
    #    either set image_is_rectified=False or switch to a rectified topic.
    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': LaunchConfiguration('marker_size'),
            'marker_id': LaunchConfiguration('marker_id'),
            # reference_frame and camera_frame are intentionally identical — this
            # makes aruco_ros skip the camera→reference TF lookup and broadcast
            # the marker pose directly in the camera optical frame.
            'reference_frame': calib_cfg['camera']['optical_frame'],
            'camera_frame': calib_cfg['camera']['optical_frame'],
            'marker_frame': calib_cfg['aruco']['marker_frame'],
        }],
        remappings=[
            ('/image', calib_cfg['camera']['image_topic']),
            ('/camera_info', calib_cfg['camera']['info_topic']),
        ],
    )

    # 3. rqt_image_view — shows the marker-detection overlay AND keeps aruco_ros awake.
    #    aruco_ros/single early-returns from its image callback when no subscriber exists
    #    on its non-TF output topics (pose, transform, marker, pixel, position, debug,
    #    image). Without a subscriber it never runs detection, never broadcasts TF, and
    #    easy_handeye2 reports "tracking system base/marker frames not connected".
    #    rqt_image_view subscribes to /aruco_single/result, which keeps the callback
    #    active. The window also doubles as live feedback that the marker is being
    #    detected before clicking "Take Sample".
    aruco_viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='aruco_result_viewer',
        arguments=['/aruco_single/result'],
        output='screen',
    )

    # 4. easy_handeye2 — samples robot and tracking TFs, solves AX=XB
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
        realsense_launch,
        aruco_single,
        aruco_viewer,
        easy_handeye2_launch,
    ])

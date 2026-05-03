# fr3_calibration

ROS 2 package for hand-eye calibration on the Franka FR3. Wires `realsense2_camera`, `aruco_ros`, and `easy_handeye2` into a single launch flow with FR3-specific frames. Supports both eye-in-hand (wrist camera mounted on the flange) and eye-to-hand (front camera fixed in the workspace) calibration.

## Layout

```
config/
  marker.yaml                          ArUco marker geometry (id, size)
  calibration.yaml                     Eye-in-hand: frames, topics, name
  calibration_eye_to_hand.yaml         Eye-to-hand: frames, topics, name
launch/
  eye_in_hand.launch.py                Wrist camera + aruco + easy_handeye2 GUI
  eye_to_hand.launch.py                Front camera + aruco + easy_handeye2 GUI
  move_group.launch.py                 MoveIt move_group on the inference PC (planner only)
  publish_eye_in_hand_calibration.launch.py    Broadcast saved wrist calibration as static TF
  publish_eye_to_hand_calibration.launch.py    Broadcast saved front calibration as static TF
fr3_calibration/
  make_marker.py                       Stage a printable marker (JPG + Letter/A4 PDFs)
  calibration_pose_recorder.py         Record joint poses to YAML
  calibration_pose_runner.py           Replay recorded poses via MoveIt
  calibration_publisher.py             Compose saved calibration with camera /tf_static
rviz/
  fr3_calib.rviz                       RViz config: robot model + camera point cloud
data/                                  Generated artifacts (gitignored)
```

## Frames

| Mode | Robot frame | Tracking base (camera optical) | Camera root |
|------|-------------|--------------------------------|-------------|
| eye-in-hand | `fr3_link8` | `wrist_camera_color_optical_frame` | `wrist_camera_link` |
| eye-to-hand | `fr3_link0` | `front_camera_color_optical_frame` | `front_camera_link` |

The marker frame is `aruco_marker_frame`, broadcast by `aruco_ros/single` as a child of the camera optical frame.

## Build

```bash
cd <workspace>
colcon build --packages-select fr3_calibration --symlink-install
source install/setup.bash
```

## Output Path

`easy_handeye2` writes calibrations to `~/.ros2/easy_handeye2/calibrations/<name>.calib` (path hardcoded in the package). Symlink `~/.ros2/easy_handeye2` to `<package>/data/easy_handeye2/` so calibrations are stored under the package:

```bash
mkdir -p <workspace>/src/fr3_calibration/data/easy_handeye2/calibrations
mkdir -p ~/.ros2
ln -s <workspace>/src/fr3_calibration/data/easy_handeye2 ~/.ros2/easy_handeye2
```

## Marker

`aruco_ros` ships pre-rendered marker JPGs at 5 cm for IDs 26 and 582 from the `ARUCO_MIP_36h12` dictionary. The default config uses ID 26.

```bash
ros2 run fr3_calibration make_marker
```

This stages the JPG and renders Letter and A4 PDFs in `data/`. Print at 100% / "Actual Size", measure the printed marker side with calipers, and update `marker_size_m` in `config/marker.yaml`.

## Eye-in-Hand Calibration

The wrist camera is mounted on the FR3 flange. The marker is fixed in the workspace.

Prerequisites in separate terminals:

```bash
# RT control PC
ros2 launch franka_bringup franka.launch.py robot_ip:=<ip> arm_id:=fr3
ros2 run controller_manager spawner fr3_arm_controller \
    -t joint_trajectory_controller/JointTrajectoryController \
    --param-file <franka_fr3_moveit_config>/config/fr3_ros_controllers.yaml

# Inference PC (provides robot_state_publisher and the TF tree)
ros2 launch fr3_teleop teleop.launch.py
```

Calibration session:

```bash
ros2 launch fr3_calibration eye_in_hand.launch.py
```

This starts the RealSense (wrist camera, pinned by serial), `aruco_ros/single`, an `rqt_image_view` window on `/aruco_single/result`, and the `easy_handeye2` rqt calibrator. The image window is also required: `aruco_ros/single` only runs detection while a subscriber exists on its non-TF output topics.

In the rqt calibrator GUI: drive the arm to ≥15 diverse poses (large rotations on each axis), click "Take Sample" at each, then "Compute" and "Save". The result is written to `data/easy_handeye2/calibrations/fr3_eye_in_hand.calib`.

## Eye-to-Hand Calibration

The front camera is fixed in the workspace. The marker is rigidly attached to `fr3_link7` (or whichever link is set as `robot.effector_frame` in `config/calibration_eye_to_hand.yaml`).

Same prerequisites as eye-in-hand. Calibration session:

```bash
ros2 launch fr3_calibration eye_to_hand.launch.py
```

This starts the RealSense (front camera, pinned by serial), `aruco_ros/single`, the image viewer, and `easy_handeye2` in `eye_on_base` mode. Drive the arm so the marker is visible to the front camera at each pose, sample, compute, save. Result: `data/easy_handeye2/calibrations/fr3_eye_to_hand.calib`.

## Pose Recording and Replay

For repeatable calibration, record a pose list once and replay it.

Record:

```bash
ros2 run fr3_calibration calibration_pose_recorder           # default: eye_in_hand_calib_poses.yaml
ros2 run fr3_calibration calibration_pose_recorder \
    --out config/eye_to_hand_calib_poses.yaml                # eye-to-hand
```

Drive the arm to each pose (joystick teleop or programming-mode hand-guide), press Enter to capture. The script reads `/joint_states` and writes the joint angles to YAML.

Replay (in addition to the calibration session above):

```bash
# move_group is required; if fr3_teleop's joystick.launch.py is not running,
# launch the standalone planner:
ros2 launch fr3_calibration move_group.launch.py

# Replay
ros2 run fr3_calibration calibration_pose_runner             # eye-in-hand
ros2 run fr3_calibration calibration_pose_runner \
    --poses config/eye_to_hand_calib_poses.yaml              # eye-to-hand
```

The runner drives through poses via `pymoveit2` at 10% velocity by default (`--vel-scale 0.2` to speed up). The operator holds the EAD throughout. At each pose, click "Take Sample" in the GUI, then press Enter in the runner terminal. After the last pose, click "Compute" and "Save" in the GUI.

If MoveIt Servo is running (e.g., from `fr3_teleop/launch/joystick.launch.py`), stop it before replay:

```bash
ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger {}
```

Servo continuously publishes to `/fr3_arm_controller/joint_trajectory` and overrides the runner's action goals, producing instantaneous SUCCEEDED responses with no actual motion.

## Publish the Saved Calibration

```bash
ros2 launch fr3_calibration publish_eye_in_hand_calibration.launch.py
ros2 launch fr3_calibration publish_eye_to_hand_calibration.launch.py
```

Each broadcasts a single static TF: `fr3_link8 -> wrist_camera_link` for eye-in-hand, `fr3_link0 -> front_camera_link` for eye-to-hand. The respective camera driver must be running first; the publisher reads the camera's `optical_frame -> camera_link` from `/tf_static` and composes it with the saved calibration.

## Dependencies

- `realsense2_camera` (apt)
- `aruco_ros` (source: `pal-robotics/aruco_ros` humble-devel)
- `easy_handeye2` (source)
- `rqt_image_view` (apt)
- `pymoveit2` (apt) — for `calibration_pose_runner`
- `franka_fr3_moveit_config` (source: `franka_ros2` v0.1.15) — for `move_group.launch.py`
- `python3-transforms3d`, `python3-numpy` (apt) — for `calibration_publisher`
- `python3-reportlab` (apt) — for marker PDF generation

# fr3_calibration

ROS 2 package for hand-eye calibration on the Franka FR3. Wires `realsense2_camera`, `aruco_ros` (pal-robotics), and `easy_handeye2` into a single launch flow with FR3-specific frames matching `fr3_data_collection` and `fr3_teleop`.

## Contents

| File | Purpose |
|------|---------|
| `config/marker.yaml` | ArUco marker geometry (id, size) — single source of truth |
| `config/calibration.yaml` | Frames, topics, calibration name |
| `launch/eye_in_hand.launch.py` | Bring up RealSense + aruco_ros/single + easy_handeye2 |
| `launch/publish_calibration.launch.py` | Publish a saved calibration as a static TF |
| `fr3_calibration/make_marker.py` | Stage a printable ArUco marker JPG |

## Frames

Configured in `config/calibration.yaml` to match `fr3_teleop/config/servo_config.yaml`:

| Role | Frame |
|------|-------|
| `robot_base_frame` | `fr3_link0` |
| `robot_effector_frame` | `fr3_link8` |
| `tracking_base_frame` (camera optical) | `camera_color_optical_frame` |
| `tracking_marker_frame` | `aruco_marker_frame` (broadcast by `aruco_ros/single`) |

## Output

`easy_handeye2` saves calibrations to `~/.ros2/easy_handeye2/calibrations/<name>.calib` (hardcoded in the package). For this project the path is symlinked to `~/thesis/data/calib/easy_handeye2/calibrations/`:

```bash
mkdir -p ~/thesis/data/calib/easy_handeye2/calibrations
mkdir -p ~/.ros2
ln -s ~/thesis/data/calib/easy_handeye2 ~/.ros2/easy_handeye2
```

## Build

```bash
cd ~/franka_ws
colcon build --packages-select fr3_calibration --symlink-install
source install/setup.bash
```

## Print the marker

`aruco_ros` ships pre-rendered marker JPGs at 5 cm (IDs 26 and 582). The default config uses ID 26.

```bash
ros2 run fr3_calibration make_marker
# Copies the prebuilt JPG to ~/thesis/data/calib/marker.jpg
```

Print at 100% / "actual size" — disable any "fit to page" or "scale to printable area" option. The shipped JPG carries embedded DPI metadata that yields a 5 cm marker only at 100% scale; "fit to page" can produce a 4-7 cm marker depending on paper size (Letter vs A4) and printer margins. Print one, measure the black square's side with calipers, and update `config/marker.yaml` (`marker_size_m`) to the measured value before calibrating.

To use a different ID or custom size, edit `config/marker.yaml` and either add an entry to `_ARUCO_ROS_PREBUILT` in `make_marker.py` or generate a custom marker with `cv2.aruco`.

## Calibrate (eye-in-hand)

The camera is mounted on the FR3 flange. The marker is fixed in the workspace (taped to a rigid board on the table; must not move during the session).

`easy_handeye2` requires a complete TF chain `fr3_link0 → fr3_link8`, which `robot_state_publisher` provides. This launch file does NOT start `robot_state_publisher`. Bring up `fr3_teleop` (or any other launch that starts `robot_state_publisher` against the FR3 URDF) before launching calibration:

```bash
# Terminal 1: franka_ros2 on RT PC (separate machine)
# Terminal 2: bring up FR3 teleop on the laptop (provides robot_state_publisher + TF tree)
ros2 launch fr3_teleop teleop.launch.py
# Terminal 3: calibration session on the laptop
ros2 launch fr3_calibration eye_in_hand.launch.py
```

This starts the RealSense, runs `aruco_ros/single` against the camera stream, launches `easy_handeye2`'s rqt calibrator, and opens an `rqt_image_view` window on `/aruco_single/result`. The image window doubles as live detection feedback and is also required: `aruco_ros/single` only runs detection while at least one subscriber exists on its non-TF output topics, so removing the viewer means no TF is broadcast and `easy_handeye2` cannot sample. The marker must be in the camera's FOV.

In the rqt calibrator UI: kinesthetically move the FR3 to ≥15 diverse poses (large rotations on each axis), click "take sample" at each, then "compute" and "save". The result lands at `~/thesis/data/calib/easy_handeye2/calibrations/fr3_eye_in_hand.calib`.

The RT PC and laptop must agree on wall time within a few milliseconds; otherwise `easy_handeye2` fails with `Lookup would require extrapolation into the past`. Use any standard ROS 2 multi-machine NTP setup.

### Override at the command line

```bash
ros2 launch fr3_calibration eye_in_hand.launch.py \
    marker_id:=582 \
    marker_size:=0.050
```

Edit `config/marker.yaml` for persistent changes (it's the single source of truth for marker geometry).

## Publish the saved calibration

```bash
ros2 launch fr3_calibration publish_calibration.launch.py
```

Add this to the daily startup so downstream code (e.g., `fr3_data_collection`) sees the calibrated frame.

## Dependencies

- `realsense2_camera` (apt: `ros-humble-realsense2-camera`)
- `aruco_ros` (source-built from pal-robotics/aruco_ros humble-devel)
- `easy_handeye2` (source-built)
- `rqt_image_view` (apt: `ros-humble-rqt-image-view`)

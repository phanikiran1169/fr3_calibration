# calibration_publisher.py
# Publishes a saved easy_handeye2 calibration as a static TF from the robot
# frame (effector for eye_in_hand, base for eye_on_base) to the camera
# root frame. The saved transform stops at the optical frame; this node
# composes it with the camera driver's optical->root static TF (from
# /tf_static) so the camera's full frame tree is reachable from the robot.

import sys
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from transforms3d.affines import compose, decompose
from transforms3d.quaternions import mat2quat, quat2mat


def _xyzw_to_wxyz(q):
    return (q[3], q[0], q[1], q[2])


def _wxyz_to_xyzw(q):
    return (q[1], q[2], q[3], q[0])


def _make_4x4(t, q_xyzw):
    """Build a 4x4 matrix from translation and quaternion (x, y, z, w)."""
    R = quat2mat(_xyzw_to_wxyz(q_xyzw))
    return compose(np.asarray(t, dtype=float), R, np.ones(3))


def _split_4x4(M):
    """Decompose a 4x4 matrix into translation and quaternion (x, y, z, w)."""
    t, R, _, _ = decompose(M)
    return t, _wxyz_to_xyzw(mat2quat(R))


def _load_calibration(name):
    """Load a saved calibration YAML from ~/.ros2/easy_handeye2/calibrations/."""
    path = Path.home() / '.ros2' / 'easy_handeye2' / 'calibrations' / f'{name}.calib'
    if not path.exists():
        raise FileNotFoundError(f'Calibration not found: {path}')
    with open(path) as f:
        data = yaml.safe_load(f)
    p = data['parameters']
    t = data['transform']['translation']
    r = data['transform']['rotation']

    # easy_handeye2 stores the transform with parent = robot_effector_frame
    # for eye_in_hand and parent = robot_base_frame for eye_on_base; the
    # child is always tracking_base_frame.
    calibration_type = p.get('calibration_type', 'eye_in_hand')
    if calibration_type == 'eye_in_hand':
        parent_frame = p['robot_effector_frame']
    elif calibration_type == 'eye_on_base':
        parent_frame = p['robot_base_frame']
    else:
        raise ValueError(f'Unknown calibration_type: {calibration_type!r}')

    return {
        'calibration_type': calibration_type,
        'parent_frame': parent_frame,
        'tracking_base_frame': p['tracking_base_frame'],
        'translation': np.array([t['x'], t['y'], t['z']], dtype=float),
        'rotation_xyzw': (r['x'], r['y'], r['z'], r['w']),
    }


class CalibrationPublisher(Node):
    """Composes the saved robot->tracking_base transform with the camera
    driver's tracking_base->camera_root transform (looked up from /tf_static)
    and broadcasts a single static TF: robot_frame -> camera_root. The
    robot_frame is the effector for eye_in_hand and the base for eye_on_base."""

    def __init__(self, calibration_name, camera_root_frame, lookup_timeout_s):
        super().__init__('calibration_publisher')
        self.calibration_name = calibration_name
        self.camera_root_frame = camera_root_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)

        self._published = False
        self._deadline_ns = self.get_clock().now().nanoseconds + int(lookup_timeout_s * 1e9)
        self._timer = self.create_timer(0.5, self._try_publish_once)

    def _try_publish_once(self):
        if self._published:
            return

        try:
            calib = _load_calibration(self.calibration_name)
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration: {e}')
            self._timer.cancel()
            rclpy.shutdown()
            return

        tracking_base = calib['tracking_base_frame']
        try:
            tf = self.tf_buffer.lookup_transform(
                tracking_base, self.camera_root_frame, rclpy.time.Time())
        except Exception as e:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns > self._deadline_ns:
                self.get_logger().error(
                    f'Timed out waiting for static TF '
                    f'{tracking_base} -> {self.camera_root_frame}: {e}. '
                    f'Is the camera driver running?')
                self._timer.cancel()
                rclpy.shutdown()
            return

        T_parent_track = _make_4x4(calib['translation'], calib['rotation_xyzw'])

        t = tf.transform.translation
        r = tf.transform.rotation
        T_track_root = _make_4x4(
            (t.x, t.y, t.z),
            (r.x, r.y, r.z, r.w),
        )

        T_parent_root = T_parent_track @ T_track_root
        translation, (qx, qy, qz, qw) = _split_4x4(T_parent_root)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = calib['parent_frame']
        msg.child_frame_id = self.camera_root_frame
        msg.transform.translation.x = float(translation[0])
        msg.transform.translation.y = float(translation[1])
        msg.transform.translation.z = float(translation[2])
        msg.transform.rotation.x = float(qx)
        msg.transform.rotation.y = float(qy)
        msg.transform.rotation.z = float(qz)
        msg.transform.rotation.w = float(qw)

        self.broadcaster.sendTransform(msg)
        self._published = True
        self._timer.cancel()
        self.get_logger().info(
            f'Published static TF ({calib["calibration_type"]}): '
            f'{calib["parent_frame"]} -> {self.camera_root_frame} '
            f'(t={np.round(translation, 4).tolist()}, '
            f'q=[{qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}])')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(
        description='Publish a saved easy_handeye2 calibration as a static TF '
                    'from the robot frame (effector for eye_in_hand, base for '
                    'eye_on_base) to the camera root frame.')
    parser.add_argument('--calibration-name', default='fr3_eye_in_hand',
                        help='Name of the saved calibration (default: fr3_eye_in_hand)')
    parser.add_argument('--camera-root-frame', default='wrist_camera_link',
                        help='Camera root frame to attach to the robot (default: wrist_camera_link)')
    parser.add_argument('--lookup-timeout', type=float, default=20.0,
                        help='Seconds to wait for the camera driver to publish '
                             'static TFs before giving up (default: 20)')
    args, _ = parser.parse_known_args(argv)

    rclpy.init()
    node = CalibrationPublisher(
        calibration_name=args.calibration_name,
        camera_root_frame=args.camera_root_frame,
        lookup_timeout_s=args.lookup_timeout,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)

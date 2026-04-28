# calibration_pose_recorder.py: Capture FR3 joint configurations to a YAML
# file for later replay by calibration_pose_runner during eye-in-hand calibration.

import argparse
import os
import sys
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState


FR3_JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3',
    'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]

# Reject joint_state snapshots older than this. franka_bringup publishes at
# 30 Hz; 0.5 s catches the case where the publisher has died but the latch
# still holds a stale message.
STALE_THRESHOLD_S = 0.5


class JointStateLatch(Node):
    """Subscribes to /joint_states and holds the most recent message that
    contains all FR3 arm joints. Gripper-only messages are ignored."""

    def __init__(self):
        super().__init__('calibration_pose_recorder')
        self._latest = None
        # Default QoS (RELIABLE, VOLATILE, depth 10) matches franka_bringup's
        # /joint_states publisher in v0.1.15. Some drivers publish under
        # SENSOR_DATA (BEST_EFFORT) — switch QoS here if no callbacks fire.
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

    def _on_joint_state(self, msg):
        if all(j in msg.name for j in FR3_JOINT_NAMES):
            self._latest = msg

    def latest_positions(self):
        """Return (positions, age_seconds) or (None, None) if not received."""
        if self._latest is None:
            return None, None
        name_to_pos = dict(zip(self._latest.name, self._latest.position))
        positions = [float(name_to_pos[j]) for j in FR3_JOINT_NAMES]
        now = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(self._latest.header.stamp)
        age = (now - msg_time).nanoseconds / 1e9
        return positions, age


def _read_header_comment(path):
    """Return the leading comment block (lines starting with #) from an
    existing YAML file, or None if the file doesn't exist or has no header."""
    if not path.exists():
        return None
    header_lines = []
    with open(path) as f:
        for line in f:
            if line.startswith('#'):
                header_lines.append(line)
            else:
                break
    return ''.join(header_lines) if header_lines else None


def _load_existing_poses(path, log):
    if not path.exists():
        return []
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        log.error(f'Existing poses file at {path} is malformed: {e}')
        log.error('Fix or remove it before re-running.')
        sys.exit(2)
    if not isinstance(data, dict):
        log.error(f'Existing poses file at {path} must be a YAML mapping, '
                  f'got {type(data).__name__}.')
        sys.exit(2)
    return list(data.get('poses', []))


def _save_poses(path, poses, header):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w') as f:
        if header:
            f.write(header)
        yaml.safe_dump(
            {'joint_names': FR3_JOINT_NAMES, 'poses': poses},
            f, default_flow_style=False, sort_keys=False,
        )


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Record FR3 joint configurations to a calibration poses YAML'
    )
    # Default path resolves to fr3_calibration/config/ in the source tree
    # when the package is built with --symlink-install (which is documented
    # in the README). Pass --out explicitly if you build without symlinks.
    default_out = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        'config', 'calibration_poses.yaml',
    )
    parser.add_argument('--out', default=default_out,
                        help=f'Output YAML path (default: {default_out})')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--append', action='store_true',
                      help='Append to existing poses file')
    mode.add_argument('--overwrite', action='store_true',
                      help='Replace existing poses file')
    args = parser.parse_args(argv)

    out_path = Path(os.path.expanduser(args.out))

    rclpy.init()
    node = JointStateLatch()
    log = node.get_logger()

    if out_path.exists() and not (args.append or args.overwrite):
        log.error(f'Poses file already exists: {out_path}')
        log.error('Pass --append to add to it, or --overwrite to replace it.')
        node.destroy_node()
        rclpy.shutdown()
        return 2

    if args.append and out_path.exists():
        poses = _load_existing_poses(out_path, log)
        header = _read_header_comment(out_path)
    else:
        poses = []
        header = None
    if header is None:
        stamp = datetime.now().astimezone().strftime('%Y-%m-%d %H:%M:%S %Z')
        header = f'# fr3_calibration poses — recorded {stamp}\n'
    next_index = len(poses) + 1

    log.info(f'Recording calibration poses to: {out_path}')
    log.info('Drive the arm to a pose, then press Enter to capture. '
             'Type q + Enter to quit.')

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            user = input(f'[pose_{next_index:02d}] Enter to capture, q to quit: ').strip().lower()
            if user == 'q':
                break

            rclpy.spin_once(node, timeout_sec=0.05)
            positions, age = node.latest_positions()
            if positions is None:
                log.warn('No /joint_states with all FR3 joints received yet — '
                         'is franka_bringup running? Skipping.')
                continue
            if age > STALE_THRESHOLD_S:
                log.warn(f'Latest /joint_states is {age:.2f} s old '
                         f'(> {STALE_THRESHOLD_S} s) — publisher may have '
                         'stopped. Skipping.')
                continue

            poses.append({
                'name': f'pose_{next_index:02d}',
                'positions': positions,
            })
            _save_poses(out_path, poses, header)
            log.info(f'Captured. Total poses: {len(poses)}')
            next_index += 1

    except KeyboardInterrupt:
        pass
    finally:
        log.info(f'Saved {len(poses)} pose(s) to {out_path}')
        if not poses:
            log.error('No poses captured.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if poses else 3


if __name__ == '__main__':
    sys.exit(main())

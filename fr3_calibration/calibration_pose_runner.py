# calibration_pose_runner.py: Drive the FR3 through a list of recorded poses.
# Operator triggers easy_handeye2 sample/compute/save via the GUI between
# motions; this script handles only motion.

import argparse
import os
import signal
import sys
import threading
import time
from pathlib import Path

import rclpy
import yaml
from pymoveit2 import MoveIt2, MoveIt2State
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


GROUP_NAME = 'fr3_arm'
JOINT_NAMES = ['fr3_joint1', 'fr3_joint2', 'fr3_joint3',
               'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7']
BASE_LINK = 'fr3_link0'
EE_LINK = 'fr3_link8'

MOTION_TIMEOUT_S = 60.0
POLL_DT_S = 0.05


def prompt_yn(message, default_yes=True):
    """Linux-style Y/n prompt. Enter alone returns the default. Loops on
    invalid input. Stays on stdin/stdout — must NOT go through ROS logging
    because operator interaction has to interleave with stdin."""
    suffix = '(Y/n)' if default_yes else '(y/N)'
    while True:
        resp = input(f'{message} {suffix}: ').strip().lower()
        if not resp:
            return default_yes
        if resp in ('y', 'yes'):
            return True
        if resp in ('n', 'no'):
            return False
        print('Please answer y or n.')


def _load_poses(path, log):
    if not path.exists():
        log.error(f'Poses file not found: {path}')
        log.error('Record poses first with: '
                  'ros2 run fr3_calibration calibration_pose_recorder')
        sys.exit(2)
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        log.error(f'Poses file at {path} is malformed: {e}')
        sys.exit(2)
    if not isinstance(data, dict):
        log.error(f'Poses file at {path} must be a YAML mapping, '
                  f'got {type(data).__name__}.')
        sys.exit(2)
    poses = data.get('poses') or []
    if not poses:
        log.error(f'No poses in {path}.')
        sys.exit(2)
    return poses


def _drive_to(moveit, positions, stop_event, log):
    """Plan + execute via MoveIt2 /move_action. Polls query_state in the
    main thread; the parent executor spins the node in a background thread.

    Returns True only on a SUCCEEDED action result; False on planning
    failure, execution abort, timeout, no-goal-dispatched, or
    operator-requested cancel.

    Always drains to IDLE before returning so a late result callback
    cannot flip motion_suceeded after we've moved on."""
    moveit.motion_suceeded = False
    moveit.move_to_configuration(positions)

    saw_non_idle = False
    deadline = time.monotonic() + MOTION_TIMEOUT_S
    aborting = False

    while True:
        state = moveit.query_state()
        if state != MoveIt2State.IDLE:
            saw_non_idle = True

        if not aborting and (stop_event.is_set() or time.monotonic() >= deadline):
            if time.monotonic() >= deadline and not stop_event.is_set():
                log.warn(f'Motion exceeded {MOTION_TIMEOUT_S} s — cancelling.')
            aborting = True

        if aborting and state == MoveIt2State.EXECUTING:
            try:
                moveit.cancel_execution()
            except Exception as e:
                log.warn(f'cancel_execution raised: {e}')

        if state == MoveIt2State.IDLE:
            if not saw_non_idle:
                return False
            if aborting:
                return False
            return bool(moveit.motion_suceeded)

        time.sleep(POLL_DT_S)


def run(args):
    rclpy.init()
    node = Node('calibration_pose_runner')
    log = node.get_logger()
    cb = ReentrantCallbackGroup()

    moveit = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=EE_LINK,
        group_name=GROUP_NAME,
        callback_group=cb,
        # /move_action path; default plan()+execute() spins the node and
        # would race with the executor we start below.
        use_move_group_action=True,
    )
    moveit.max_velocity = args.vel_scale
    moveit.max_acceleration = args.vel_scale

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    stop_event = threading.Event()

    def _on_sigint(signum, frame):
        # Signal handler only flips a flag; cancellation is performed in
        # _drive_to under normal control flow.
        if not stop_event.is_set():
            print('\nCtrl-C: cancelling current motion and shutting down...',
                  file=sys.stderr)
            stop_event.set()

    signal.signal(signal.SIGINT, _on_sigint)

    try:
        poses = _load_poses(Path(os.path.expanduser(args.poses)), log)
        log.info(f'Loaded {len(poses)} pose(s) from {args.poses}')
        log.info(f'Velocity scale: {args.vel_scale}')

        if not prompt_yn('EAD ready in your hand?', default_yes=False):
            log.info('Aborted by operator.')
            return 1

        for i, pose in enumerate(poses, start=1):
            if stop_event.is_set():
                log.info('Aborting run (stop requested).')
                return 130
            name = pose['name']
            positions = pose['positions']
            prefix = f'[{i}/{len(poses)}] {name}'

            if not prompt_yn(f'{prefix}: Move to this pose?', default_yes=True):
                log.info(f'{prefix}: skipped (no motion).')
                continue

            if not _drive_to(moveit, positions, stop_event, log):
                log.error(f'{prefix}: plan/execute failed.')
                if stop_event.is_set():
                    return 130
                if not prompt_yn('  Skip and continue?', default_yes=True):
                    log.info('Aborting run.')
                    return 4
                continue

            # Operator clicks "Take Sample" in the easy_handeye2 GUI here.
            input(f'  Pose reached. Click "Take Sample" in the GUI, '
                  'then press Enter to continue: ')

        log.info('All poses processed. In the easy_handeye2 GUI: '
                 'click Compute, then Save.')
        return 0
    finally:
        executor.shutdown()
        node.destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Drive FR3 through recorded calibration poses. Operator '
                    'triggers Take Sample / Compute / Save in the easy_handeye2 GUI.'
    )
    default_poses = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        'config', 'calibration_poses.yaml',
    )
    parser.add_argument('--poses', default=default_poses,
                        help=f'Poses YAML path (default: {default_poses})')
    parser.add_argument('--vel-scale', type=float, default=0.1,
                        help='MoveIt2 max velocity scaling [0..1] (default: 0.1)')
    args = parser.parse_args(argv)

    try:
        return run(args)
    except KeyboardInterrupt:
        return 130
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

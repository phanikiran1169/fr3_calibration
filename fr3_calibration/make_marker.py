# make_marker.py: Copy a prebuilt ArUco marker JPG from aruco_ros into the
# package data directory for printing.

import argparse
import os
import shutil
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


_ARUCO_ROS_PREBUILT = {
    (26, 0.050): 'marker26_5cm_margin_2cm.jpg',
    (582, 0.050): 'marker582_5cm_margin_2cm.jpg',
}


def _load_marker_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Stage a printable ArUco marker for FR3 calibration'
    )
    default_config = os.path.join(
        get_package_share_directory('fr3_calibration'), 'config', 'marker.yaml'
    )
    default_out = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        'data', 'marker.jpg',
    )
    parser.add_argument('--config', default=default_config,
                        help=f'Marker config YAML (default: {default_config})')
    parser.add_argument('--out', default=default_out,
                        help=f'Output path (default: {default_out})')
    args = parser.parse_args(argv)

    cfg = _load_marker_config(Path(args.config))['aruco']
    marker_id = int(cfg['marker_id'])
    marker_size_m = float(cfg['marker_size_m'])

    key = (marker_id, marker_size_m)
    if key not in _ARUCO_ROS_PREBUILT:
        available = ', '.join(f'(id={mid}, size={sz}m)'
                              for mid, sz in _ARUCO_ROS_PREBUILT)
        print(f'No prebuilt marker for id={marker_id}, size={marker_size_m}m.',
              file=sys.stderr)
        print(f'Available: {available}', file=sys.stderr)
        return 2

    src_dir = Path(get_package_share_directory('aruco_ros')) / 'etc'
    src = src_dir / _ARUCO_ROS_PREBUILT[key]
    if not src.exists():
        print(f'Source not found: {src}', file=sys.stderr)
        return 3

    dest = Path(os.path.expanduser(args.out))
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)

    print(f'Copied {src} -> {dest}')
    print(f'  marker_id:   {marker_id}')
    print(f'  marker size: {marker_size_m * 1000:.0f} mm')
    print('Print at 100% scale. Measure the printed side with calipers and '
          f'update marker_size_m in {args.config} if it deviates.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

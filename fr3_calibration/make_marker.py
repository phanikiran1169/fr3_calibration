# make_marker.py: Copy a printable ArUco marker from aruco_ros/etc/ into the
# thesis calibration data directory, with a one-line summary of the size and ID.
#
# fr3_calibration: aruco_ros ships pre-rendered marker JPGs at fixed sizes
# (markers 26 and 582 at 5 cm). This script picks the right file based on
# config/marker.yaml and stages it for printing.
#
# After printing, measure the marker side with calipers and update marker.yaml
# (marker_size_m) with the actual value.

import argparse
import os
import shutil
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


# aruco_ros ships these prebuilt 5 cm marker JPGs in the etc/ directory.
# Each entry maps (marker_id, marker_size_m) -> filename inside aruco_ros/etc/
_ARUCO_ROS_PREBUILT = {
    (26, 0.050): 'marker26_5cm_margin_2cm.jpg',
    (582, 0.050): 'marker582_5cm_margin_2cm.jpg',
}


def _load_marker_config(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Stage a printable ArUco marker for FR3 calibration'
    )
    default_config = os.path.join(
        get_package_share_directory('fr3_calibration'), 'config', 'marker.yaml'
    )
    parser.add_argument('--config', default=default_config,
                        help=f'Marker config YAML (default: {default_config})')
    parser.add_argument('--out', default='~/thesis/data/calib/marker.jpg',
                        help='Output path (default: ~/thesis/data/calib/marker.jpg)')
    args = parser.parse_args(argv)

    cfg = _load_marker_config(Path(args.config))['aruco']
    marker_id = int(cfg['marker_id'])
    marker_size_m = float(cfg['marker_size_m'])

    key = (marker_id, marker_size_m)
    if key not in _ARUCO_ROS_PREBUILT:
        available = ', '.join(f'(id={mid}, size={sz}m)'
                              for mid, sz in _ARUCO_ROS_PREBUILT)
        print(f'No prebuilt aruco_ros marker for id={marker_id}, size={marker_size_m}m.',
              file=sys.stderr)
        print(f'Available prebuilt markers: {available}', file=sys.stderr)
        print('Edit config/marker.yaml to use one of these, or generate a custom '
              'marker with cv2.aruco and update this script.', file=sys.stderr)
        return 2

    src_dir = Path(get_package_share_directory('aruco_ros')) / 'etc'
    src = src_dir / _ARUCO_ROS_PREBUILT[key]
    if not src.exists():
        print(f'aruco_ros prebuilt marker not found at {src}', file=sys.stderr)
        return 3

    dest = Path(os.path.expanduser(args.out))
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)

    print(f'Copied {src} -> {dest}')
    print(f'  marker_id:   {marker_id}')
    print(f'  marker size: {marker_size_m * 1000:.0f} mm')
    print('Print at 100% scale (no "fit to page"). After printing, measure the')
    print(f'marker side with calipers and update {args.config} if it deviates.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

# make_marker.py
# Stages a printable ArUco marker. Copies the prebuilt JPG from aruco_ros
# and renders Letter and A4 PDFs with the marker centered at the configured
# physical size, so printing at "Actual Size" yields a marker matching
# marker_size_m.

import argparse
import os
import shutil
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from reportlab.lib.pagesizes import LETTER, A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


_ARUCO_ROS_PREBUILT = {
    (26, 0.050): 'marker26_5cm_margin_2cm.jpg',
    (582, 0.050): 'marker582_5cm_margin_2cm.jpg',
}

# The prebuilt JPGs ship a marker with a fixed white margin around it. Total
# image side = marker_side + 2 * margin. The "_2cm" variant has a 2 cm
# (20 mm) margin on each side, so total = marker_size_m + 0.040.
_PREBUILT_MARGIN_M = 0.020


def _load_marker_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _render_pdf(jpg_path, pdf_path, page_size, image_side_m):
    """Place the marker JPG centered on a single page at the given physical
    side length."""
    page_w, page_h = page_size
    side_mm = image_side_m * 1000.0
    side_pt = side_mm * mm  # reportlab unit conversion
    x = (page_w - side_pt) / 2.0
    y = (page_h - side_pt) / 2.0

    c = canvas.Canvas(str(pdf_path), pagesize=page_size)
    c.drawImage(str(jpg_path), x, y, width=side_pt, height=side_pt,
                preserveAspectRatio=True, mask='auto')
    c.setFont('Helvetica', 9)
    label = (f'ArUco MIP_36h12  marker side: {side_mm - 2 * _PREBUILT_MARGIN_M * 1000:.1f} mm  '
             f'image side: {side_mm:.1f} mm  Print at 100% / Actual Size')
    c.drawCentredString(page_w / 2.0, 12 * mm, label)
    c.showPage()
    c.save()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Stage a printable ArUco marker for FR3 calibration'
    )
    default_config = os.path.join(
        get_package_share_directory('fr3_calibration'), 'config', 'marker.yaml'
    )
    default_out_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        'data',
    )
    parser.add_argument('--config', default=default_config,
                        help=f'Marker config YAML (default: {default_config})')
    parser.add_argument('--out-dir', default=default_out_dir,
                        help=f'Output directory (default: {default_out_dir})')
    args = parser.parse_args(argv)

    cfg = _load_marker_config(Path(args.config))['aruco']
    marker_id = int(cfg['marker_id'])
    marker_size_m = float(cfg['marker_size_m'])

    # The prebuilt JPGs are keyed on the marker's nominal size (5 cm). The
    # measured marker_size_m may differ slightly after printing, but for
    # selecting the source asset we use the nominal (rounded) value.
    nominal_size = round(marker_size_m, 2)
    key = (marker_id, nominal_size)
    if key not in _ARUCO_ROS_PREBUILT:
        available = ', '.join(f'(id={mid}, size={sz}m)'
                              for mid, sz in _ARUCO_ROS_PREBUILT)
        print(f'No prebuilt marker for id={marker_id}, '
              f'nominal size={nominal_size}m.', file=sys.stderr)
        print(f'Available: {available}', file=sys.stderr)
        return 2

    src_dir = Path(get_package_share_directory('aruco_ros')) / 'etc'
    src = src_dir / _ARUCO_ROS_PREBUILT[key]
    if not src.exists():
        print(f'Source not found: {src}', file=sys.stderr)
        return 3

    out_dir = Path(os.path.expanduser(args.out_dir))
    out_dir.mkdir(parents=True, exist_ok=True)

    jpg_dest = out_dir / 'marker.jpg'
    shutil.copy2(src, jpg_dest)

    image_side_m = nominal_size + 2 * _PREBUILT_MARGIN_M
    letter_pdf = out_dir / 'marker_letter.pdf'
    a4_pdf = out_dir / 'marker_a4.pdf'
    _render_pdf(jpg_dest, letter_pdf, LETTER, image_side_m)
    _render_pdf(jpg_dest, a4_pdf, A4, image_side_m)

    print(f'Wrote:')
    print(f'  {jpg_dest}')
    print(f'  {letter_pdf}')
    print(f'  {a4_pdf}')
    print(f'  marker_id:    {marker_id}')
    print(f'  marker side:  {nominal_size * 1000:.0f} mm (nominal)')
    print(f'  image side:   {image_side_m * 1000:.0f} mm (marker + 2 cm white margin each side)')
    print('Print at 100% / Actual Size (no fit-to-page). Measure the printed '
          f'marker side with calipers and update marker_size_m in {args.config} '
          'if it deviates.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

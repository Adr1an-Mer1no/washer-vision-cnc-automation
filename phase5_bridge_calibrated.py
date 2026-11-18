"""WasherVision Phase 5: calibrated, read-only alignment planner.

No motion commands are sent. Use a newly saved YOLO centroid and a fresh Arduino
STATUS for each preview. Carriage X moves right with positive X; platform Y
moves camera-Y positive with positive Y. Calibration remains provisional.
"""
import argparse
import csv
import json
import math
import re
import time
from datetime import datetime, timezone
from pathlib import Path

DEFAULT_MAPPING = Path(__file__).with_name('camera_machine_mapping_calibrated.json')


def parse_status(line):
    if not line.startswith('STATUS '):
        return None
    return dict(re.findall(r'([A-Za-z][A-Za-z0-9]*)=([^ ]+)', line))


def connect(port, baud=9600):
    import serial
    conn = serial.Serial(port, baudrate=baud, timeout=0.3, write_timeout=1)
    time.sleep(2.5)  # Arduino Uno commonly resets on serial open
    conn.reset_input_buffer()
    return conn


def ask(conn, command, seconds=2):
    if command not in ('PING', 'STATUS'):
        raise ValueError('Only PING and STATUS are permitted')
    conn.write((command + '\n').encode('ascii'))
    deadline = time.monotonic() + seconds
    lines = []
    while time.monotonic() < deadline:
        raw = conn.readline()
        if raw:
            line = raw.decode('utf-8', errors='replace').strip()
            if line:
                lines.append(line)
                if (command == 'PING' and line.startswith('PONG ')) or (command == 'STATUS' and line.startswith('STATUS ')):
                    break
    return lines


def latest_detections(csv_path):
    with csv_path.open(newline='', encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError('No saved detections; press S in vision window first')
    stamp = rows[-1]['timestamp_utc']
    return [r for r in rows if r['timestamp_utc'] == stamp] or [rows[-1]]


def round_steps(mm, steps_per_mm):
    # Match Arduino lroundf, including half-away-from-zero.
    value = mm * steps_per_mm
    return math.floor(value + 0.5) if value >= 0 else math.ceil(value - 0.5)


def check_status(status):
    if status is None:
        return ['No valid STATUS received']
    checks = {'mode': 'AUTO', 'axis': 'XY', 'stop': '0', 'moving': '0',
              'homing': '0', 'homeX': '1', 'homeY': '1',
              'X': 'CLOSED', 'Y': 'CLOSED'}
    return [f'{key} must be {expected} (received {status.get(key, "missing")})'
            for key, expected in checks.items() if status.get(key) != expected]


def plan(row, status, cfg):
    errors = check_status(status)
    if errors:
        raise ValueError('STATUS not safe for preview: ' + '; '.join(errors))
    x = float(row['workspace_x_mm'])
    y = float(row['workspace_y_mm'])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError('Invalid camera coordinates')
    sx = int(status['posXsteps'])
    sy = int(status['posYsteps'])
    spx, spy = cfg['steps_per_mm']
    anchor_mx, anchor_my = cfg['anchor_machine_mm']
    anchor_cx, anchor_cy = cfg['anchor_camera_mm']
    kx, ky = cfg['camera_mm_per_commanded_mm']
    if min(spx, spy, kx, ky) <= 0:
        raise ValueError('Calibration scale must be positive')
    mx, my = sx / spx, sy / spy
    # X carriage changes its camera-frame pickup location; camera washer X stays fixed.
    pickup_x = anchor_cx + kx * (mx - anchor_mx)
    # Y carriage pickup point is fixed in the camera frame; platform moves washer Y.
    pickup_y = anchor_cy
    delta_x = (x - pickup_x) / kx
    delta_y = (pickup_y - y) / ky
    # Quantize to firmware's integer steps. Use RELATIVE movement from current STATUS.
    dx_steps = round_steps(delta_x, spx)
    dy_steps = round_steps(delta_y, spy)
    tx_steps, ty_steps = sx + dx_steps, sy + dy_steps
    max_x, max_y = cfg['max_steps']
    max_cmd = cfg['max_command_steps']
    warnings = []
    if not 0 <= tx_steps <= max_x or not 0 <= ty_steps <= max_y:
        warnings.append('BLOCKED: proposed target exceeds firmware soft limits')
    if abs(dx_steps) > max_cmd or abs(dy_steps) > max_cmd:
        warnings.append('BLOCKED: proposed correction exceeds 10 mm per axis per command; do not issue as one MOVE')
    if not cfg.get('verified', False):
        warnings.append('Calibration is PROVISIONAL: physical ±1 mm accuracy is not instrument-verified')
    warnings.append('This is not an authorization to move; inspect carriage clearance, limits and washer stability')
    return {'camera': (x, y), 'machine_now_mm': (mx, my),
            'pickup_camera_now': (pickup_x, pickup_y),
            'error_camera_mm': (x-pickup_x, y-pickup_y),
            'delta_mm_unrounded': (delta_x, delta_y),
            'delta_steps': (dx_steps, dy_steps),
            'target_steps': (tx_steps, ty_steps),
            'target_mm': (tx_steps/spx, ty_steps/spy), 'warnings': warnings}


def load_status(args):
    if args.status_line:
        return parse_status(args.status_line)
    if args.status_file:
        lines = Path(args.status_file).read_text().splitlines()
        matches = [parse_status(line.strip()) for line in lines]
        matches = [s for s in matches if s]
        return matches[-1] if matches else None
    if args.serial_check or args.preview:
        with connect(args.port, args.baud) as conn:
            if args.serial_check:
                for command in ('PING', 'STATUS'):
                    print('>', command)
                    for line in ask(conn, command):
                        print('<', line)
            if args.preview:
                responses = ask(conn, 'STATUS')
                return next((parse_status(s) for s in responses if parse_status(s)), None)
    return None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='/dev/cu.usbmodem1201')
    p.add_argument('--baud', type=int, default=9600)
    p.add_argument('--csv', default='washer_coordinates_fusion.csv')
    p.add_argument('--mapping', default=str(DEFAULT_MAPPING))
    p.add_argument('--serial-check', action='store_true', help='Send read-only PING and STATUS')
    p.add_argument('--preview', action='store_true', help='Preview latest saved detections')
    p.add_argument('--status-line', help='Paste a complete recent STATUS line instead of connecting to Arduino')
    p.add_argument('--status-file', help='Read most recent STATUS from text file instead of serial')
    args = p.parse_args()
    if not args.serial_check and not args.preview:
        p.error('Specify --serial-check and/or --preview')
    if args.status_line and args.status_file:
        p.error('Use only one of --status-line and --status-file')
    status = load_status(args)
    if args.preview:
        cfg = json.loads(Path(args.mapping).read_text(encoding='utf-8'))
        print('CALIBRATED PREVIEW ONLY — NO MOTOR COMMANDS WILL BE SENT')
        for row in latest_detections(Path(args.csv)):
            if not row.get('workspace_x_mm') or not row.get('workspace_y_mm'):
                continue
            try:
                result = plan(row, status, cfg)
            except (ValueError, KeyError, TypeError) as exc:
                print('BLOCKED:', exc)
                continue
            print(f"Track {row.get('track_id', '?')} {row.get('voted_class') or row.get('class_name', '?')}")
            print(f"  Camera washer: ({result['camera'][0]:.2f}, {result['camera'][1]:.2f}) mm")
            print(f"  Current machine: ({result['machine_now_mm'][0]:.2f}, {result['machine_now_mm'][1]:.2f}) mm")
            print(f"  Predicted carriage pickup in camera: ({result['pickup_camera_now'][0]:.2f}, {result['pickup_camera_now'][1]:.2f}) mm")
            print(f"  Camera error (washer minus pickup): ({result['error_camera_mm'][0]:+.2f}, {result['error_camera_mm'][1]:+.2f}) mm")
            print(f"  Proposed relative correction: ({result['delta_mm_unrounded'][0]:+.2f}, {result['delta_mm_unrounded'][1]:+.2f}) mm")
            print(f"  Quantized relative steps: X={result['delta_steps'][0]:+d}, Y={result['delta_steps'][1]:+d}")
            print(f"  Proposed target steps: X={result['target_steps'][0]}, Y={result['target_steps'][1]}")
            for warning in result['warnings']:
                print('  WARNING:', warning)
    print('DRY RUN ONLY: only PING/STATUS may be sent; no HOME, MOVE, GOTO, RESETSTOP or other motor commands.')


if __name__ == '__main__':
    main()

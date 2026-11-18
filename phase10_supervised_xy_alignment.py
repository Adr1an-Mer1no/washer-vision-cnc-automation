"""WasherVision Phase 10: one-at-a-time supervised XY vision correction + Phase 9 planner.

PREVIEW reads latest saved YOLO centroid and queries STATUS on the SAME serial
connection. PREVIEW and PLAN XY NEVER send motor commands. ALIGN STEP may send ONE bounded
XY command only after explicit operator approval. MOVE commands remain manually
entered and individually approved by the existing supervised controller.

Requires phase5_bridge_calibrated.py and phase5_supervised_controller.py in the
same folder, plus camera_machine_mapping_calibrated.json and YOLO CSV.
"""
import argparse
import csv
import json
import math
import re
import time
from datetime import datetime, timezone
from pathlib import Path

from phase5_bridge_calibrated import plan
from phase5_supervised_controller import query_status, safe_status, send_move, read_until

ROOT = Path(__file__).resolve().parent
LAST_MOVED_FRAME = None  # session-local timestamp of the frame used for last ALIGN STEP


def newest_frame(path, max_age):
    with path.open(newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError('CSV has no detections; save a new YOLO frame with S')
    last = rows[-1]
    stamp = last.get('timestamp_utc', '')
    if not stamp:
        raise ValueError('Missing detection timestamp')
    ts = datetime.fromisoformat(stamp.replace('Z', '+00:00'))
    if ts.tzinfo is None:
        raise ValueError('Detection timestamp has no timezone')
    age = (datetime.now(timezone.utc) - ts.astimezone(timezone.utc)).total_seconds()
    if not 0 <= age <= max_age:
        raise ValueError(f'Detection is {age:.1f}s old (allowed 0–{max_age:g}s); press S again')
    frame = [r for r in rows if r.get('timestamp_utc') == stamp]
    if len(frame) != 1:
        raise ValueError(f'Latest frame has {len(frame)} detections; isolate ONE washer')
    row = frame[0]
    if not row.get('workspace_x_mm') or not row.get('workspace_y_mm'):
        raise ValueError('Missing calibrated workspace coordinates')
    if not math.isfinite(float(row['workspace_x_mm'])) or not math.isfinite(float(row['workspace_y_mm'])):
        raise ValueError('Nonfinite camera coordinates')
    if float(row.get('confidence') or 0) < 0.65:
        raise ValueError('Low detection confidence (<0.65); inspect the washer')
    if 'OVERLAP' in (row.get('status') or '').upper():
        raise ValueError('Detection flagged overlap')
    return row, age


def preview(status, args, cfg):
    if not safe_status(status):
        raise ValueError('Arduino is not homed, stationary, and safe in AUTO mode')
    row, age = newest_frame(args.csv, args.max_age)
    result = plan(row, status, cfg)
    print('\nREAD-ONLY INTEGRATED PREVIEW — NO MOTOR COMMANDS')
    print(f"Washer track {row.get('track_id','?')} / {row.get('voted_class') or row.get('class_name','?')} | saved {age:.1f}s ago")
    print(f"Camera washer: ({result['camera'][0]:.2f}, {result['camera'][1]:.2f}) mm")
    print(f"Machine now: ({result['machine_now_mm'][0]:.2f}, {result['machine_now_mm'][1]:.2f}) mm")
    print(f"Predicted pickup: ({result['pickup_camera_now'][0]:.2f}, {result['pickup_camera_now'][1]:.2f}) camera mm")
    print(f"Camera error: ({result['error_camera_mm'][0]:+.2f}, {result['error_camera_mm'][1]:+.2f}) mm")
    print(f"Proposed correction: X={result['delta_steps'][0]:+d}, Y={result['delta_steps'][1]:+d} steps")
    print(f"Proposed target: X={result['target_steps'][0]}, Y={result['target_steps'][1]} steps")
    for warning in result['warnings']:
        print('WARNING:', warning)
    print('No movement sent. PREVIEW is NOT authorization to move.\n')




def split_steps(total, max_per_command):
    """Split integer-step corrections into individually bounded signed segments."""
    if not isinstance(total, int) or isinstance(total, bool):
        raise ValueError('Step correction must be an integer')
    if not isinstance(max_per_command, int) or max_per_command <= 0:
        raise ValueError('Invalid per-command step limit')
    sign = 1 if total >= 0 else -1
    remaining = abs(total)
    parts = []
    while remaining:
        n = min(remaining, max_per_command)
        parts.append(sign * n)
        remaining -= n
    return parts


def plan_xy(status, args, cfg):
    """Print bounded hypothetical commands. NEVER write to the serial port."""
    if not safe_status(status):
        raise ValueError('Requires AUTO, homed X/Y, stop=0, stationary, closed X/Y switches')
    row, age = newest_frame(args.csv, args.max_age)
    result = plan(row, status, cfg)
    sx, sy = int(status['posXsteps']), int(status['posYsteps'])
    tx, ty = map(int, result['target_steps'])
    max_x, max_y = cfg['max_steps']
    max_cmd = cfg['max_command_steps']
    if not isinstance(max_cmd, int) or max_cmd != 50:
        raise ValueError('Unexpected maximum command steps')
    if not (0 <= sx <= max_x and 0 <= sy <= max_y):
        raise ValueError('Current position outside expected limits')
    if not (0 <= tx <= max_x and 0 <= ty <= max_y):
        raise ValueError('Proposed target exceeds X/Y soft limits')
    if any('BLOCKED: proposed target' in warning for warning in result['warnings']):
        raise ValueError('Bridge reports an out-of-limits target')
    dx, dy = map(int, result['delta_steps'])
    if (tx, ty) != (sx + dx, sy + dy):
        raise ValueError('Planner position inconsistency')
    commands = []
    current = {'X': sx, 'Y': sy}
    for axis, delta, maximum in [('X', dx, max_x), ('Y', dy, max_y)]:
        for part in split_steps(delta, max_cmd):
            target = current[axis] + part
            if not 0 <= target <= maximum:
                raise ValueError('Intermediate target outside travel limits')
            mm = part / 5.0
            commands.append((axis, part, mm, current[axis], target))
            current[axis] = target
    if (current['X'], current['Y']) != (tx, ty):
        raise ValueError('Planner final target inconsistency')
    print('\nPHASE 9 READ-ONLY XY PLAN — NO MOTOR COMMANDS')
    print(f"Saved washer track {row.get('track_id','?')} | age {age:.1f}s | confidence {float(row['confidence']):.2f}")
    print(f"Camera washer: ({result['camera'][0]:.2f}, {result['camera'][1]:.2f}) mm")
    print(f"Camera residual: X={result['error_camera_mm'][0]:+.2f}, Y={result['error_camera_mm'][1]:+.2f} mm")
    print(f'Current steps: X={sx}, Y={sy}; hypothetical target: X={tx}, Y={ty}')
    if not commands:
        print('No correction required at current step resolution.')
    else:
        print(f'Hypothetical sequence ({len(commands)} segments; each <=50 steps):')
        for i, (axis, steps, mm, before, after) in enumerate(commands, 1):
            print(f'  {i:02d}. MOVE {axis}{mm:+g}  | {steps:+d} steps | {axis}: {before} -> {after}')
    for warning in result['warnings']:
        if 'exceeds 10 mm per axis per command' not in warning:
            print('NOTE:', warning)
    print('NOT EXECUTABLE: planned steps are a snapshot, not approvals or a live path.')
    print('Reacquire a fresh YOLO frame and STATUS after ANY physical movement.')
    print('Z remains manual-only. No movement was sent.\n')
    return commands



def align_step(conn, args, cfg):
    """Replan from live state; approve and execute at most ONE <=10-step XY move."""
    global LAST_MOVED_FRAME
    before = query_status(conn)
    if not safe_status(before):
        raise ValueError('Requires AUTO, homed X/Y, stop=0, stationary, closed X/Y switches')
    row, age = newest_frame(args.csv, args.max_age)
    if row.get('timestamp_utc') == LAST_MOVED_FRAME:
        raise ValueError('Capture a NEW YOLO frame with S after the previous alignment move')
    result = plan(row, before, cfg)
    if any('BLOCKED:' in w for w in result['warnings']):
        raise ValueError('Bridge warning blocks alignment: ' + '; '.join(result['warnings']))
    xerr, yerr = map(float, result['error_camera_mm'])
    if not all(math.isfinite(v) for v in (xerr, yerr)):
        raise ValueError('Invalid camera residual')
    print(f'\nPHASE 10 SINGLE-STEP ALIGNMENT | frame age {age:.1f}s | confidence {float(row["confidence"]):.2f}')
    print(f'Camera residual X={xerr:+.2f} mm, Y={yerr:+.2f} mm')
    deadband = 1.0  # Provisional; NOT verified physical pickup tolerance.
    if abs(xerr) <= deadband and abs(yerr) <= deadband:
        print('HOLD: residual within provisional 1.0-mm camera deadband. No movement.')
        print('This does not establish physical pickup accuracy.\n')
        return
    sx, sy = int(before['posXsteps']), int(before['posYsteps'])
    if not (0 <= sx <= 950 and 0 <= sy <= 1155):
        raise ValueError('Current reported position outside configured limits')
    dx, dy = map(int, result['delta_steps'])
    tx, ty = map(int, result['target_steps'])
    if (tx, ty) != (sx + dx, sy + dy):
        raise ValueError('Inconsistent bridge target')
    if not (0 <= tx <= 950 and 0 <= ty <= 1155):
        raise ValueError('Proposed full target outside limits')
    # Select the largest camera-coordinate error first, and never move an axis
    # already inside the deadband. No fallback to another axis without replanning.
    axes = sorted([('X', xerr, dx, sx, 950), ('Y', yerr, dy, sy, 1155)],
                  key=lambda v: abs(v[1]), reverse=True)
    axis, error, correction, pos, limit = next((a for a in axes if abs(a[1]) > deadband), (None,0,0,0,0))
    if axis is None:
        print('HOLD: no axis outside deadband')
        return
    if correction == 0:
        print('HOLD: computed correction rounds to zero steps; inspect mapping')
        return
    part = max(-10, min(10, correction))
    target = pos + part
    if not 0 <= target <= limit:
        raise ValueError('Single-step target outside soft limits')
    mm = part / 5.0
    print(f'PROPOSED ONE MOVE: {axis} {pos} -> {target} steps ({part:+d} steps / {mm:+.1f} mm)')
    print('Gripper detached; Z elevated; path clear; E-stop accessible. Check physical direction.')
    print('No batch movement. A NEW YOLO S capture is mandatory after this step.')
    if input('Type YES to execute exactly this ONE move: ').strip() != 'YES':
        print('Cancelled; no movement sent')
        return
    # Recheck both axes and all safety state immediately before issuing command.
    again = query_status(conn)
    if not safe_status(again) or any(again.get(k) != before.get(k) for k in
        ('posXsteps','posYsteps','homeX','homeY','mode','axis','stop')):
        raise ValueError('Arduino status changed after planning; no movement sent')
    # Freshness recheck and identity guard; never execute a stale/replaced frame.
    latest, latest_age = newest_frame(args.csv, args.max_age)
    if latest != row or latest_age > args.max_age:
        raise ValueError('YOLO detection changed or expired; no movement sent')
    LAST_MOVED_FRAME = row.get('timestamp_utc')  # Never retry same observation after a write attempt.
    conn.write((f'MOVE {axis}{mm:g}\n').encode('ascii'))
    lines = read_until(conn, 8.0, 'OK MOVE DONE')
    if not any(line.startswith('OK MOVE DONE') for line in lines):
        print('WARNING: missing MOVE DONE; use physical emergency stop if motion persists.')
        print('Do not retry automatically. Inspect hardware and STATUS.')
        return
    after = query_status(conn)
    if (not safe_status(after) or int(after['pos'+axis+'steps']) != target or
        int(after['pos'+('Y' if axis=='X' else 'X')+'steps']) != (sy if axis=='X' else sx)):
        print('WARNING: unexpected post-move state; STOP and investigate. No further movement.')
        return
    print('VERIFIED Arduino reported step position; physical alignment NOT yet verified.')
    print('Capture NEW YOLO frame with S, then PLAN XY or ALIGN STEP again.\n')

def mode_command(conn, cmd):
    """Single approved mode/stop command; no motor jog is sent."""
    before = query_status(conn)
    if before.get('moving') != '0' or before.get('homing') != '0':
        print('BLOCKED: motion or homing is active')
        return
    if cmd == 'MODE MANUAL':
        print('MODE MANUAL stops motion, sets stop=1, and INVALIDATES X/Y homing.')
        print('Z will remain stopped until a separate RESETSTOP is approved.')
    elif cmd == 'MODE AUTO':
        if before.get('stop') != '1':
            print('BLOCKED: firmware requires STOP before MODE AUTO')
            return
        print('MODE AUTO requires X/Y rehoming before any X/Y MOVE or PREVIEW.')
    elif cmd == 'RESETSTOP':
        print('RESETSTOP re-enables motion. In MANUAL, the joystick may immediately jog the selected axis.')
        print('Verify joystick is centered, selected axis and travel direction, clear space, and E-stop access.')
    elif cmd == 'STOP':
        print('STOP latches stop=1 and disables movement.')
    print('Confirm the machine is stationary, joystick centered, and travel area clear.')
    if input(f'Type YES to send {cmd}: ').strip() != 'YES':
        print('Cancelled')
        return
    again = query_status(conn)
    if again.get('moving') != '0' or again.get('homing') != '0':
        print('BLOCKED: machine state changed')
        return
    if cmd == 'MODE AUTO' and again.get('stop') != '1':
        print('BLOCKED: stop no longer latched')
        return
    conn.write((cmd + '\n').encode('ascii'))
    lines = read_until(conn, 2, None)
    expected = {'STOP': 'OK STOP', 'MODE MANUAL': 'OK MANUAL STOPPED',
                'MODE AUTO': 'OK AUTO', 'RESETSTOP': 'OK RESETSTOP'}[cmd]
    if not any(line.startswith(expected) for line in lines):
        print('WARNING: no expected acknowledgement. Inspect machine and status; do not continue.')
        return
    after = query_status(conn)
    expected_mode = 'MANUAL' if cmd == 'MODE MANUAL' else 'AUTO' if cmd == 'MODE AUTO' else None
    expected_stop = '0' if cmd == 'RESETSTOP' else '1'
    if (expected_mode is not None and after.get('mode') != expected_mode) or after.get('stop') != expected_stop:
        print('WARNING: unexpected mode/stop status; do not jog until resolved')
    else:
        print('Verified mode/stop response. This does NOT verify physical clearance or Z travel.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='/dev/cu.usbmodem1301')
    p.add_argument('--csv', type=Path, default=ROOT / 'washer_coordinates_fusion.csv')
    p.add_argument('--mapping', type=Path, default=ROOT / 'camera_machine_mapping_calibrated.json')
    p.add_argument('--max-age', type=float, default=30.0, help='Maximum seconds since S saved the YOLO detection')
    p.add_argument('--offline-status', help='Read-only testing without opening serial; accepts complete STATUS line')
    args = p.parse_args()
    if not (0 < args.max_age <= 120):
        p.error('--max-age must be >0 and <=120 seconds')
    cfg = json.loads(args.mapping.read_text(encoding='utf-8'))
    if cfg['steps_per_mm'] != [5.0, 5.0] or cfg['max_steps'] != [950, 1155] or cfg['max_command_steps'] != 50:
        p.error('Mapping disagrees with confirmed firmware settings; no connection opened')
    if args.offline_status:
        status = dict(re.findall(r'([A-Za-z][A-Za-z0-9]*)=([^ ]+)', args.offline_status))
        try:
            preview(status, args, cfg)
        except (ValueError, KeyError, OSError) as exc:
            print('BLOCKED:', exc)
        return

    import serial
    print('Opening the serial port may RESET the Arduino and invalidate homing.')
    print('Close the OLD supervised controller and Arduino Serial Monitor first.')
    print('Commands: STATUS, PING, STOP, MODE MANUAL, MODE AUTO, RESETSTOP, HOME XY, PREVIEW, PLAN XY, ALIGN STEP, MOVE X<number>, MOVE Y<number>, QUIT')
    print('PREVIEW/PLAN XY are read-only. ALIGN STEP executes at most one approved <=10-step move.')
    with serial.Serial(args.port, 9600, timeout=0.2, write_timeout=1) as conn:
        time.sleep(2.5)
        conn.reset_input_buffer()
        while True:
            try:
                cmd = input('Arduino+Vision> ').strip().upper()
                if cmd in ('QUIT', 'EXIT'):
                    break
                if cmd == 'STATUS':
                    query_status(conn)
                elif cmd == 'PREVIEW':
                    preview(query_status(conn), args, cfg)
                elif cmd == 'PLAN XY':
                    plan_xy(query_status(conn), args, cfg)
                elif cmd == 'ALIGN STEP':
                    align_step(conn, args, cfg)
                elif cmd == 'PING':
                    conn.write(b'PING\n')
                    read_until(conn, 2, 'PONG ')
                elif cmd in ('STOP', 'MODE MANUAL', 'MODE AUTO', 'RESETSTOP'):
                    mode_command(conn, cmd)
                elif cmd == 'HOME XY':
                    print('Verify stop cause resolved, joystick centered, clear travel path, E-stop accessible.')
                    if input(f'Type YES to send {cmd}: ').strip() != 'YES':
                        print('Cancelled')
                        continue
                    conn.write((cmd + '\n').encode('ascii'))
                    read_until(conn, 40, 'OK HOME Y DONE')
                    query_status(conn)
                else:
                    m = re.fullmatch(r'MOVE ([XY])([+-]?(?:\d+(?:\.\d*)?|\.\d+))', cmd)
                    if m:
                        amount = float(m.group(2))
                        if not math.isfinite(amount):
                            raise ValueError('Nonfinite movement')
                        send_move(conn, m.group(1), amount)
                    else:
                        print('BLOCKED: unrecognized command')
            except (ValueError, KeyError, RuntimeError, OSError, csv.Error) as exc:
                print('BLOCKED:', exc)
            except (KeyboardInterrupt, EOFError):
                print('\nClosing session; next connection may reset Arduino')
                break


if __name__ == '__main__':
    main()

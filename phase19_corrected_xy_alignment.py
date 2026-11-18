"""Phase 19: supervised camera-guided XY alignment with reversed camera Y mapping.

Requires existing phase5_supervised_controller.py,
phase10_supervised_xy_alignment.py, phase12_integrated_xy_alignment_go_to_view.py,
washer_coordinates_fusion.csv, and camera_machine_mapping_phase19_y_reversed.json.

IMPORTANT: Provisional camera-machine mapping; reaching commanded steps does NOT
verify physical pickup alignment. No Z commands. Operator must monitor motion,
keep physical emergency stop accessible, Z raised, and gripper detached.
"""
import argparse
import csv
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import serial
import phase10_supervised_xy_alignment as p10
import phase12_integrated_xy_alignment_go_to_view as p12
from phase5_supervised_controller import query_status, safe_status, read_until

ROOT = Path(__file__).resolve().parent
VIEW = (478, 563)
LIMITS = (950, 1155)
STEPS_PER_MM = 5
MAX_COMMAND_STEPS = 25             # 5 mm per individual MOVE
MAX_TOTAL_STEPS = 500              # 100 mm cumulative |dX| + |dY|
MAX_COMMANDS = 20                  # 500 steps / 25 steps per command
MAX_DETECTION_AGE_S = 5.0
MAX_TARGET_CHANGE_STEPS = 3


def position(status):
    return int(status['posXsteps']), int(status['posYsteps'])


def stop(conn, reason):
    print('HOLD:', reason, flush=True)
    try:
        conn.write(b'STOP\n')
        lines = read_until(conn, 2.0, 'OK STOP')
        if not any(line.startswith('OK STOP') for line in lines):
            print('WARNING: STOP not acknowledged; use physical E-stop if necessary.')
    except Exception as exc:
        print('WARNING: STOP communication failed:', exc,
              'Use physical E-stop if necessary.')


def checked_stop(conn, reason):
    stop(conn, reason)
    try:
        status = query_status(conn)
        if status.get('stop') != '1' or status.get('moving') != '0':
            print('WARNING: STOP NOT VERIFIED; use physical E-stop if necessary.')
        else:
            print('STOP VERIFIED; reported position:', position(status))
    except Exception as exc:
        print('WARNING: Cannot verify STOP:', exc,
              'Use physical E-stop if necessary.')


def target_from_camera(args, cfg):
    with args.csv.open(newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError('No camera detections')
    row = rows[-1]
    timestamp = row['timestamp_utc']
    if sum(r.get('timestamp_utc') == timestamp for r in rows) > 1:
        raise ValueError('Multiple detections in latest frame')
    ts = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
    if ts.tzinfo is None:
        raise ValueError('Timestamp has no timezone')
    age = (datetime.now(timezone.utc) - ts).total_seconds()
    x, y = float(row['workspace_x_mm']), float(row['workspace_y_mm'])
    confidence = float(row['confidence'])
    sx, sy = float(row['std_x_mm']), float(row['std_y_mm'])
    samples = int(row['stability_samples'])
    if not all(math.isfinite(v) for v in (age, x, y, confidence, sx, sy)):
        raise ValueError('Nonfinite detection')
    if not 0 <= age <= MAX_DETECTION_AGE_S:
        raise ValueError(f'Stale detection ({age:.2f}s)')
    if not (0.8 <= confidence <= 1 and 0 <= x <= 101.6 and 0 <= y <= 127):
        raise ValueError('Confidence/workspace invalid')
    if samples < 3 or not (0 <= sx <= 0.3 and 0 <= sy <= 0.3):
        raise ValueError('Washer position unstable')
    if 'OVERLAP' in row.get('status', '').upper():
        raise ValueError('Overlap')
    if row.get('motor_command_sent', '').strip().lower() not in ('false', '0'):
        raise ValueError('Unexpected motor-command flag')
    anchor_cam = cfg['anchor_camera_mm']
    anchor_machine = cfg['anchor_machine_mm']
    gains = cfg['camera_mm_per_commanded_mm']
    if any(not math.isfinite(float(v)) for v in anchor_cam + anchor_machine + gains):
        raise ValueError('Invalid mapping')
    if not (float(gains[0]) > 0 and float(gains[1]) < 0):
        raise ValueError('Phase 19 requires positive X gain and negative Y gain')
    target = tuple(round((float(anchor_machine[i]) +
                          ([x, y][i] - float(anchor_cam[i])) / float(gains[i]))
                         * STEPS_PER_MM) for i in range(2))
    if any(not 0 <= target[i] <= LIMITS[i] for i in range(2)):
        raise ValueError('Target outside soft limits')
    return target, row, age


def motion_plan(start, target):
    delta = (target[0] - start[0], target[1] - start[1])
    total = abs(delta[0]) + abs(delta[1])
    ncmd = sum((abs(d) + MAX_COMMAND_STEPS - 1) // MAX_COMMAND_STEPS
               for d in delta)
    return delta, total, ncmd


def align_xy(conn, args, cfg, session):
    if session['alignment_used'] or not session['homed'] or not session['view']:
        print('BLOCKED: fresh HOME XY and GO TO VIEW required in this serial session.')
        return
    status = query_status(conn)
    if not safe_status(status) or position(status) != VIEW:
        print('BLOCKED: AUTO, homed, idle, STOP released, VIEW position required.')
        return
    try:
        target, row, age = target_from_camera(args, cfg)
        delta, total, ncmd = motion_plan(VIEW, target)
        if total == 0:
            print('Already at commanded target; no motion.')
            checked_stop(conn, 'No alignment motion needed')
            return
        if total > MAX_TOTAL_STEPS or ncmd > MAX_COMMANDS:
            print(f'BLOCKED: travel {total} steps / {ncmd} commands exceeds run limit.')
            return
        if any(not 0 <= target[i] <= LIMITS[i] for i in range(2)):
            print('BLOCKED: target outside soft limits.')
            return
        print(f'Fresh detection {row["class_name"]} confidence={float(row["confidence"]):.3f} age={age:.2f}s')
        print(f'COMMANDED ALIGNMENT: {VIEW} -> {target} steps')
        print(f'X {delta[0]:+d} steps ({delta[0]/STEPS_PER_MM:+g} mm); '
              f'Y {delta[1]:+d} steps ({delta[1]/STEPS_PER_MM:+g} mm)')
        print(f'Total {total} steps ({total/STEPS_PER_MM:g} mm); '
              f'{ncmd} commands, X first then Y, <= {MAX_COMMAND_STEPS} steps each.')
        print('WARNING: Reversed Y mapping is UNVERIFIED; target may be inaccurate.')
        print('Check physical alignment and reference before authorizing any motion.')
        print('No Z commands. Z must be raised; gripper detached; path clear; '
              'joystick centered; physical E-stop accessible.')
        print('Check the ENTIRE X-then-Y path and machine clearance, not just the endpoint.')
        if input('Type PATH CLEAR to confirm the full path is clear: ').strip() != 'PATH CLEAR':
            print('Cancelled; no movement.')
            return
        if input('Type MAPPING PROVISIONAL to acknowledge unverified calibration: ').strip() != 'MAPPING PROVISIONAL':
            print('Cancelled; no movement.')
            return
        if input('Type ALIGN XY NOW to approve the entire X/Y sequence: ').strip() != 'ALIGN XY NOW':
            print('Cancelled; no movement.')
            return
        fresh, fresh_row, fresh_age = target_from_camera(args, cfg)
        if (fresh_row.get('track_id') != row.get('track_id') or
                any(abs(fresh[i] - target[i]) > MAX_TARGET_CHANGE_STEPS
                    for i in range(2))):
            raise RuntimeError('Fresh target changed after confirmation')
        status = query_status(conn)
        if not safe_status(status) or position(status) != VIEW:
            raise RuntimeError('Machine state changed after approval')
        session['alignment_used'] = True
        session['view'] = False
        current = VIEW
        count = 0
        for axis, idx in (('X', 0), ('Y', 1)):
            while current[idx] != target[idx]:
                count += 1
                if count > MAX_COMMANDS:
                    raise RuntimeError('Command count limit reached')
                status = query_status(conn)
                if not safe_status(status) or position(status) != current:
                    raise RuntimeError('Unexpected state before next move')
                remaining = target[idx] - current[idx]
                chunk = max(-MAX_COMMAND_STEPS, min(MAX_COMMAND_STEPS, remaining))
                expected = list(current)
                expected[idx] += chunk
                expected = tuple(expected)
                if not 0 <= expected[idx] <= LIMITS[idx]:
                    raise RuntimeError('Soft limit exceeded')
                command = f'MOVE {axis}{chunk / STEPS_PER_MM:g}\n'
                print(f'[{count}/{ncmd}] {axis}: {current} -> {expected}; '
                      f'sending {command.strip()}', flush=True)
                conn.write(command.encode('ascii'))
                lines = read_until(conn, 8.0, 'OK MOVE DONE')
                if not any(line.startswith('OK MOVE DONE') for line in lines):
                    raise RuntimeError('MOVE DONE not acknowledged')
                status = query_status(conn)
                if not safe_status(status) or position(status) != expected:
                    raise RuntimeError(f'Position mismatch: expected {expected}, '
                                       f'reported {position(status)}')
                current = expected
        print('COMMANDED XY TARGET REACHED by Arduino counters:', current)
        print('Physical washer-to-pickup alignment remains UNVERIFIED; inspect.')
        checked_stop(conn, 'Phase 19 XY sequence complete')
    except (Exception, KeyboardInterrupt) as exc:
        session['alignment_used'] = True
        session['view'] = False
        checked_stop(conn, f'Alignment aborted: {exc}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/cu.usbmodem1301')
    parser.add_argument('--csv', type=Path, default=ROOT / 'washer_coordinates_fusion.csv')
    parser.add_argument('--mapping', type=Path,
                        default=ROOT / 'camera_machine_mapping_phase19_y_reversed.json')
    args = parser.parse_args()
    cfg = json.loads(args.mapping.read_text())
    if (cfg.get('steps_per_mm') != [5.0, 5.0] or
            cfg.get('max_steps') != [950, 1155] or
            cfg.get('max_command_steps') != 50):
        parser.error('Mapping limits mismatch')
    print('PHASE 19: REVERSED Y CAMERA MAPPING; 500 TOTAL STEPS MAX; NO Z COMMANDS')
    print('Commands: STATUS, PREVIEW, MODE AUTO, RESETSTOP, STOP, HOME XY, GO TO VIEW, ALIGN XY, QUIT')
    print('Opening serial resets Arduino; rehome and GO TO VIEW before alignment.')
    print('Y CAMERA MAPPING SIGN REVERSED; anchor unchanged; CALIBRATION UNVERIFIED.')
    session = {'homed': False, 'view': False, 'alignment_used': False}
    with serial.Serial(args.port, 9600, timeout=0.2, write_timeout=1) as conn:
        time.sleep(2.5)
        conn.reset_input_buffer()
        while True:
            try:
                cmd = input('Phase19> ').strip().upper()
                if cmd in ('QUIT', 'EXIT'):
                    checked_stop(conn, 'Operator exiting controller')
                    break
                elif cmd == 'STATUS':
                    print(query_status(conn))
                elif cmd == 'PREVIEW':
                    target, row, age = target_from_camera(args, cfg)
                    delta, total, ncmd = motion_plan(VIEW, target)
                    print(f'YOLO {row["class_name"]} target {target} steps age {age:.2f}s; '
                          f'travel {total} steps / {ncmd} commands; NO MOVE')
                    if total > MAX_TOTAL_STEPS or ncmd > MAX_COMMANDS:
                        print('BLOCKED: exceeds Phase 19 travel/command limit')
                elif cmd in ('MODE AUTO', 'RESETSTOP', 'STOP'):
                    p10.mode_command(conn, cmd)
                    if cmd == 'MODE AUTO':
                        session.update(homed=False, view=False, alignment_used=False)
                    elif cmd == 'STOP':
                        session['view'] = False
                elif cmd == 'HOME XY':
                    p12.supervised_home_xy(conn)
                    status = query_status(conn)
                    session['homed'] = safe_status(status) and position(status) == (0, 0)
                    session['view'] = False
                    if session['homed']:
                        session['alignment_used'] = False
                    print('HOME VERIFIED:', session['homed'])
                elif cmd == 'GO TO VIEW':
                    if not session['homed'] or session['alignment_used']:
                        print('BLOCKED: fresh verified HOME XY required')
                        continue
                    p12.go_to_view(conn, cfg)
                    status = query_status(conn)
                    session['view'] = safe_status(status) and position(status) == VIEW
                    print('VIEW VERIFIED BY COUNTERS:', session['view'],
                          'Physically inspect carriage.')
                elif cmd == 'ALIGN XY':
                    align_xy(conn, args, cfg, session)
                else:
                    print('Unknown command; no movement')
            except (KeyboardInterrupt, EOFError):
                checked_stop(conn, 'Operator interrupted session')
                break
            except (ValueError, KeyError, RuntimeError, OSError, IndexError) as exc:
                checked_stop(conn, f'Command error: {exc}')
    print('Connection closed.')


if __name__ == '__main__':
    main()

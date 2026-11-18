"""WasherVision Phase 12: guided, operator-supervised XY alignment.

Uses Phase 10 single-step safety logic and supervised mode/stop handling.
Supports operator-approved XY homing and supervised camera-view positioning (single-step or one-confirmation sequential segments). No Z commands.
Run from the same directory as the Phase 10 controller and its dependencies.
"""
import argparse
import csv
import json
import math
import re
import time
from datetime import datetime, timezone
from pathlib import Path

import phase10_supervised_xy_alignment as p10
from phase5_supervised_controller import query_status, safe_status, send_move, read_until
from phase5_bridge_calibrated import plan

ROOT = Path(__file__).resolve().parent
DEADBAND_MM = 1.0
VIEW_STEPS = {"X": 478, "Y": 563}
VIEW_MAX_SEGMENT_STEPS = 50  # firmware limit; one operator-approved move per invocation


def frame_time(row):
    return datetime.fromisoformat(row['timestamp_utc'].replace('Z', '+00:00')).astimezone(timezone.utc)


def guided_alignment(conn, args, cfg, start):
    """One guided iteration per invocation. Never chains physical movements."""
    before = query_status(conn)
    if not safe_status(before):
        raise ValueError('Machine must be homed, stationary, safe, and in AUTO mode')
    print('\nPHASE 12 GUIDED ALIGNMENT — ONE ITERATION')
    print('Confirm: Z elevated, gripper detached, washer stable, clear XY paths, E-stop accessible.')
    print('Press S in the YOLO app NOW to save a NEW frame, then return here.')
    if input('Type READY after saving a new frame (anything else cancels): ').strip() != 'READY':
        print('Cancelled; no motor command sent')
        return
    row, age = p10.newest_frame(args.csv, args.max_age)
    # Require a frame created after the guided iteration began, not merely a recent old one.
    if frame_time(row) <= start:
        raise ValueError('Frame predates this iteration; press S and retry')
    current = query_status(conn)
    if not safe_status(current) or any(current.get(k) != before.get(k) for k in
        ('posXsteps', 'posYsteps', 'homeX', 'homeY', 'mode', 'axis', 'stop')):
        raise ValueError('Arduino state changed during capture; no movement sent')
    result = plan(row, current, cfg)
    if any('BLOCKED:' in w for w in result['warnings']):
        raise ValueError('Planner blocked: ' + '; '.join(result['warnings']))
    xerr, yerr = map(float, result['error_camera_mm'])
    if not all(math.isfinite(v) for v in (xerr, yerr)):
        raise ValueError('Nonfinite residual')
    print(f'Camera residual: X={xerr:+.2f} mm, Y={yerr:+.2f} mm; confidence={float(row["confidence"]):.2f}; age={age:.1f}s')
    if abs(xerr) <= DEADBAND_MM and abs(yerr) <= DEADBAND_MM:
        print('ALIGNMENT COMPLETE — within provisional camera deadband on BOTH axes.')
        print('No motor command sent. This does NOT prove physical pickup accuracy.')
        return
    print('At least one axis exceeds deadband. Delegating ONE correction to Phase 10 safeguards.')
    # Phase 10 verifies frame freshness, status, limits, per-move bound, and explicit YES.
    p10.align_step(conn, args, cfg)
    print('Iteration ended. If a move occurred, press S AFTER it stops, then run ALIGN WORKFLOW again.')
    print('Never reuse the pre-movement camera frame to claim alignment.')



def supervised_home_xy(conn):
    """Require operator approval and verify both axes homed after firmware homing."""
    before = query_status(conn)
    if before.get('mode') != 'AUTO' or before.get('stop') != '0':
        raise ValueError('HOME XY requires AUTO mode with stop=0')
    if before.get('moving') != '0' or before.get('homing') != '0':
        raise ValueError('HOME XY blocked while moving/homing')
    if before.get('axis') != 'XY':
        raise ValueError('HOME XY requires XY axis selection')
    if before.get('X') != 'CLOSED' or before.get('Y') != 'CLOSED':
        raise ValueError('Unexpected limit-switch state; inspect switches before homing')
    print('HOME XY WILL MOVE THE CARRIAGE toward the X/Y home switches.')
    print('Confirm Z elevated, gripper detached, joystick centered, home paths clear,')
    print('home-switch operation verified, and physical E-stop accessible.')
    if input('Type YES to perform ONE supervised HOME XY: ').strip() != 'YES':
        print('Cancelled; no homing command sent')
        return
    again = query_status(conn)
    fields = ('mode', 'axis', 'stop', 'moving', 'homing', 'X', 'Y', 'homeX', 'homeY', 'posXsteps', 'posYsteps')
    if any(again.get(k) != before.get(k) for k in fields):
        raise ValueError('Machine state changed before homing; no command sent')
    conn.write(b'HOME XY\n')
    lines = read_until(conn, 40, 'OK HOME Y DONE')
    if not any(line.startswith('OK HOME Y DONE') for line in lines):
        print('WARNING: Homing completion not acknowledged; inspect machine. Do not align.')
        return
    after = query_status(conn)
    if not safe_status(after) or after.get('posXsteps') != '0' or after.get('posYsteps') != '0':
        print('WARNING: Homing status did not verify; do not align.')
        return
    print('VERIFIED: Arduino reports X/Y homed at zero steps. Physical home accuracy is not independently verified.')



def go_to_view_step(conn, cfg):
    """Offer ONE supervised <=50-step move toward the historical camera view position.

    Never batches movements; user must invoke again for each segment.
    """
    before = query_status(conn)
    if not safe_status(before):
        raise ValueError('GO TO VIEW STEP requires AUTO, stop=0, homed XY, stationary, closed switches')
    limits = dict(zip(('X', 'Y'), cfg['max_steps']))
    current = {axis: int(before['pos' + axis + 'steps']) for axis in ('X', 'Y')}
    for axis in ('X', 'Y'):
        if not 0 <= current[axis] <= limits[axis]:
            raise ValueError('Reported machine position outside configured limits')
        if not 0 <= VIEW_STEPS[axis] <= limits[axis]:
            raise ValueError('Configured viewing position outside travel limits')
    print('\nPHASE 12 CAMERA VIEW — ONE SUPERVISED SEGMENT')
    print('Historical reference: X=478, Y=563 steps; NOT independently verified safe for this setup.')
    print(f'Current reported position: X={current["X"]}, Y={current["Y"]} steps')
    axis = next((a for a in ('X', 'Y') if current[a] != VIEW_STEPS[a]), None)
    if axis is None:
        print('VIEW REFERENCE REACHED (Arduino step counters only).')
        print('Check that the washer is actually visible, then capture a NEW YOLO S frame.')
        return
    delta = VIEW_STEPS[axis] - current[axis]
    steps = max(-VIEW_MAX_SEGMENT_STEPS, min(VIEW_MAX_SEGMENT_STEPS, delta))
    target = current[axis] + steps
    mm = steps / 5.0
    if not 0 <= target <= limits[axis]:
        raise ValueError('Proposed segment exceeds soft limits')
    print(f'PROPOSED ONE MOVE: {axis} {current[axis]} -> {target} steps ({steps:+d} steps / {mm:+g} mm)')
    print('Observe actual travel direction. Confirm Z elevated, gripper detached,')
    print('washer stable, ENTIRE segment path clear, and physical E-stop accessible.')
    print('This is NOT a collision-free path planner; the operator must inspect clearance.')
    if input('Type YES to execute exactly this ONE segment: ').strip() != 'YES':
        print('Cancelled; no movement sent')
        return
    again = query_status(conn)
    fields = ('mode', 'axis', 'stop', 'moving', 'homing', 'X', 'Y',
              'homeX', 'homeY', 'posXsteps', 'posYsteps')
    if not safe_status(again) or any(again.get(k) != before.get(k) for k in fields):
        raise ValueError('Machine state changed after approval; no movement sent')
    conn.write((f'MOVE {axis}{mm:g}\n').encode('ascii'))
    lines = read_until(conn, 8.0, 'OK MOVE DONE')
    if not any(line.startswith('OK MOVE DONE') for line in lines):
        print('WARNING: MOVE completion not acknowledged. Do not retry; inspect machine.')
        return
    after = query_status(conn)
    other = 'Y' if axis == 'X' else 'X'
    if (not safe_status(after) or
        int(after['pos' + axis + 'steps']) != target or
        int(after['pos' + other + 'steps']) != current[other]):
        print('WARNING: Unexpected reported position; STOP and investigate.')
        return
    print(f'VERIFIED Arduino reported {axis}={target}. Physical position not independently verified.')
    print('Inspect physical position before running GO TO VIEW STEP again.')
    print('No camera capture required until view reference is reached; do not ALIGN yet.')


def go_to_view(conn, cfg):
    """Single-approval sequential bounded moves to historical view reference.

    Operator must continuously supervise and use physical E-stop if needed.
    Each segment is independently checked; any uncertainty aborts the sequence.
    """
    before = query_status(conn)
    if not safe_status(before):
        raise ValueError('GO TO VIEW requires AUTO, stop=0, homed XY, stationary, closed switches')
    limits = dict(zip(('X', 'Y'), cfg['max_steps']))
    current = {a: int(before['pos' + a + 'steps']) for a in ('X', 'Y')}
    for a in ('X', 'Y'):
        if not (0 <= current[a] <= limits[a] and 0 <= VIEW_STEPS[a] <= limits[a]):
            raise ValueError('Current/target position outside configured travel limits')
    if current == VIEW_STEPS:
        print('VIEW REFERENCE REACHED (Arduino counters only). Check camera visibility.')
        return
    print('\nGO TO VIEW — SUPERVISED MULTI-SEGMENT SEQUENCE')
    print(f'Current reported X={current["X"]}, Y={current["Y"]} steps')
    print(f'Historical target X={VIEW_STEPS["X"]}, Y={VIEW_STEPS["Y"]} steps')
    print('Firmware permits up to 50 steps (10 mm) per command; controller will chain bounded moves.')
    print('Sequence is X first, then Y. It does NOT plan a collision-free route.')
    print('VERIFY ENTIRE X-THEN-Y PATH: Z elevated, gripper detached, washer secure,')
    print('joystick centered, both directions physically verified, no obstructions,')
    print('physical E-stop accessible. WATCH MOTION CONTINUOUSLY; press E-stop if unsafe.')
    print('The controller cannot independently detect a physical collision or missed steps.')
    if input('Type YES to approve the ENTIRE supervised sequence (anything else cancels): ').strip() != 'YES':
        print('Cancelled; no motor command sent')
        return
    fields = ('mode', 'axis', 'stop', 'moving', 'homing', 'X', 'Y',
              'homeX', 'homeY', 'posXsteps', 'posYsteps')
    again = query_status(conn)
    if not safe_status(again) or any(again.get(k) != before.get(k) for k in fields):
        raise ValueError('Machine state changed after approval; no movement sent')
    for axis in ('X', 'Y'):
        other = 'Y' if axis == 'X' else 'X'
        while current[axis] != VIEW_STEPS[axis]:
            # Recheck all safety state and both counters immediately before every move.
            state = query_status(conn)
            if not safe_status(state) or any(state.get(k) != again.get(k) for k in
                ('mode', 'axis', 'stop', 'moving', 'homing', 'X', 'Y', 'homeX', 'homeY')):
                print('ABORT: unsafe/changed machine state. Inspect machine; do not retry blindly.')
                return
            if any(int(state['pos' + a + 'steps']) != current[a] for a in ('X', 'Y')):
                print('ABORT: reported position changed unexpectedly. Inspect machine.')
                return
            delta = VIEW_STEPS[axis] - current[axis]
            steps = max(-VIEW_MAX_SEGMENT_STEPS, min(VIEW_MAX_SEGMENT_STEPS, delta))
            target = current[axis] + steps
            if not 0 <= target <= limits[axis]:
                print('ABORT: proposed segment exceeds configured travel limits')
                return
            mm = steps / 5.0
            print(f'MOVING {axis}: {current[axis]} -> {target} steps ({mm:+g} mm)', flush=True)
            conn.write((f'MOVE {axis}{mm:g}\n').encode('ascii'))
            lines = read_until(conn, 8.0, 'OK MOVE DONE')
            if not any(line.startswith('OK MOVE DONE') for line in lines):
                print('ABORT: movement completion not acknowledged. Inspect machine; do not retry blindly.')
                return
            state = query_status(conn)
            if (not safe_status(state) or
                int(state['pos' + axis + 'steps']) != target or
                int(state['pos' + other + 'steps']) != current[other]):
                print('ABORT: unexpected post-move status. Inspect machine; do not retry blindly.')
                return
            current[axis] = target
    print('VIEW REFERENCE REACHED according to Arduino step counters only.')
    print('Physically inspect the carriage and confirm washer is visible before YOLO capture.')
    print('No Z movement was commanded. Do not ALIGN unless camera visibility is verified.')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/cu.usbmodem1301')
    parser.add_argument('--csv', type=Path, default=ROOT / 'washer_coordinates_fusion.csv')
    parser.add_argument('--mapping', type=Path, default=ROOT / 'camera_machine_mapping_calibrated.json')
    parser.add_argument('--max-age', type=float, default=30.0)
    args = parser.parse_args()
    if not 0 < args.max_age <= 30:
        parser.error('Phase 12 requires 0 < max-age <= 30 seconds')
    cfg = json.loads(args.mapping.read_text(encoding='utf-8'))
    if cfg['steps_per_mm'] != [5.0, 5.0] or cfg['max_steps'] != [950, 1155] or cfg['max_command_steps'] != 50:
        parser.error('Mapping does not match confirmed firmware; no serial connection opened')
    import serial
    print('WARNING: Opening serial may RESET Arduino and invalidate XY homing.')
    print('Close all other serial connections before starting.')
    print('Commands: STATUS, PREVIEW, PLAN XY, ALIGN WORKFLOW, ALIGN STEP, STOP, RESETSTOP, MODE AUTO, MODE MANUAL, HOME XY, GO TO VIEW STEP, GO TO VIEW, QUIT')
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
                    p10.preview(query_status(conn), args, cfg)
                elif cmd == 'PLAN XY':
                    p10.plan_xy(query_status(conn), args, cfg)
                elif cmd == 'ALIGN WORKFLOW':
                    guided_alignment(conn, args, cfg, datetime.now(timezone.utc))
                elif cmd == 'ALIGN STEP':
                    p10.align_step(conn, args, cfg)
                elif cmd in ('STOP', 'RESETSTOP', 'MODE AUTO', 'MODE MANUAL'):
                    p10.mode_command(conn, cmd)
                elif cmd == 'HOME XY':
                    supervised_home_xy(conn)
                elif cmd == 'GO TO VIEW STEP':
                    go_to_view_step(conn, cfg)
                elif cmd == 'GO TO VIEW':
                    go_to_view(conn, cfg)
                else:
                    print('BLOCKED: unknown command; no manual jogs in Phase 12.')
            except (ValueError, KeyError, RuntimeError, OSError, csv.Error) as exc:
                print('BLOCKED:', exc)
            except (KeyboardInterrupt, EOFError):
                print('\nSession closing. Reopening serial may reset homing.')
                break


if __name__ == '__main__':
    main()

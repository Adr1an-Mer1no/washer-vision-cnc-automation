"""WasherVision supervised persistent Arduino session.

No automatic alignment or motion. Each MOVE is manually typed, safety checked,
confirmed, and checked against firmware STATUS before/after execution.
The serial port is opened only once. Keep Arduino Serial Monitor closed.
"""
import argparse
import math
import re
import time
import serial

STEPS_PER_MM = 5
LIMITS = {'X': 950, 'Y': 1155}
ALLOWED_INIT = {'MODE AUTO', 'RESETSTOP', 'HOME XY'}


def read_until(conn, seconds=2.0, end_prefix=None):
    end = time.monotonic() + seconds
    lines = []
    while time.monotonic() < end:
        raw = conn.readline()
        if not raw:
            continue
        line = raw.decode('utf-8', errors='replace').strip()
        if line:
            print('<', line, flush=True)
            lines.append(line)
            if line.startswith('ERR ') or 'STOPPED' in line:
                break
            if end_prefix and line.startswith(end_prefix):
                break
    return lines


def query_status(conn):
    conn.reset_input_buffer()
    conn.write(b'STATUS\n')
    lines = read_until(conn, 2.0, 'STATUS ')
    matching = [line for line in lines if line.startswith('STATUS ')]
    if not matching:
        raise RuntimeError('No STATUS received; no movement allowed')
    return dict(re.findall(r'([A-Za-z][A-Za-z0-9]*)=([^ ]+)', matching[-1]))


def safe_status(status):
    expected = {'mode': 'AUTO', 'axis': 'XY', 'stop': '0', 'moving': '0',
                'homing': '0', 'homeX': '1', 'homeY': '1',
                'X': 'CLOSED', 'Y': 'CLOSED'}
    return all(status.get(k) == v for k, v in expected.items())


def send_move(conn, axis, mm):
    status = query_status(conn)
    if not safe_status(status):
        print('BLOCKED: controller not homed, idle, and safe in AUTO mode')
        return
    steps = math.floor(mm * STEPS_PER_MM + 0.5) if mm >= 0 else math.ceil(mm * STEPS_PER_MM - 0.5)
    if steps == 0 or abs(mm) > 10 or abs(steps) > 50:
        print('BLOCKED: move must be nonzero and at most 10 mm (50 steps)')
        return
    pos = int(status['pos' + axis + 'steps'])
    target = pos + steps
    if not 0 <= target <= LIMITS[axis]:
        print(f'BLOCKED: {axis} target {target} outside [0, {LIMITS[axis]}] steps')
        return
    print(f'PROPOSED: {axis} {pos} -> {target} steps ({mm:+g} mm requested; {steps:+d} steps).')
    print('Check physical clearance, secured washer, and emergency-stop access.')
    if input('Type YES to execute this single movement: ').strip() != 'YES':
        print('Cancelled')
        return
    # Recheck state immediately before movement; no other process should own port.
    again = query_status(conn)
    if not safe_status(again) or int(again['pos' + axis + 'steps']) != pos:
        print('BLOCKED: machine state changed since approval')
        return
    conn.write((f'MOVE {axis}{mm:g}\n').encode('ascii'))
    lines = read_until(conn, 8.0, 'OK MOVE DONE')
    if not any(line.startswith('OK MOVE DONE') for line in lines):
        print('WARNING: no MOVE DONE acknowledgement. Inspect machine; do not continue.')
        return
    after = query_status(conn)
    if not safe_status(after) or int(after['pos' + axis + 'steps']) != target:
        print('WARNING: unexpected post-move STATUS; stop and investigate')
    else:
        print(f'VERIFIED: {axis} reached {target} reported steps. Physical travel still requires observation.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='/dev/cu.usbmodem1301')
    args = p.parse_args()
    print('Supervised serial session. Opening port may reset Arduino ONCE.')
    print('Commands: STATUS, PING, MODE AUTO, RESETSTOP, HOME XY, MOVE X<number>, MOVE Y<number>, QUIT')
    print('No GOTO, batch moves, or automatic movement supported.')
    with serial.Serial(args.port, 9600, timeout=0.2, write_timeout=1) as conn:
        time.sleep(2.5)
        conn.reset_input_buffer()
        while True:
            try:
                cmd = input('Arduino> ').strip().upper()
            except (EOFError, KeyboardInterrupt):
                print('\nSession closed; opening a new connection may reset Arduino.')
                break
            if cmd in {'QUIT', 'EXIT'}:
                break
            if cmd == 'STATUS':
                query_status(conn)
            elif cmd == 'PING':
                conn.write(b'PING\n')
                read_until(conn, 2.0, 'PONG ')
            elif cmd in ALLOWED_INIT:
                print('Only proceed after verifying the stop cause is resolved, joystick centered, and path clear.')
                if input(f'Type YES to send {cmd}: ').strip() != 'YES':
                    print('Cancelled')
                    continue
                conn.write((cmd + '\n').encode('ascii'))
                read_until(conn, 40 if cmd == 'HOME XY' else 2.0,
                           'OK HOME Y DONE' if cmd == 'HOME XY' else None)
                query_status(conn)
            else:
                m = re.fullmatch(r'MOVE ([XY])([+-]?(?:\d+(?:\.\d*)?|\.\d+))', cmd)
                if not m:
                    print('BLOCKED: unrecognized command')
                    continue
                try:
                    amount = float(m.group(2))
                    if not math.isfinite(amount):
                        raise ValueError()
                    send_move(conn, m.group(1), amount)
                except (ValueError, KeyError, RuntimeError) as exc:
                    print('BLOCKED:', exc)

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Set PX4's offboard-loss failsafe (the last layer, when the whole Pi is dead) to RTL or LAND.

Layers, outermost last:  AS2 mission safety -> FC guardian on the Pi (AS2 dead) -> PX4 failsafe
(Pi or XRCE link dead: no offboard setpoints for COM_OF_LOSS_T seconds -> COM_OBL_RC_ACT).
Use the same choice as the guardian's emergency_action, so every layer does the same thing.

  # hardware: FC USB cable on the Pi (stop anything else using the port first) or a laptop
  px4_failsafe_params.py --mavlink /dev/ttyACM0 --action RTL
  # SITL: PX4's own shell client for instance N (what sitl_onboard.sh runs)
  px4_failsafe_params.py --sitl-instance 0 --action LAND
  # only show the related parameters
  px4_failsafe_params.py --mavlink udpout:127.0.0.1:18570 --check

PX4 v1.14 parameter names (src/modules/commander/commander_params.c):
  COM_OBL_RC_ACT  offboard loss action: 0 Position, 1 Altitude, 2 Manual, 3 Return, 4 Land, 5 Hold,
                  6 Terminate, 7 Disarm                 -> set to 3 (RTL) or 4 (LAND)
  COM_OF_LOSS_T   seconds without offboard setpoints before that action (default 1.0) -> set
  RTL_RETURN_ALT  this drone's own return height: base + step x slot (slot = system ID - 1). Use the
                  fleet separation as the step (--return-step): 16 / 26 / 36 m with 10 m (ordinary GNSS),
                  16 / 19 / 22 m with 3 m (RTK), so drones returning together keep it by height alone
  RTL_CONE_ANG    0: always climb to RTL_RETURN_ALT (PX4's default cone of 45 deg lets a drone near
                  home return lower, which would undo the separation)
Shown, not changed (set them once for the airframe in QGC):
  NAV_DLL_ACT     GCS datalink loss. The GCS talks to the Pi's gateway, never to PX4 itself, so PX4
                  never sees a GCS: keep 0 (disabled) or PX4 failsafes right after arming.
  NAV_RCL_ACT, COM_RCL_EXCEPT  RC loss action / exceptions (bit 2 = ignore RC loss in offboard)
  COM_RC_IN_MODE  RC input mode (4 = no RC required)
  RTL_DESCEND_ALT, COM_DISARM_LAND
"""
import argparse
import os
import struct
import subprocess
import sys
import time

try:
    from as2_rescue_runtime.rtl import RETURN_BASE_M, RETURN_STEP_M, return_altitude
except ImportError:  # run without the rescue workspace sourced (e.g. as2_pi.sh failsafe): same rule
    RETURN_BASE_M, RETURN_STEP_M = 16.0, 10.0

    def return_altitude(slot, base=RETURN_BASE_M, step=RETURN_STEP_M):
        if not (0 < base <= 120 and 0 <= step <= 20) or base + step * max(0, slot) > 120:
            raise ValueError('return altitude must stay within 0..120 m')
        return base + step * max(0, int(slot))

ACTION_VALUE = {'RTL': 3, 'LAND': 4}
ACTION_NAMES = {0: 'Position', 1: 'Altitude', 2: 'Manual', 3: 'Return (RTL)', 4: 'Land', 5: 'Hold',
                6: 'Terminate', 7: 'Disarm'}
SHOW = ('COM_OBL_RC_ACT', 'COM_OF_LOSS_T', 'NAV_DLL_ACT', 'NAV_RCL_ACT', 'COM_RCL_EXCEPT',
        'COM_RC_IN_MODE', 'RTL_RETURN_ALT', 'RTL_CONE_ANG', 'RTL_DESCEND_ALT', 'COM_DISARM_LAND')
INT, FLOAT = 6, 9   # MAV_PARAM_TYPE_INT32 / REAL32
TYPES = {'COM_OF_LOSS_T': FLOAT, 'RTL_RETURN_ALT': FLOAT, 'RTL_DESCEND_ALT': FLOAT, 'COM_DISARM_LAND': FLOAT}
PX4_BIN = os.environ.get('PX4_BIN_DIR', '/home/prince/px4_sitl/PX4-Autopilot/build/px4_sitl_default/bin')


def wanted(args, slot):
    if args.check:
        return {}
    return {'COM_OBL_RC_ACT': ACTION_VALUE[args.action], 'COM_OF_LOSS_T': float(args.loss_timeout),
            'RTL_RETURN_ALT': float(return_altitude(slot, args.return_base, args.return_step)),
            'RTL_CONE_ANG': 0}


# ---------------------------------------------------------------- SITL (PX4 shell client)
def sitl(args):
    client = os.path.join(PX4_BIN, 'px4-param')
    if not os.access(client, os.X_OK):
        raise SystemExit('px4-param not found in {} (set PX4_BIN_DIR)'.format(PX4_BIN))
    base = [client, '--instance', str(args.sitl_instance)]
    values = wanted(args, args.slot if args.slot >= 0 else args.sitl_instance)
    for name, value in values.items():
        subprocess.run(base + ['set', name, str(value)], check=True, stdout=subprocess.DEVNULL, timeout=10)
    shown = ''
    for name in values or ('COM_OBL_RC_ACT', 'COM_OF_LOSS_T', 'RTL_RETURN_ALT', 'RTL_CONE_ANG'):
        shown += subprocess.run(base + ['show', name], capture_output=True, text=True, timeout=10).stdout
    lines = [line.strip() for line in shown.splitlines() if 'COM_' in line or 'RTL_' in line]
    print('PX4 SITL instance {}: {}'.format(args.sitl_instance, '; '.join(lines) or shown.strip()))
    return verify_text(lines, values)


def verify_text(lines, values):
    """Each parameter's own 'px4-param show' line must end with the value set."""
    for name, value in values.items():
        expected = ('{:.4f}'.format(value) if isinstance(value, float) else str(value))
        line = next((l for l in lines if (' ' + name + ' ') in (' ' + l + ' ')), '')
        if not line.rstrip().endswith(': ' + expected):
            print('NOT CONFIRMED: {} = {}'.format(name, value), file=sys.stderr)
            return 1
    return 0


# ---------------------------------------------------------------- MAVLink (hardware / any PX4)
def encode(name, value):
    """PX4 uses byte-wise parameter encoding: an INT32 travels as its raw bits in the float field."""
    if TYPES.get(name, INT) == INT:
        return struct.unpack('<f', struct.pack('<i', int(value)))[0], INT
    return float(value), FLOAT


def decode(msg):
    if msg.param_type == INT:
        return struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
    return round(msg.param_value, 3)


def read(link, system, name, timeout=3.0):
    link.mav.param_request_read_send(system, 1, name.encode(), -1)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        msg = link.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg is not None and msg.get_srcSystem() == system and msg.param_id.rstrip('\x00') == name:
            return decode(msg)
    return None


def mavlink(args):
    from pymavlink import mavutil
    link = mavutil.mavlink_connection(args.mavlink, baud=args.baud, source_system=254, source_component=190)
    if args.mavlink.startswith('udpout'):
        link.mav.heartbeat_send(6, 8, 0, 0, 4)   # let PX4 learn our address
    heartbeat = link.recv_match(type='HEARTBEAT', blocking=True, timeout=10)
    if heartbeat is None:
        raise SystemExit('No PX4 heartbeat on {}'.format(args.mavlink))
    system = args.system or heartbeat.get_srcSystem()
    status = 0
    for name, value in wanted(args, args.slot if args.slot >= 0 else system - 1).items():
        packed, kind = encode(name, value)
        for _ in range(3):
            link.mav.param_set_send(system, 1, name.encode(), packed, kind)
            if read(link, system, name) == (round(value, 3) if kind == FLOAT else value):
                break
        else:
            print('NOT CONFIRMED: {} = {}'.format(name, value), file=sys.stderr)
            status = 1
    print('PX4 system {} on {}:'.format(system, args.mavlink))
    for name in SHOW:
        value = read(link, system, name)
        note = ' ({})'.format(ACTION_NAMES.get(value, '?')) if name == 'COM_OBL_RC_ACT' and value is not None else ''
        warn = '   <- should be 0: PX4 has no direct GCS link' if name == 'NAV_DLL_ACT' and value not in (0, None) else ''
        print('  {:16} {}{}{}'.format(name, value, note, warn))
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    where = parser.add_mutually_exclusive_group(required=True)
    where.add_argument('--mavlink', help="PX4 MAVLink URL: /dev/ttyACM0, udpout:127.0.0.1:18570, ...")
    where.add_argument('--sitl-instance', type=int, help='PX4 SITL instance number (uses px4-param)')
    parser.add_argument('--action', type=str.upper, choices=sorted(ACTION_VALUE), default='RTL')
    parser.add_argument('--loss-timeout', type=float, default=1.0, help='COM_OF_LOSS_T seconds (default 1.0)')
    parser.add_argument('--check', action='store_true', help='only show the parameters')
    parser.add_argument('--baud', type=int, default=57600)
    parser.add_argument('--system', type=int, default=0, help='PX4 MAV_SYS_ID (default: from heartbeat)')
    parser.add_argument('--slot', type=int, default=-1,
                        help='return-altitude slot (default: SITL instance, or system ID - 1)')
    parser.add_argument('--return-base', type=float, default=RETURN_BASE_M,
                        help='RTL_RETURN_ALT of slot 0, metres above home (default %(default)s)')
    parser.add_argument('--return-step', type=float, default=RETURN_STEP_M,
                        help='extra height per slot, metres (default %(default)s)')
    args = parser.parse_args()
    if not 0.2 <= args.loss_timeout <= 60:
        parser.error('--loss-timeout must be 0.2..60 s')
    try:
        return_altitude(max(args.slot, 0), args.return_base, args.return_step)
    except ValueError as exc:
        parser.error(str(exc))
    sys.exit(sitl(args) if args.sitl_instance is not None else mavlink(args))


if __name__ == '__main__':
    main()

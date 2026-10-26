"""Start the vendored A8 tracker (hybrid_tracker.py) with the installed model files.

Modes:
  preview  detection + selection only, no gimbal commands (A8_PREVIEW_ONLY=1)
  track    detection + selection + gimbal tracking (bench-verify first, props off)

Defaults match run_tracking_multi.sh from the multilock1 package. Extra arguments after
``--`` are passed to hybrid_tracker.py unchanged, e.g. ``-- --conf 0.45``.
"""
import argparse
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory


# Gimbal response profiles (onboard panel: Camera & AI -> Gimbal speed). "normal" is the tracker's
# original tuning; "fast"/"max" raise the SIYI speed cap and the gains and aim ahead of a moving target.
# SIYI units are not deg/s: bench-check each profile with props off before flight.
GIMBAL_PROFILES = {
    'normal': dict(cap=35, kp_yaw=20.0, kp_pitch=16.0, lead_s=0.0),
    'fast': dict(cap=70, kp_yaw=30.0, kp_pitch=24.0, lead_s=0.15),
    'max': dict(cap=100, kp_yaw=40.0, kp_pitch=32.0, lead_s=0.2),
}


def gimbal_arguments(profile, command_cap=None):
    """hybrid_tracker.py arguments for a gimbal profile; an explicit --command-cap still wins."""
    if profile not in GIMBAL_PROFILES:
        raise ValueError('gimbal speed must be one of: ' + ', '.join(GIMBAL_PROFILES))
    chosen = GIMBAL_PROFILES[profile]
    cap = chosen['cap'] if command_cap is None else command_cap
    return ['--siyi-command-cap', str(cap), '--kp-yaw', str(chosen['kp_yaw']),
            '--kp-pitch', str(chosen['kp_pitch']), '--lead-s', str(chosen['lead_s'])]


def tracker_home():
    return Path(get_package_share_directory('as2_a8_tracker')) / 'tracker'


def active_model(home):
    """The model chosen in the onboard panel (model library, active.json), else the shipped one."""
    sys.path.insert(0, str(home / 'src'))
    try:
        import model_library
    except ImportError as exc:   # tracker files older than the model library
        return home / 'model', [], [], (640, 360), 'model library unavailable (%s)' % exc
    return model_library.active_launch(home / 'model')


def build_command(arguments, extra):
    home = tracker_home()
    # The tracker needs newer numpy/OpenCV/ncnn than ROS Humble's Python: the Pi image installs
    # them in a separate environment and points A8_TRACKER_PYTHON at it.
    python = os.environ.get('A8_TRACKER_PYTHON') or sys.executable
    model, classes, lock_classes, capture, note = active_model(home)
    if note:
        print('a8_tracker: ' + note, file=sys.stderr, flush=True)
    threads = arguments.threads
    try:   # detector threads chosen in the panel from the measured CPU left by AS2
        import model_library
        chosen = int(model_library.load_active().get('threads') or 0)
        if chosen and not note:
            threads = min(chosen, len(arguments.detector_cores or arguments.cores or [0, 1, 2, 3]))
    except (ImportError, ValueError, TypeError):
        pass
    command = [python, str(home / 'src' / 'hybrid_tracker.py'),
               '--model', str(model), '--nano-dir', str(home / 'model'),
               '--res', '%dx%d' % tuple(capture),
               '--rtsp', arguments.rtsp, '--siyi-ip', arguments.camera_ip,
               '--gimbal-mode', 'siyi_udp', '--mavlink', 'none', '--no-scan-on-start',
               '--threads', str(threads), '--kd-yaw', '0', '--kd-pitch', '0',
               ] + gimbal_arguments(getattr(arguments, 'gimbal_speed', 'normal'), arguments.command_cap) + [
               '--min-person-height', str(arguments.min_person_height),
               # dualtrack2 (2026-10-24): its own gimbal control and single-target selection, so no
               # --control-profile / --target-policy / --target-hold-seconds.
               '--telemetry-port', str(arguments.telemetry_port),
               '--control-port', str(arguments.control_port),
               '--log-dir', arguments.log_dir, '--headless']
    if arguments.cores:
        command += ['--cores'] + [str(c) for c in arguments.cores]
        if arguments.detector_cores:
            command += ['--detector-cores'] + [str(c) for c in arguments.detector_cores]
    else:
        command.append('--no-pin')
    if arguments.stream:
        command += ['--stream', str(arguments.stream)]
    if arguments.mode == 'preview':
        command.append('--attitude-in-preview')
    if classes:
        command += ['--classes'] + list(classes)
    if lock_classes:
        command += ['--lock-classes'] + list(lock_classes)
    return command + list(extra)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    extra = []
    if '--' in argv:
        index = argv.index('--')
        argv, extra = argv[:index], argv[index + 1:]
    # Drop ROS launch arguments if started through launch_ros.
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    parser = argparse.ArgumentParser(prog='a8_tracker', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--mode', choices=['preview', 'track'], default='preview')
    parser.add_argument('--camera-ip', default='192.168.144.25')
    parser.add_argument('--rtsp', default=None,
                        help='Defaults to rtsp://<camera-ip>:8554/main.264')
    parser.add_argument('--threads', type=int, default=2)
    parser.add_argument('--cores', type=int, nargs='*', default=[2, 3],
                        help='CPU cores for vision (Pi 5: 2 3). Empty list disables pinning')
    parser.add_argument('--detector-cores', type=int, nargs='*', default=[],
                        help='Own cores for the person detector (Pi 5: 2 3; --cores then 0 1). Empty = share --cores')
    parser.add_argument('--stream', type=int, default=8080, help='MJPEG preview port, 0 = off')
    parser.add_argument('--min-person-height', type=float, default=24)
    parser.add_argument('--command-cap', type=int, default=None,
                        help='SIYI speed cap 1..100; default: from --gimbal-speed (normal = 35)')
    parser.add_argument('--gimbal-speed', choices=sorted(GIMBAL_PROFILES), default='normal',
                        help='Gimbal response: normal (original), fast, max')
    parser.add_argument('--telemetry-port', type=int, default=9090)
    parser.add_argument('--control-port', type=int, default=9091)
    parser.add_argument('--log-dir', default=str(Path.home() / '.local/share/as2-rescue-logs/a8_tracker'))
    parser.add_argument('--print-only', action='store_true', help='Print the command and exit')
    arguments = parser.parse_args(argv)
    arguments.rtsp = arguments.rtsp or 'rtsp://%s:8554/main.264' % arguments.camera_ip
    command = build_command(arguments, extra)
    if arguments.print_only:
        print(' '.join(command))
        return 0
    Path(arguments.log_dir).mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, A8_PREVIEW_ONLY='1' if arguments.mode == 'preview' else '0',
                       OMP_NUM_THREADS=str(arguments.threads), OPENBLAS_NUM_THREADS='1')
    os.chdir(str(tracker_home()))
    os.execve(command[0], command, environment)


if __name__ == '__main__':
    sys.exit(main())

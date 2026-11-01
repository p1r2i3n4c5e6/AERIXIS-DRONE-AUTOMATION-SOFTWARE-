"""How each onboard link is started — shared by the Pi GUI (as2_pi_launcher.py) and the
headless Docker service (as2_onboard_service.py), so both bring an aircraft up identically.

Every function takes ``value(name)``: a lookup into the saved settings
(~/.config/as2/pi_launcher.json), falling back to DEFAULTS. No GUI imports here.
"""
import hashlib
import json
import os
import secrets
import sys
from pathlib import Path

ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')
ROS_SETUP = '/opt/ros/{}/setup.bash'.format(ROS_DISTRO)
AS2_ROOT = Path(os.environ.get('AS2_WORKSPACE', Path(__file__).resolve().parents[2]))
RESCUE_ROOT = Path(os.environ.get('AS2_RESCUE_WORKSPACE', AS2_ROOT / 'rescue_ws'))
CONFIG_DIR = Path(os.environ.get('AS2_CONFIG_DIR', Path.home() / '.config' / 'as2'))
CONFIG_FILE = CONFIG_DIR / 'pi_launcher.json'
MISSION_ROOT = Path(os.environ.get('AS2_MISSION_STATE_ROOT', Path.home() / '.local/share/as2-missions'))
XRCE_AGENT = Path(os.environ.get('AS2_XRCE_AGENT', AS2_ROOT / 'install/px4_microxrce_agent/bin/MicroXRCEAgent'))

DEFAULTS = dict(
    drone_id='drone0', system_id='1',
    gcs_mode='serial', gcs_device='/dev/ttyUSB0', gcs_baud='57600', gcs_host='192.168.1.100', gcs_port='14551',
    gcs_bind_port='14550', web_port='8090',
    mesh_mode='both', mesh_device='/dev/ttyUSB1', mesh_baud='57600', mesh_port='24800',
    mesh_peers_ip='192.168.1.255', mesh_names='drone1,drone2', mesh_passphrase='',
    fc_mode='serial', fc_device='/dev/ttyAMA0', fc_baud='921600', fc_udp_port='8888', fc_health_url='/dev/ttyACM0',
    fmu_prefix='', camera_mode='preview', camera_ip='192.168.144.25', vision_cores='0 1', detector_cores='2 3',
    gimbal_speed='normal',
    sim_persons='20,15;-25,40',
    allow_flight=False, path_clear=False, operator_payload=False, overhead_verified=False,
    drop_min_agl='8', drop_max_agl='20', aux_set='0', aux_value='1.0', separation_m='10',
    payload_servos='', payload_hold='-1.0',
    emergency_action='RTL', guardian_timeout='3.0')


GIMBAL_SPEEDS = ('normal', 'fast', 'max')   # as2_a8_tracker tracker_process.GIMBAL_PROFILES


def load_settings():
    settings = dict(DEFAULTS)
    try:
        settings.update(json.loads(CONFIG_FILE.read_text()))
    except (OSError, ValueError):
        pass
    # AS2_<KEY>=... in the environment overrides the file (handy in docker compose).
    for key in DEFAULTS:
        env = os.environ.get('AS2_' + key.upper())
        if env is not None:
            settings[key] = env.lower() in ('1', 'true', 'yes', 'on') if isinstance(DEFAULTS[key], bool) else env
    return settings


def shell(command, system_python_first=True):
    """Run a command with ROS, AS2 and the rescue overlay sourced.

    AS2 links put the system Python packages first: a pip numpy 2.x in ~/.local breaks the
    system scipy that AS2's Python API imports. The camera tracker needs the pip packages
    (ncnn, OpenCV with TrackerNano), so it runs with the normal order (in the Docker image it
    uses its own environment instead, see A8_TRACKER_PYTHON).
    """
    first = 'export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH; ' if system_python_first else ''
    inner = ('source {ros} && source {as2}/install/setup.bash 2>/dev/null; '
             'source {rescue}/install/setup.bash 2>/dev/null; {first}exec {cmd}'
             ).format(ros=ROS_SETUP, as2=AS2_ROOT, rescue=RESCUE_ROOT, first=first, cmd=command)
    return ['bash', '-c', inner]


def ros_run(package, executable, workspace):
    """'ros2 run' without the ros2cli start-up (0.13 s per link on the dev laptop, more on a Pi):
    exec the installed console script directly; fall back to 'ros2 run' for other layouts."""
    path = Path(workspace) / 'install' / package / 'lib' / package / executable
    return str(path) if os.access(str(path), os.X_OK) else 'ros2 run {} {}'.format(package, executable)


def drone_id(value):
    drone = str(value('drone_id')).strip()
    if not drone or not drone.replace('_', '').isalnum() or drone[0].isdigit():
        raise ValueError('Drone ID must look like drone0')
    return drone


def token_file(drone):
    return MISSION_ROOT / drone / 'pairing.token'


def ensure_token(drone):
    path = token_file(drone)
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        descriptor = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(descriptor, 'w') as stream:
            stream.write(secrets.token_urlsafe(48) + '\n')
    return path


def peers(value):
    return [n.strip() for n in str(value('mesh_names')).split(',') if n.strip()]


def mesh_key_file(value, drone):
    passphrase = str(value('mesh_passphrase'))
    if len(passphrase) < 8:
        raise ValueError('Enter the fleet mesh passphrase (8+ characters, same on every drone).')
    key = hashlib.sha256(('as2-mesh:' + passphrase).encode()).hexdigest()
    config = dict(send_key=key, receive_keys={name: key for name in set(peers(value)) | {'base'}})
    path = CONFIG_DIR / 'mesh_{}.json'.format(drone)
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(descriptor, 'w') as stream:
        json.dump(config, stream)
    return path


def payload_servos(spec, aux_set=0):
    """[(actuator set 1..6, drops)] from the panel's "set:drops,set:drops" (several packages,
    2026-11-01); '' = the single legacy aux_set. Same rules as as2_rescue_runtime.payload_node
    servo_list (kept here because the panel runs without ROS)."""
    import re
    out = []
    for part in str(spec or '').replace(' ', '').split(','):
        if not part:
            continue
        match = re.fullmatch(r'([1-6])(?::(\d{1,2}))?', part)
        if not match:
            raise ValueError('Servo "{}": use set:drops with set 1..6'.format(part))
        drops = int(match.group(2) or 1)
        if not 1 <= drops <= 20:
            raise ValueError('Servo on set {}: 1..20 drops'.format(match.group(1)))
        if any(s == int(match.group(1)) for s, _ in out):
            raise ValueError('Actuator set {} is listed twice'.format(match.group(1)))
        out.append((int(match.group(1)), drops))
    if not out and 1 <= int(aux_set or 0) <= 6:
        out = [(int(aux_set), 1)]
    return out


def write_profile(value, drone, mesh_active):
    """Per-aircraft parameter override applied on top of the packaged aircraft.yaml."""
    import yaml
    flight = bool(value('allow_flight'))
    operator = bool(value('operator_payload'))
    aux_set = int(value('aux_set'))
    servos = payload_servos(value('payload_servos'), aux_set)
    others = [p for p in peers(value) if p != drone] or ['']
    profile = {
        '/**/rescue_runtime': {'ros__parameters': dict(
            enable_motion=flight, allow_operator_load=operator,
            overhead_envelope_verified=bool(value('overhead_verified')),
            min_overhead_agl_m=float(value('drop_min_agl')), max_overhead_agl_m=float(value('drop_max_agl')),
            minimum_separation_m=float(value('separation_m')))},
        '/**/rescue_route_publisher': {'ros__parameters': dict(
            origin_source='estimator', altitude_frame_verified=True,
            path_clear_verified=bool(value('path_clear')),
            # Same folder the mission manager stages into (AS2_MISSION_STATE_ROOT aware).
            bundle_directory=str(MISSION_ROOT / drone))},
        '/**/swarm_agent': {'ros__parameters': dict(
            expected_peers=others if mesh_active else [''],
            minimum_separation_m=float(value('separation_m')))},
        '/**/payload_aux_adapter': {'ros__parameters': dict(
            enabled=bool(servos), allow_operator_load=operator, actuator_set=aux_set,
            servos=','.join('{}:{}'.format(s, n) for s, n in servos), hold_value=float(value('payload_hold')),
            release_value=float(value('aux_value')), target_system=int(value('system_id')),
            mission_directory=str(MISSION_ROOT / drone))},
    }
    path = CONFIG_DIR / '{}_runtime.yaml'.format(drone)
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(profile, sort_keys=False))
    return path


def _device(path, what, hint=''):
    if not Path(path).exists():
        raise ValueError('{} {} not found.{}'.format(what, path, ' ' + hint if hint else ''))
    return path


# ---------------------------------------------------------------- link commands (argv)
def cmd_fc(value):
    """Micro XRCE-DDS agent between the AERIXIS core and PX4 (UART on the Pi, UDP for SITL)."""
    if not XRCE_AGENT.is_file():
        raise ValueError('MicroXRCEAgent not installed at {}. Build the AERIXIS workspace (install/) first.'.format(XRCE_AGENT))
    lib = XRCE_AGENT.parent.parent / 'lib'
    if value('fc_mode') == 'serial':
        device = _device(value('fc_device'), 'Flight controller port',
                         'On a Pi 5 enable the UART (dtparam=uart0=on) and use /dev/ttyAMA0.')
        command = 'serial --dev {} -b {}'.format(device, int(value('fc_baud')))
    else:
        command = 'udp4 -p {}'.format(int(value('fc_udp_port')))
    # ROS must be sourced: the agent links against ROS 2's Fast-DDS libraries.
    return ['bash', '-c', 'source {} && export LD_LIBRARY_PATH={}:$LD_LIBRARY_PATH && exec {} {}'.format(
        ROS_SETUP, lib, XRCE_AGENT, command)]


def cmd_as2(value, profile):
    sim = value('fc_mode') == 'udp'
    arguments = 'drone_id:={} target_system_id:={} override:={} use_sim_time:=false'.format(
        drone_id(value), int(value('system_id')), profile)
    # ros2 launch rejects 'name:=' with an empty value: pass optional arguments only when set
    # (the launch file's defaults are empty already).
    if str(value('fmu_prefix')).strip():
        arguments += ' fmu_prefix:={}'.format(value('fmu_prefix'))
    health = str(value('fc_health_url') or '').strip()
    if sim and not health.startswith('udp'):
        health = ''   # SITL: a serial FC USB path does not exist; 'udpin:0.0.0.0:14540' does
    if health:
        arguments += ' fc_mavlink_url:={}'.format(health)
    return shell('ros2 launch as2_rescue_runtime onboard.launch.py ' + arguments)


def cmd_manager(value):
    command = '{}/scripts/start_mission_manager.sh {}'.format(RESCUE_ROOT, drone_id(value))
    if value('allow_flight'):
        command += ' --allow-flight'
    return shell(command)


def cmd_gcs(value):
    drone = drone_id(value)
    token = ensure_token(drone)
    common = ('-p drone_namespace:={} -p system_id:={} -p fleet_token_file:={} '
              '-p bind_address:=0.0.0.0 -p udp_port:={}').format(drone, int(value('system_id')), token, int(value('gcs_bind_port')))
    # PX4 link liveness under the same prefix as the AERIXIS core uses: a real FC publishes
    # /fmu/... (fmu_prefix ''); the gateway's own default /<drone>/fmu/... only matches namespaced SITL.
    common += ' -p px4_status_topic:={}/fmu/out/vehicle_status_v1'.format(str(value('fmu_prefix') or '').strip())
    # MAVLink waypoint missions as go_to legs: the onboard core runs takeoff/go_to/follow_path/land,
    # not the polynomial trajectory behaviour the gateway defaulted to (such a mission failed).
    common += ' -p use_trajectory:=false'
    # Pose/twist through the AERIXIS core's rate gate (25 Hz, rescue.launch.py): telemetry over a
    # 57600 radio needs no more, and 100 Hz cost this Python node much of a Pi 5 core (2026-10-25).
    common += (' -r /{0}/self_localization/pose:=/{0}/rescue/in/pose'
               ' -r /{0}/self_localization/twist:=/{0}/rescue/in/twist').format(drone)
    if value('gcs_mode') == 'serial':
        device = _device(value('gcs_device'), 'Telemetry radio', 'Plug it in or pick another port.')
        args = common + ' -p serial_device:={} -p serial_baud:={}'.format(device, int(value('gcs_baud')))
    else:
        args = common + ' -p gcs_host:={} -p gcs_port:={}'.format(value('gcs_host'), int(value('gcs_port')))
    return shell(ros_run('as2_mavlink_gateway', 'mavlink_gateway', AS2_ROOT) + ' --ros-args ' + args)


def cmd_mesh(value):
    """Drone-to-drone mesh. 'both' (default): the base-station network is the main link and the LoRa
    modem an always-on fallback (mesh_bridge transport:=both; a missing modem is retried, the base
    station carries on). 'udp': base station / IP mesh only. 'serial': LoRa only."""
    drone = drone_id(value)
    key = mesh_key_file(value, drone)
    mode = str(value('mesh_mode'))
    if mode not in ('both', 'udp', 'serial'):
        raise ValueError('Mesh mode must be both, udp, serial or off')
    parts = []
    if mode in ('udp', 'both'):
        ips = [p.strip() for p in str(value('mesh_peers_ip')).replace(';', ',').split(',') if p.strip()]
        if not ips:
            raise ValueError("Enter the other drones' base-station IPs or the subnet broadcast address.")
        port = int(value('mesh_port'))
        parts.append('-p bind_port:={} -p "peers:=[{}]"'.format(
            port, ','.join("'{}:{}'".format(p, port) if ':' not in p else "'{}'".format(p) for p in ips)))
    if mode in ('serial', 'both'):
        device = _device(value('mesh_device'), 'LoRa modem') if mode == 'serial' else str(value('mesh_device'))
        parts.append('-p device:={} -p baud:={}'.format(device, int(value('mesh_baud'))))
    transport = '-p transport:={} {}'.format(mode, ' '.join(parts))
    return shell(ros_run('as2_rescue_runtime', 'mesh_bridge', RESCUE_ROOT) + ' --ros-args -r __ns:=/{} '
                 '-p identity:={} -p key_file:={} {}'.format(drone, drone, key, transport))


def emergency_action(value):
    action = str(value('emergency_action')).strip().upper()
    if action not in ('RTL', 'LAND'):
        raise ValueError('Emergency action must be RTL or LAND')
    return action


def cmd_guardian(value):
    """FC guardian: emergency RTL/LAND straight to PX4 over the XRCE link when the AERIXIS core stops.
    Needs only the XRCE agent; started before the AERIXIS core and restarted on its own."""
    # guardian.launch.py: the guardian plus its own C++ rate gate for PX4's ~100 Hz local position
    # (height only, 20 Hz). Own gate, so the guardian never depends on the AERIXIS core it watches;
    # either process exiting ends the link and the supervisor restarts both.
    arguments = 'drone_id:={} target_system:={} emergency_action:={} as2_timeout_s:={}'.format(
        drone_id(value), int(value('system_id')), emergency_action(value), float(value('guardian_timeout')))
    if str(value('fmu_prefix')).strip():
        arguments += ' fmu_prefix:={}'.format(value('fmu_prefix'))
    return shell('ros2 launch as2_rescue_runtime guardian.launch.py ' + arguments)


def sim_camera_available():
    """The simulated camera ships with the laptop SITL build only; the Pi image leaves it out."""
    return (Path(RESCUE_ROOT) / 'install' / 'as2_a8_tracker' / 'lib' / 'as2_a8_tracker' / 'a8_sim_vision').exists()


def cmd_camera(value):
    mode = value('camera_mode')
    if mode == 'off':
        return None
    if mode == 'sim' and not sim_camera_available():
        raise ValueError('The simulated camera is not installed on this onboard computer: pick Preview or Track.')
    speed = str(value('gimbal_speed') or 'normal')
    if speed not in GIMBAL_SPEEDS:
        raise ValueError('Gimbal speed must be one of: ' + ', '.join(GIMBAL_SPEEDS))
    arguments = 'drone_id:={} vision:={} camera_ip:={} cores:="{}" detector_cores:="{}" gimbal_speed:={}'.format(
        drone_id(value), mode, value('camera_ip'), value('vision_cores'), value('detector_cores'), speed)
    if mode == 'sim':
        arguments += ' sim_persons:="{}"'.format(value('sim_persons'))
    return shell('ros2 launch as2_a8_tracker a8_tracker.launch.py ' + arguments, system_python_first=False)


def cmd_web(value):
    """Onboard control panel (web page) — started last, after every other link."""
    port = int(value('web_port') or 0)
    if not port:
        return None
    return [sys.executable, str(Path(__file__).resolve().parent / 'onboard_web.py'), '--port', str(port)]


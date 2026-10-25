#!/usr/bin/env python3
"""AERIXIS onboard launcher for Raspberry Pi 5 (or any companion computer).

A small window that connects the aircraft's links and starts the AERIXIS core. It never arms, flies
or releases anything by itself: missions are uploaded and started from the laptop GCS
over the telemetry radio, and the onboard mission manager and rescue runtime keep their
own gates.

  1  GCS telemetry     long-range radio (serial, chosen baud) or UDP to the laptop GCS
  2  Mesh network      drone-to-drone only: LoRa UART modem or IP mesh / base station
  3  Flight controller Micro XRCE-DDS agent over UART (or UDP for PX4 SITL)
  4  AERIXIS core       AERIXIS core, rescue runtime, track logger, onboard mission manager
  5  Camera            A8 mini person detection: preview, gimbal tracking or SITL sim

Run:  rescue_ws/scripts/run_pi_launcher.sh
"""
import json
import os
import signal
import socket
import subprocess
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, scrolledtext, ttk

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
# Link commands, defaults and paths are shared with the headless Docker service.
from onboard_links import CONFIG_DIR, CONFIG_FILE, DEFAULTS, ROS_SETUP  # noqa: E402
import onboard_links as links  # noqa: E402

BAUDS = ['9600', '19200', '38400', '57600', '115200', '230400', '460800', '921600', '1500000']
COLORS = dict(off='#9aa3ad', starting='#e0a400', on='#21a35b', warn='#e07b00', error='#d23b3b')

def serial_ports():
    ports = set()
    try:
        from serial.tools import list_ports
        ports.update(p.device for p in list_ports.comports())
    except ImportError:
        pass
    for pattern in ('ttyUSB*', 'ttyACM*', 'ttyAMA*', 'serial*', 'ttyS0'):
        ports.update(str(p) for p in Path('/dev').glob(pattern))
    return sorted(ports)


def local_addresses():
    addresses = []
    try:
        output = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=2).stdout
        addresses = output.split()
    except (OSError, subprocess.TimeoutExpired):
        pass
    return addresses or ['127.0.0.1']


class Link:
    """One child process group that can be started and stopped."""

    def __init__(self, name, log):
        self.name, self.log = name, log
        self.process = None
        self.command = []
        self.state = 'off'
        self.tail = ''

    def running(self):
        return self.process is not None and self.process.poll() is None

    def start(self, command, env=None):
        if self.running():
            self.log('[{}] already running\n'.format(self.name))
            return
        self.command = command
        self.state = 'starting'
        self.log('[{}] start: {}\n'.format(self.name, command[-1] if command[0] == 'bash' else ' '.join(command)))
        try:
            self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                            stdin=subprocess.DEVNULL, text=True, bufsize=1,
                                            start_new_session=True, env=env)
        except (OSError, ValueError) as exc:
            self.state = 'error'
            self.log('[{}] could not start: {}\n'.format(self.name, exc))
            return
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        for line in iter(self.process.stdout.readline, ''):
            self.tail = line.strip()[-160:]
            self.log('[{}] {}'.format(self.name, line))
        code = self.process.wait()
        self.state = 'off' if code in (0, -2, -15) else 'error'
        self.log('[{}] ended ({})\n'.format(self.name, code))

    def stop(self):
        if not self.running():
            self.state = 'off'
            return
        self.log('[{}] stopping\n'.format(self.name))
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        except (ProcessLookupError, PermissionError):
            pass
        self.state = 'off'


class Monitor:
    """Read-only ROS view of the aircraft for the status lights (optional)."""

    def __init__(self, drone):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        from as2_msgs.msg import PlatformInfo
        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('pi_launcher_monitor_' + drone)
        self.data, self.at = {}, {}
        prefix = '/' + drone + '/'
        for key, topic in (('rescue', 'rescue/state'), ('vision', 'vision/target'),
                           ('mesh', 'swarm/radio_health'), ('swarm', 'swarm/status'),
                           ('route', 'rescue/route_health'), ('payload', 'payload/status')):
            self.node.create_subscription(String, prefix + topic, lambda m, k=key: self.store(k, json.loads(m.data)), 10)
        self.node.create_subscription(PlatformInfo, prefix + 'platform/info', lambda m: self.store('platform', dict(
            connected=m.connected, armed=m.armed, offboard=m.offboard, state=m.status.state)), 10)
        self.running = True
        self.thread = threading.Thread(target=self.spin, daemon=True)
        self.thread.start()

    def store(self, key, value):
        self.data[key], self.at[key] = value, time.monotonic()

    def get(self, key, max_age=2.0):
        if key in self.at and time.monotonic() - self.at[key] <= max_age:
            return self.data[key]
        return None

    def spin(self):
        while self.running and self.rclpy.ok():
            try:
                self.rclpy.spin_once(self.node, timeout_sec=0.2)
            except Exception:
                time.sleep(0.2)

    def close(self):
        self.running = False
        self.thread.join(timeout=1)
        self.node.destroy_node()


class Launcher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('AERIXIS Onboard - Raspberry Pi 5')
        self.geometry('860x700')
        self.minsize(720, 560)
        self.settings = dict(DEFAULTS)
        try:
            self.settings.update(json.loads(CONFIG_FILE.read_text()))
        except (OSError, ValueError):
            pass
        self.vars = {}
        self.links = {name: Link(name, self.log) for name in ('gcs', 'mesh', 'fc', 'as2', 'manager', 'camera')}
        # FC guardian: started with the FC link, independent of the AERIXIS core (no light of its own; shown on the FC row).
        self.guardian = Link('guardian', self.log)
        self.monitor = None
        style = ttk.Style(self)
        style.configure('Title.TLabel', font=('TkDefaultFont', 11, 'bold'))
        style.configure('Big.TButton', padding=6)

        top = ttk.Frame(self, padding=(10, 8, 10, 0))
        top.pack(fill='x')
        ttk.Label(top, text='Drone ID').pack(side='left')
        ttk.Entry(top, textvariable=self.var('drone_id'), width=10).pack(side='left', padx=(4, 12))
        ttk.Label(top, text='MAVLink system ID').pack(side='left')
        ttk.Entry(top, textvariable=self.var('system_id'), width=5).pack(side='left', padx=(4, 12))
        ttk.Label(top, text='This computer: ' + ', '.join(local_addresses()[:3])).pack(side='left')
        ttk.Button(top, text='Stop all', command=self.stop_all).pack(side='right')
        ttk.Button(top, text='Connect all', style='Big.TButton', command=self.connect_all).pack(side='right', padx=6)

        notebook = ttk.Notebook(self)
        notebook.pack(fill='both', expand=True, padx=10, pady=8)
        self.connect_tab(notebook)
        self.safety_tab(notebook)
        self.status_tab(notebook)
        log_frame = ttk.LabelFrame(self, text='Log')
        log_frame.pack(fill='both', expand=False, padx=10, pady=(0, 10))
        self.output = scrolledtext.ScrolledText(log_frame, height=9, wrap='word', font=('TkFixedFont', 9))
        self.output.pack(fill='both', expand=True, padx=4, pady=4)
        self.protocol('WM_DELETE_WINDOW', self.close)
        self.after(1000, self.refresh)

    # ------------------------------------------------------------------ widgets
    def var(self, name, kind=tk.StringVar):
        if name not in self.vars:
            value = self.settings.get(name, DEFAULTS.get(name, ''))
            self.vars[name] = tk.BooleanVar(value=bool(value)) if isinstance(DEFAULTS.get(name), bool) else kind(value=str(value))
        return self.vars[name]

    def value(self, name):
        return self.vars[name].get() if name in self.vars else self.settings.get(name, DEFAULTS.get(name))

    def row(self, parent, title, link, help_text):
        frame = ttk.Frame(parent, padding=(8, 6))
        frame.pack(fill='x')
        light = tk.Canvas(frame, width=16, height=16, highlightthickness=0)
        light.grid(row=0, column=0, rowspan=2, sticky='n', pady=4)
        dot = light.create_oval(2, 2, 14, 14, fill=COLORS['off'], outline='')
        ttk.Label(frame, text=title, style='Title.TLabel').grid(row=0, column=1, sticky='w', padx=6)
        detail = ttk.Label(frame, text=help_text, foreground='#56606b')
        detail.grid(row=0, column=2, columnspan=6, sticky='w')
        body = ttk.Frame(frame)
        body.grid(row=1, column=1, columnspan=7, sticky='w', padx=6, pady=(4, 0))
        self.lights = getattr(self, 'lights', {})
        self.lights[link] = (light, dot, detail, help_text)
        ttk.Separator(parent).pack(fill='x', padx=8)
        return body

    def combo(self, parent, name, values, width=14):
        box = ttk.Combobox(parent, textvariable=self.var(name), values=values, width=width)
        box.pack(side='left', padx=(2, 8))
        return box

    def entry(self, parent, label, name, width=14):
        if label:
            ttk.Label(parent, text=label).pack(side='left')
        ttk.Entry(parent, textvariable=self.var(name), width=width).pack(side='left', padx=(2, 8))

    def radios(self, parent, name, options):
        for value, text in options:
            ttk.Radiobutton(parent, text=text, value=value, variable=self.var(name)).pack(side='left', padx=(0, 6))

    def connect_tab(self, notebook):
        tab = ttk.Frame(notebook)
        notebook.add(tab, text='Connect')
        ports = serial_ports()
        body = self.row(tab, '1  GCS telemetry', 'gcs', 'Long-range radio to the laptop GCS. Missions arrive here.')
        self.radios(body, 'gcs_mode', [('serial', 'Radio (serial)'), ('udp', 'UDP')])
        self.port_boxes = [self.combo(body, 'gcs_device', ports)]
        ttk.Label(body, text='baud').pack(side='left')
        self.combo(body, 'gcs_baud', BAUDS, 8)
        self.entry(body, 'GCS IP', 'gcs_host', 13)
        ttk.Button(body, text='Connect', command=self.start_gcs).pack(side='left', padx=4)

        body = self.row(tab, '2  Mesh network', 'mesh', 'Drone-to-drone only (never to the GCS). Same passphrase on every drone.')
        self.radios(body, 'mesh_mode', [('both', 'Base station + LoRa fallback'), ('udp', 'Base station / IP mesh only'), ('serial', 'LoRa only')])
        self.port_boxes.append(self.combo(body, 'mesh_device', ports))
        self.combo(body, 'mesh_baud', BAUDS, 8)
        ttk.Button(body, text='Connect', command=self.start_mesh).pack(side='left', padx=4)
        body2 = ttk.Frame(body.master)
        body2.grid(row=2, column=1, columnspan=7, sticky='w', padx=6, pady=(4, 0))
        self.entry(body2, 'Peer IPs / broadcast', 'mesh_peers_ip', 22)
        self.entry(body2, 'Other drones', 'mesh_names', 16)
        ttk.Label(body2, text='Passphrase').pack(side='left')
        ttk.Entry(body2, textvariable=self.var('mesh_passphrase'), show='•', width=12).pack(side='left', padx=2)

        body = self.row(tab, '3  Flight controller', 'fc', 'Micro XRCE-DDS agent between the AERIXIS core and PX4 (TELEM2 UART on the Pi).')
        self.radios(body, 'fc_mode', [('serial', 'UART'), ('udp', 'UDP (SITL)')])
        self.port_boxes.append(self.combo(body, 'fc_device', ports))
        self.combo(body, 'fc_baud', BAUDS, 8)
        self.entry(body, 'UDP port', 'fc_udp_port', 6)
        ttk.Button(body, text='Connect', command=self.start_fc).pack(side='left', padx=4)

        body = self.row(tab, '4  AERIXIS core + missions', 'as2', 'AERIXIS core, rescue runtime, track logger and onboard mission manager.')
        self.entry(body, 'PX4 topic prefix', 'fmu_prefix', 10)
        # Read-only MAVLink tap: sensor health + PX4 warnings (compass/mag calibration...) for the GCS card.
        self.entry(body, 'FC USB health tap', 'fc_health_url', 14)
        ttk.Button(body, text='Start AERIXIS core', command=self.start_as2).pack(side='left', padx=4)
        ttk.Button(body, text='Pairing token', command=self.show_token).pack(side='left', padx=4)

        body = self.row(tab, '5  Camera (A8 mini)', 'camera', 'Person detection + gimbal tracking at the camera IP.')
        self.radios(body, 'camera_mode', [('off', 'Off'), ('preview', 'Detect only'),
                                          ('track', 'Detect + gimbal track'), ('sim', 'SITL sim')])
        self.entry(body, 'IP', 'camera_ip', 14)
        ttk.Button(body, text='Start', command=self.start_camera).pack(side='left', padx=4)
        ttk.Button(tab, text='Refresh serial ports', command=self.refresh_ports).pack(anchor='e', padx=10, pady=6)

    def safety_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text='Safety profile')
        ttk.Label(tab, text='These switches are written to the aircraft profile when the AERIXIS core starts. '
                            'Leave them off for bench tests.', wraplength=760).pack(anchor='w', pady=(0, 8))
        checks = [('allow_flight', 'Allow flight: AERIXIS may arm, take off and fly uploaded missions'),
                  ('path_clear', 'Flight area checked: no obstacles at survey altitude inside the KML area'),
                  ('operator_payload', 'Operator payload mode: I confirm loading in the GCS (servo, no load sensor)'),
                  ('overhead_verified', 'Overhead drop envelope verified with an inert payload at the heights below')]
        for name, text in checks:
            ttk.Checkbutton(tab, text=text, variable=self.var(name)).pack(anchor='w', pady=2)
        grid = ttk.Frame(tab)
        grid.pack(anchor='w', pady=8)
        for row, (label, name) in enumerate([('Drop height min (m above take-off)', 'drop_min_agl'),
                                             ('Drop height max (m above take-off)', 'drop_max_agl'),
                                             ('PX4 offboard actuator set for the servo (1-6, 0 = off)', 'aux_set'),
                                             ('Servo release value (-1..1)', 'aux_value'),
                                             ('Minimum drone separation (m)', 'separation_m')]):
            ttk.Label(grid, text=label).grid(row=row, column=0, sticky='w', pady=2)
            ttk.Entry(grid, textvariable=self.var(name), width=8).grid(row=row, column=1, sticky='w', padx=8)
        row = ttk.Frame(tab)
        row.pack(anchor='w', pady=4)
        ttk.Label(row, text='Emergency action if the AERIXIS core stops in flight (FC guardian; set PX4 COM_OBL_RC_ACT the same)').pack(side='left')
        self.combo(row, 'emergency_action', ['RTL', 'LAND'], 6)
        ttk.Label(tab, text='The actuator set is PX4 "Offboard Actuator Set N" mapped to your AUX pin in the PX4 '
                            'Actuators page, not the AUX pin number. Test the servo with props removed.',
                  wraplength=760, foreground='#56606b').pack(anchor='w', pady=6)
        self.entry_sim = ttk.Frame(tab)
        self.entry_sim.pack(anchor='w', pady=6)
        self.entry(self.entry_sim, 'SITL simulated people (x,y;x,y metres)', 'sim_persons', 26)

    def status_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text='Status')
        self.status_text = tk.StringVar(value='Start the AERIXIS core to see live status.')
        ttk.Label(tab, textvariable=self.status_text, justify='left', font=('TkFixedFont', 10)).pack(anchor='w')

    # ------------------------------------------------------------------ helpers
    def log(self, text):
        def write():
            self.output.insert('end', text)
            if int(self.output.index('end-1c').split('.')[0]) > 2000:
                self.output.delete('1.0', '500.0')
            self.output.see('end')
        self.after(0, write)

    def save(self):
        for name, variable in self.vars.items():
            self.settings[name] = variable.get()
        CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        CONFIG_FILE.write_text(json.dumps(self.settings, indent=2))

    def drone(self):
        drone = str(self.value('drone_id')).strip()
        if not drone.replace('_', '').isalnum() or drone[0].isdigit():
            raise ValueError('Drone ID must look like drone0')
        return drone

    def refresh_ports(self):
        ports = serial_ports()
        for box in self.port_boxes:
            box['values'] = ports
        self.log('[launcher] serial ports: {}\n'.format(', '.join(ports) or 'none found'))

    def guarded(self, action):
        try:
            self.save()
            action()
        except (ValueError, OSError) as exc:
            messagebox.showerror('AERIXIS launcher', str(exc))

    def token_file(self):
        return links.token_file(self.drone())

    def ensure_token(self):
        return links.ensure_token(self.drone())

    def show_token(self):
        def show():
            path = self.ensure_token()
            token = path.read_text().strip()
            self.clipboard_clear()
            self.clipboard_append(token)
            messagebox.showinfo('Pairing token', 'Drone {}  ·  system ID {}\n\nToken (copied to clipboard):\n{}\n\n'
                                'Enter both in the laptop GCS Fleet page for this drone.'.format(
                                    self.drone(), self.value('system_id'), token))
        self.guarded(show)

    # ------------------------------------------------------------------ links
    def start_gcs(self):
        self.guarded(lambda: self.links['gcs'].start(links.cmd_gcs(self.value)))

    def mesh_key_file(self, drone, names):
        return links.mesh_key_file(self.value, drone)

    def peers(self):
        return [n.strip() for n in str(self.value('mesh_names')).split(',') if n.strip()]

    def start_mesh(self):
        self.guarded(lambda: self.links['mesh'].start(links.cmd_mesh(self.value)))

    def start_fc(self):
        def run():
            self.links['fc'].start(links.cmd_fc(self.value))
            self.guardian.stop()
            self.guardian.start(links.cmd_guardian(self.value))
        self.guarded(run)

    def write_profile(self, drone):
        return links.write_profile(self.value, drone, self.links['mesh'].running() or bool(self.value('mesh_passphrase')))

    def start_as2(self):
        def run():
            drone = self.drone()
            profile = self.write_profile(drone)
            sim = self.value('fc_mode') == 'udp'
            self.links['as2'].start(links.cmd_as2(self.value, profile))
            self.ensure_token()
            self.links['manager'].start(links.cmd_manager(self.value))
            if sim:
                self.log('[launcher] UDP flight-controller link: assuming PX4 SITL on this computer\n')
            self.start_monitor(drone)
        self.guarded(run)

    def start_camera(self):
        def run():
            drone = self.drone()
            mode = self.value('camera_mode')
            self.links['camera'].stop()
            if mode == 'off':
                return
            if mode == 'track' and not messagebox.askyesno(
                    'Gimbal tracking', 'The gimbal will move by itself when a person is selected.\n'
                    'Make sure the camera can rotate freely. Continue?'):
                return
            self.links['camera'].start(links.cmd_camera(self.value))
            if mode in ('preview', 'track'):
                self.log('[launcher] live video: http://{}:8080\n'.format(local_addresses()[0]))
        self.guarded(run)

    def start_monitor(self, drone):
        if self.monitor is not None:
            return
        try:
            self.monitor = Monitor(drone)
        except Exception as exc:  # ROS not sourced in this shell: lights use process state only.
            self.log('[launcher] live status unavailable ({}); run via run_pi_launcher.sh\n'.format(exc))

    def connect_all(self):
        steps = [('fc', self.start_fc), ('as2', self.start_as2), ('gcs', self.start_gcs)]
        if self.value('mesh_passphrase'):
            steps.insert(1, ('mesh', self.start_mesh))
        if self.value('camera_mode') != 'off':
            steps.append(('camera', self.start_camera))

        def chain(index=0):
            if index < len(steps):
                steps[index][1]()
                self.after(2500, chain, index + 1)
        chain()

    def stop_all(self):
        def run():
            for name in ('camera', 'manager', 'gcs', 'mesh', 'as2', 'fc'):
                if name == 'fc':
                    self.guardian.stop()
                self.links[name].stop()
            self.log('[launcher] everything stopped\n')
        threading.Thread(target=run, daemon=True).start()

    def close(self):
        if any(link.running() for link in self.links.values()):
            if not messagebox.askyesno('Quit', 'Stop all AERIXIS links and close?'):
                return
            for name in ('camera', 'manager', 'gcs', 'mesh', 'as2', 'fc'):
                if name == 'fc':
                    self.guardian.stop()
                self.links[name].stop()
        try:
            self.save()
        except OSError:
            pass
        if self.monitor:
            self.monitor.close()
        self.destroy()

    # ------------------------------------------------------------------ status
    def light(self, name, state, text=None):
        canvas, dot, label, default = self.lights[name]
        canvas.itemconfigure(dot, fill=COLORS[state])
        label.configure(text=text or default)

    def refresh(self):
        m = self.monitor
        platform = m.get('platform') if m else None
        rescue = m.get('rescue') if m else None
        vision = m.get('vision') if m else None
        mesh = m.get('mesh') if m else None
        route = m.get('route', 3) if m else None
        for name, link in self.links.items():
            if name == 'manager':
                continue
            state = 'starting' if link.running() else link.state
            text = None
            if link.running():
                state = 'on'
            if name == 'fc' and link.running():
                if platform:
                    state, text = ('on', 'PX4 connected · armed={} · offboard={}'.format(platform['armed'], platform['offboard'])) \
                        if platform['connected'] else ('warn', 'Agent running, PX4 not connected yet')
                else:
                    state, text = 'starting', 'Agent running; start the AERIXIS core to confirm the PX4 link'
                text += ' · guardian {}'.format('on' if self.guardian.running() else 'NOT RUNNING')
                if not self.guardian.running():
                    state = 'warn'
            if name == 'as2' and link.running():
                if rescue:
                    flags = rescue.get('flags', {})
                    healthy = flags.get('VehicleHealthy')
                    state = 'on' if healthy else 'warn'
                    text = 'AERIXIS core up · vehicle {} · motion {} · {}'.format(
                        'healthy' if healthy else 'NOT ready', 'enabled' if rescue.get('motion_enabled') else 'disabled',
                        (route or {}).get('origin_error') or 'origin OK' if route else 'waiting for route node')
                else:
                    state, text = 'starting', 'Starting the AERIXIS core… (needs the flight controller link)'
                if not self.links['manager'].running():
                    state, text = 'error', 'Mission manager stopped: ' + (self.links['manager'].tail or 'see log')
            if name == 'mesh' and link.running() and mesh is not None:
                peers = mesh.get('peers') or {}
                state = 'on' if peers and all(age < 3 for age in peers.values()) else 'warn'
                text = 'Mesh {} · peers: {}'.format(mesh.get('transport', ''), ', '.join(
                    '{} {:.1f}s'.format(k, v) for k, v in peers.items()) or 'none heard yet')
            if name == 'camera' and link.running() and vision is not None:
                state = 'on' if vision.get('fresh') else 'warn'
                text = 'Vision {} · {} · {:.0f} fps{}'.format(vision.get('source'), vision.get('lock_state'),
                                                             vision.get('fps') or 0,
                                                             ' · locked ' + vision['label'] if vision.get('locked') else '')
            if link.state == 'error' and not link.running():
                state, text = 'error', 'Stopped with an error: ' + (link.tail or 'see log')
            self.light(name, state, text)
        if m:
            lines = []
            if rescue:
                flags = rescue.get('flags', {})
                lines.append('Mission: {}   active: {}'.format(rescue.get('mission_id'), rescue.get('active_operation')))
                lines.append('Ready: ' + ' '.join('{}={}'.format(k, 'Y' if flags.get(k) else 'n') for k in (
                    'VehicleHealthy', 'FlightAvailable', 'VisionHealthy', 'TargetLocked', 'PeersFresh', 'PayloadAvailable')))
                for tag in (rescue.get('geotags') or [])[-5:]:
                    lines.append('Geotag {}  {:.7f}, {:.7f}  ±{} m'.format(tag['label'], tag['latitude'], tag['longitude'], tag['sigma_m']))
            if route:
                lines.append('Route: {}'.format(route.get('origin_error') or 'origin {}'.format(route.get('origin'))))
            if vision and vision.get('gimbal'):
                g = vision['gimbal']
                lines.append('Gimbal yaw {:.1f}  pitch {:.1f}'.format(g.get('yaw', 0), g.get('pitch', 0)))
            self.status_text.set('\n'.join(lines) or 'Waiting for AERIXIS topics…')
        self.after(1000, self.refresh)


def main():
    if not Path(ROS_SETUP).is_file():
        raise SystemExit('ROS 2 not found at ' + ROS_SETUP)
    Launcher().mainloop()


if __name__ == '__main__':
    main()

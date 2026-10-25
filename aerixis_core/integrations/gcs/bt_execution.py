"""Structured BT execution for the local GCS; no flight commands on import."""

import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import threading
import uuid
import xml.etree.ElementTree as ET

PREFIX = 'AS2_BT_EVENT '
OUTCOMES = {'SUCCESS': 0, 'FAILURE': 1, 'CANCELLED': 2, 'ERROR': 3}


class EventState:
    """Only explicit, identified executor events establish mission outcomes."""

    def __init__(self, mission_id):
        self.mission_id = mission_id
        self.run_id = None
        self.sequence = 0
        self.nodes = {}
        self.outcome = None
        self.detail = ''

    def accept(self, line):
        # The executable is launched directly: prefixed launch/ROS log text is not an event.
        if not line.startswith(PREFIX) or self.outcome is not None:
            return False
        try:
            event = json.loads(line[len(PREFIX):])
            if not isinstance(event, dict):
                return False
            sequence = event['sequence']
            run_id = event['run_id']
            if (event.get('schema_version') != 1 or
                    event.get('mission_id') != self.mission_id or
                    not isinstance(run_id, str) or not run_id or
                    type(sequence) is not int or sequence <= self.sequence or
                    self.run_id not in (None, run_id)):
                return False
            kind = event['event']
            if kind == 'started':
                if self.run_id is not None:
                    return False
                nodes = {}
                for node in event['nodes']:
                    uid = node['node_uid']
                    if type(uid) is not int or uid in nodes:
                        return False
                    if not all(isinstance(node[k], str) for k in
                               ('node_type', 'node_name', 'node_path')):
                        return False
                    nodes[uid] = dict(node, status='IDLE', display_status='pending')
                self.nodes = nodes
            elif kind == 'node_status':
                uid = event['node_uid']
                status = event['status']
                if (uid not in self.nodes or status not in
                        ('IDLE', 'RUNNING', 'SUCCESS', 'FAILURE', 'SKIPPED') or
                        event['node_path'] != self.nodes[uid]['node_path']):
                    return False
                self.nodes[uid]['status'] = status
                # Preserve the last result when the library resets a finished child to IDLE.
                if status != 'IDLE':
                    self.nodes[uid]['display_status'] = {
                        'RUNNING': 'running', 'SUCCESS': 'success',
                        'FAILURE': 'failed', 'SKIPPED': 'skipped'}[status]
            elif kind == 'finished':
                outcome = event['outcome']
                if outcome not in OUTCOMES or event['exit_code'] != OUTCOMES[outcome]:
                    return False
                self.outcome = outcome
                self.detail = str(event.get('detail', ''))
                for node in self.nodes.values():
                    if node['display_status'] == 'running':
                        node['display_status'] = 'cancelled' if outcome == 'CANCELLED' else 'unknown'
            else:
                return False
            self.run_id = run_id
            self.sequence = sequence
            return True
        except (ValueError, KeyError, TypeError):
            return False

    def finish_process(self, returncode):
        if self.outcome is None or OUTCOMES[self.outcome] != returncode:
            self.outcome = 'ERROR'
            self.detail = f'Missing or inconsistent executor result (process exit {returncode})'
        for node in self.nodes.values():
            if node['display_status'] == 'running':
                node['display_status'] = 'unknown'

    def steps(self):
        return [dict(id=f"node_{uid}", type=n['node_type'], name=n['node_name'],
                     status=n['display_status'], node_uid=uid, node_path=n['node_path'],
                     bt_status=n['status']) for uid, n in self.nodes.items()]


class LocalTreeRun:
    """One local execution attempt. Remote upload belongs to the onboard executor phase."""

    def __init__(self, drone_id, xml, workspace='/home/prince/aerostack2'):
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', drone_id):
            raise ValueError('Drone ID must be a single ROS namespace identifier')
        if len(xml.encode('utf-8')) > 1_000_000:
            raise ValueError('Behavior tree exceeds 1 MB')
        root = ET.fromstring(xml)
        if root.tag != 'root' or not root.findall('BehaviorTree'):
            raise ValueError('XML must contain a root and a BehaviorTree definition')
        self.drone_id = drone_id
        self.mission_id = uuid.uuid4().hex
        self.directory = Path(tempfile.mkdtemp(prefix=f'as2_bt_{drone_id}_'))
        self.tree_path = self.directory / 'mission.xml'
        self.tree_path.write_text(xml, encoding='utf-8')
        self.workspace = Path(workspace)
        self.state = EventState(self.mission_id)
        self.lock = threading.Lock()
        self.process = None
        self.cancel_requested = False

    def start(self):
        executable = self.workspace / 'install/as2_behavior_tree/lib/as2_behavior_tree/as2_behavior_tree_node'
        command = ['bash', '-c', 'source "$1" && source "$2" && shift 2 && exec "$@"',
                   'as2-bt', '/opt/ros/humble/setup.bash',
                   str(self.workspace / 'install/setup.bash'), str(executable),
                   '--ros-args', '-r', f'__ns:=/{self.drone_id}',
                   '-p', f'tree:={self.tree_path}', '-p', f'mission_id:={self.mission_id}']
        with self.lock:
            if self.cancel_requested:
                raise RuntimeError('Mission cancelled before process start')
            self.process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT, text=True)
        return self.process

    def cancel(self):
        with self.lock:
            self.cancel_requested = True
            if self.process is not None and self.process.poll() is None:
                # A graceful request: never infer a physical stop from this signal.
                try:
                    self.process.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass

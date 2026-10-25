"""Run PX4/Gazebo SITL and forward its warning/error output to the GCS console."""

import re
import socket
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO


def _workspace_root():
    prefix = Path(get_package_prefix('as2_mavlink_gateway')).resolve()
    for parent in (prefix, *prefix.parents):
        if (parent / 'px4_sitl' / 'run_px4_sitl.sh').is_file():
            return parent
    raise RuntimeError('Cannot find px4_sitl/run_px4_sitl.sh in this workspace.')


def generate_launch_description():
    workspace = _workspace_root()
    sender = None
    destination = ('127.0.0.1', 14552)
    severity_pattern = re.compile(r'\b(error|fatal|failed|failure|exception|warning|warn)\b', re.I)

    def forward_output(event):
        nonlocal sender
        text = event.text.decode('utf-8', errors='replace')
        for line in text.splitlines():
            match = severity_pattern.search(line)
            if not match:
                continue
            if sender is None:
                sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            level = 'WARN' if match.group(1).lower() in ('warning', 'warn') else 'ERROR'
            packet = f'{level}|PX4 SITL|{line.strip()}'.encode('utf-8', errors='replace')[:3500]
            try:
                sender.sendto(packet, destination)
            except OSError:
                pass
        return []

    def report_exit(event, _context):
        nonlocal sender
        return_code = int(event.returncode)
        if return_code not in (0, 130, -2, -15, 143):
            if sender is None:
                sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            packet = (f'ERROR|PX4 SITL|PX4/Gazebo process exited with code '
                      f'{return_code}').encode()
            try:
                sender.sendto(packet, destination)
            except OSError:
                pass
        return []

    px4 = ExecuteProcess(
        cmd=['bash', str(workspace / 'px4_sitl' / 'run_px4_sitl.sh')],
        cwd=str(workspace), output='screen', emulate_tty=True,
        name='px4_sitl_and_gazebo')

    return LaunchDescription([
        RegisterEventHandler(OnProcessIO(
            target_action=px4, on_stdout=forward_output, on_stderr=forward_output)),
        RegisterEventHandler(OnProcessExit(target_action=px4, on_exit=report_exit)),
        px4,
    ])

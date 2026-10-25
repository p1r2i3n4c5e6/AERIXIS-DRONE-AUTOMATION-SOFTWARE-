"""Find AERIXIS onboard panels (http://<onboard-computer>:8090) on this computer's networks.

Used by the GCS "Onboard calibration" page so it opens the drone's panel without typing an IP:
the onboard computer's address changes with every hotspot / Wi-Fi it joins. Only a network probe,
no file link with the onboard software: each /24 this computer is on is tried on port 8090, and a
host counts as a panel when its /api/state answers with the panel's own fields (drone_id, web_port).
"""
import ipaddress
import json
import socket
import subprocess
import threading
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor

PORT = 8090
CACHE_S = 15.0
_lock = threading.Lock()
_cache = {'at': 0.0, 'panels': []}


def local_networks():
    """The /24 around each IPv4 address of a real interface (no loopback / docker bridges)."""
    try:
        out = subprocess.run(['ip', '-4', '-o', 'addr', 'show', 'up'], capture_output=True, text=True,
                             timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    nets = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 4 or parts[1] == 'lo' or parts[1].startswith(('docker', 'br-', 'veth', 'virbr')):
            continue
        iface = ipaddress.ip_interface(parts[3])
        if iface.ip.is_loopback or iface.ip.is_link_local:
            continue
        # Bounded scan: only the /24 around the address, even on a wider network.
        net = ipaddress.ip_network('{}/24'.format(iface.ip), strict=False)
        nets.append((net, iface.ip))
    return nets


def _port_open(ip, timeout=0.5):
    try:
        with socket.create_connection((str(ip), PORT), timeout=timeout):
            return True
    except OSError:
        return False


def _panel(ip):
    try:
        with urllib.request.urlopen('http://{}:{}/api/state'.format(ip, PORT), timeout=2.5) as r:
            data = json.loads(r.read(200000).decode('utf-8', 'replace'))
    except (OSError, ValueError):
        return None
    if not isinstance(data, dict) or 'drone_id' not in data or 'web_port' not in data:
        return None
    vehicle = data.get('vehicle') or {}
    return {'address': str(ip), 'drone_id': str(data.get('drone_id') or ''),
            'hostname': str(data.get('hostname') or ''), 'fc_connected': bool(vehicle.get('connected'))}


def scan(hints=()):
    """Probe hint addresses first (last known), then every host of the local /24s."""
    own = set()
    candidates = []
    for net, ip in local_networks():
        own.add(ip)
        candidates += [h for h in net.hosts()]
    ordered, seen = [], set()
    for host in [h for h in hints if h] + candidates:
        try:
            ip = ipaddress.ip_address(str(host).split(':')[0])
        except ValueError:
            continue
        if ip in seen or ip in own:
            continue
        seen.add(ip)
        ordered.append(ip)
    panels = []
    # Second pass with a longer timeout: the first connects to a cold network can be lost while
    # ARP resolves (seen on a phone hotspot, 2026-10-25).
    for timeout in (0.5, 1.2):
        with ThreadPoolExecutor(max_workers=128) as pool:
            open_hosts = [ip for ip, ok in zip(ordered, pool.map(lambda h: _port_open(h, timeout), ordered)) if ok]
            panels = [p for p in pool.map(_panel, open_hosts) if p]
        if panels:
            break
    return panels


def discover(fresh=False, hints=()):
    with _lock:
        if not fresh and time.monotonic() - _cache['at'] < CACHE_S:
            return list(_cache['panels'])
        panels = scan(hints)
        _cache.update(at=time.monotonic(), panels=panels)
        return list(panels)

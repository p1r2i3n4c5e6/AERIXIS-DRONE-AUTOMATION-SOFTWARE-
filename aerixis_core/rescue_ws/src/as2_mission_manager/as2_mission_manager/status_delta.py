"""Radio-sized status: only what changed since the poller's cursors.

A full snapshot is ~55 KB (10 KB compressed): 200 log lines, the whole tree structure and the
ROS node list. A 2.4 GHz telemetry radio cannot carry that every second for every aircraft.
The lite form sends new log lines only, the tree structure once per run (afterwards two
characters per node), the node list only when it changes, and only the raised rescue flags.
The GCS merges it back into the full shape, so its pages see the same data.
"""
import hashlib
import json

LOG_BURST = 30
STATUS = {'IDLE': 'I', 'RUNNING': 'R', 'SUCCESS': 'S', 'FAILURE': 'F', 'SKIPPED': 'K'}
DISPLAY = {'pending': 'p', 'running': 'r', 'success': 's', 'failed': 'f', 'skipped': 'k',
           'cancelled': 'c', 'unknown': 'u'}


def _key(value):
    return hashlib.sha1(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()[:12]


def bt_key(nodes):
    return _key([[n['node_uid'], n['node_type'], n['node_name'], n.get('children_uids', [])] for n in nodes])


def bt_codes(nodes):
    return ''.join(STATUS.get(n['status'], 'I') + DISPLAY.get(n.get('display_status'), 'p')
                   for n in sorted(nodes, key=lambda n: n['node_uid']))


PRECISE = ('latitude', 'longitude')
STATIC = ('mission', 'execution', 'registry_hash', 'execution_location')


def trim(value, key=''):
    """Round floats: 7 decimals for latitude/longitude, 3 elsewhere (mm, ms, 0.001 deg)."""
    if isinstance(value, float):
        return round(value, 7 if key in PRECISE else 3)
    if isinstance(value, dict):
        return {k: trim(v, k) for k, v in value.items()}
    if isinstance(value, list):
        return [trim(v, key) for v in value]
    return value


def compact(state, query):
    out = {k: v for k, v in state.items() if k not in ('logs', 'bt_nodes', 'ros', 'registry_counts')}
    out['lite'] = True
    if out.get('mission'):
        out['mission'] = {k: v for k, v in out['mission'].items() if k not in ('nodes', 'node_types')}
    logs = state.get('logs') or []
    ids = [entry['id'] for entry in logs]
    after = query.get('log', '')
    fresh = logs[ids.index(after) + 1:] if after in ids else logs
    out['logs'] = [dict(entry, message=str(entry['message'])[:240]) for entry in fresh[-LOG_BURST:]]
    out['log_cursor'] = ids[-1] if ids else ''
    nodes = state.get('bt_nodes') or []
    out['bt_key'] = bt_key(nodes) if nodes else ''
    if nodes and query.get('bt') == out['bt_key']:
        out['bt_status'] = bt_codes(nodes)
    else:
        out['bt_nodes'] = nodes
    ros = dict(state.get('ros') or {})
    ros['nodes_key'] = _key(ros.get('nodes'))
    if query.get('ros') == ros['nodes_key']:
        ros.pop('nodes', None)
    rescue = ros.get('rescue') or {}
    data = rescue.get('data')
    if isinstance(data, dict):
        data = dict(data)
        # Flags are booleans; a missing flag reads as false on the GCS.
        data['flags'] = {k: v for k, v in (data.get('flags') or {}).items() if v not in (False, None, '')}
        payload = data.get('payload') or {}
        data['payload'] = {k: payload.get(k) for k in
                           ('truth', 'released', 'transaction_state', 'operator_confirmed', 'payload_id')}
        ros['rescue'] = dict(rescue, data=data)
    out['ros'] = ros
    # Mission/execution records change only at stage/start/finish: send them when they do.
    out['static_key'] = _key({k: state.get(k) for k in STATIC})
    if query.get('st') == out['static_key']:
        for k in STATIC:
            out.pop(k, None)
    return trim(out)

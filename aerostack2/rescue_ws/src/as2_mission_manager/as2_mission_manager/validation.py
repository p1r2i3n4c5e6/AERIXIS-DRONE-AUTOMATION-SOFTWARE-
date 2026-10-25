"""Conservative XML checks against the installed executor's exported registry."""
import hashlib
import math
import re
import xml.etree.ElementTree as ET

IDENTIFIER = re.compile(r'[A-Za-z_][A-Za-z0-9_]{0,63}')
OBSERVATION_ACTIONS = {'AlwaysSuccess', 'AlwaysFailure', 'Echo', 'Sleep'}
OPTIONAL_PORTS = {'server_name', 'server_timeout'}


def validate_xml(xml, registry, max_speed=5.0, max_height=50.0):
    if not isinstance(xml, str) or not 0 < len(xml.encode()) <= 1_000_000:
        raise ValueError('XML must contain between 1 and 1,000,000 bytes')
    if '<!DOCTYPE' in xml.upper() or '<!ENTITY' in xml.upper():
        raise ValueError('XML entities and external documents are not supported')
    try:
        root = ET.fromstring(xml)
    except ET.ParseError as exc:
        raise ValueError(f'Invalid mission XML: {exc}') from None
    if root.tag != 'root' or root.get('BTCPP_format') != '4':
        raise ValueError('A BehaviorTree.CPP v4 root is required')
    if any(e.tag not in {'BehaviorTree','TreeNodesModel'} for e in root):
        raise ValueError('Only embedded trees and node models are allowed; no external includes')
    trees = {}
    for tree in root.findall('BehaviorTree'):
        key = tree.get('ID')
        if not key or key in trees or len(tree) != 1:
            raise ValueError('Tree IDs must be unique and each tree must have one root node')
        trees[key] = tree
    main = root.get('main_tree_to_execute') or next(iter(trees), '')
    if main not in trees:
        raise ValueError('Main tree definition is missing')
    nodes, used, flight = [], set(), False

    def visit(element, path, stack, depth):
        nonlocal flight
        if depth > 40 or len(nodes) >= 500:
            raise ValueError('Tree exceeds the supported depth or 500 expanded nodes')
        kind = element.get('ID') if element.tag in ('Action','Condition','Decorator','Control','SubTree') else element.tag
        if element.tag == 'SubTree':
            if len(element) or any(k not in ('ID','name') for k in element.attrib):
                raise ValueError('SubTree remapping and scripting are not supported')
            if kind not in trees or kind in stack:
                raise ValueError('Missing or recursive SubTree')
            nodes.append({'path':path,'type':'SubTree','name':kind,'category':'SubTree'})
            visit(trees[kind][0], path+'/0', stack+(kind,), depth+1)
            return
        if kind not in registry:
            raise ValueError(f'Node is not registered onboard: {kind}')
        if kind in {'Script', 'ScriptCondition', 'Precondition'} or any(k.startswith('_') for k in element.attrib):
            raise ValueError('Script nodes and scripting attributes are not enabled in this mission profile')
        manifest = registry[kind]
        category, ports = manifest['category'], manifest['ports']
        if category in ('Action','Condition') and len(element):
            raise ValueError(f'{kind} cannot have children')
        if category == 'Decorator' and len(element) != 1:
            raise ValueError(f'{kind} requires one child')
        if category == 'Control' and not len(element):
            raise ValueError(f'{kind} requires children')
        if kind == 'RecoverThenFail' and len(element) != 2:
            raise ValueError('RecoverThenFail requires task and recovery children')
        if kind == 'IfThenElse' and len(element) not in (2,3):
            raise ValueError('IfThenElse requires two or three children')
        if kind == 'WhileDoElse' and len(element) not in (2,3):
            raise ValueError('WhileDoElse requires two or three children')
        for name, info in ports.items():
            # PortDirection INPUT=0, OUTPUT=1, INOUT=2 in the installed BT library.
            if (info['direction'] == 0 and not info['has_default'] and
                    name not in element.attrib and name not in OPTIONAL_PORTS):
                raise ValueError(f'{kind} is missing input: {name}')
        for name, value in element.attrib.items():
            if name in ('ID','name'):
                continue
            if name not in ports:
                raise ValueError(f'Unknown port {name} on {kind}')
            if '{' in value or '}' in value:
                raise ValueError('Dynamic blackboard remapping is not yet supported by the onboard validator')
            typename = ports[name]['type'].lower()
            if typename in ('double','float','int','unsigned int','long','unsigned long'):
                number = float(value)
                if not math.isfinite(number):
                    raise ValueError(f'{kind}.{name} must be finite')
                if typename.startswith('unsigned') and number < 0:
                    raise ValueError(f'{kind}.{name} must be nonnegative')
                if 'int' in typename or 'long' in typename:
                    if number != int(number):
                        raise ValueError(f'{kind}.{name} requires an integer')
                if name in ('speed','max_speed') and not 0 < number <= max_speed:
                    raise ValueError(f'{kind}.{name} exceeds the configured speed envelope')
                if name == 'height' and not 0 < number <= max_height:
                    raise ValueError('Takeoff height exceeds the configured envelope')
            if kind in ('GoTo','GoToGps') and name == 'pose':
                coords = [float(x) for x in value.split(';')]
                if len(coords) != 3 or not all(math.isfinite(x) for x in coords) or not 0 < coords[2] <= max_height:
                    raise ValueError('Waypoint needs three finite coordinates and a permitted altitude')
        if kind == 'FollowPath':
            raise ValueError('Typed FollowPath input is not yet supported; use explicit GoTo waypoints')
        flight |= category == 'Action' and kind not in OBSERVATION_ACTIONS
        used.add(kind)
        nodes.append({'path':path,'type':kind,'name':element.get('name',kind),'category':category})
        for index, child in enumerate(element):
            visit(child, f'{path}/{index}', stack, depth+1)
    visit(trees[main][0], main+'/0', (main,), 0)
    return {'sha256':hashlib.sha256(xml.encode()).hexdigest(),'nodes':nodes,
            'requires_flight':flight,'node_types':sorted(used), 'node_count':len(nodes)}

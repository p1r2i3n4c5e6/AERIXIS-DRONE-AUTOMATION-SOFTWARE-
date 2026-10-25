"""Shared KML -> reviewable per-aircraft XML and immutable route bundles."""
import hashlib
import json
import math
from . import bundle, coverage, kml, watch

MODES={'survey':'surveillance','waypoints':'waypoints','search_geotag':'search_geotag',
       'rescue_delivery':'rescue_delivery','waypoint_delivery':'waypoint_delivery',
       'swarm_search':'swarm_search','swarm_delivery':'swarm_delivery',
       'human_track_rtl':'human_track_rtl','human_orbit':'human_orbit','human_geotag_resume':'human_geotag_resume',
       'human_geotag_drop_resume':'human_geotag_drop_resume','swarm_coverage':'swarm_coverage',
       'custom':'custom'}
DEFAULT_TEMPLATE='human_geotag_resume'
VISION_OPTIONS=('search_pitch_deg','lost_timeout_s','track_duration_s','orbit_radius_m','orbit_laps',
                'orbit_speed','approach_speed','rtl_speed','suppress_s','mesh_wait_s','search_time_s','scan_rate_dps')
MAX_AREA_VERTICES=40


def search_area(shell):
    """KML shell for the onboard detection filter, thinned to a radio-friendly vertex count."""
    ring=[list(p[:2]) for p in shell]
    if len(ring)>1 and ring[0]==ring[-1]:ring=ring[:-1]
    if len(ring)>MAX_AREA_VERTICES:
        step=len(ring)/MAX_AREA_VERTICES;ring=[ring[int(i*step)] for i in range(MAX_AREA_VERTICES)]
    return ring


def station_for(region,waypoints):
    """Watch point for one aircraft: the middle of its own share of the area, kept inside."""
    xs=[w['pose'][0] for w in waypoints];ys=[w['pose'][1] for w in waypoints]
    centre=[sum(xs)/len(xs),sum(ys)/len(ys)]
    if region.contains(centre):return centre
    return min((w['pose'][:2] for w in waypoints),key=lambda p:math.hypot(p[0]-centre[0],p[1]-centre[1]))

def plan_mission(request):
    # Trees are owned by this package (trees.py) so the planner depends on
    # nothing from the runtime, keeping the package graph acyclic.
    from .trees import mission, vision_mission, VISION_MODES, END_ACTIONS, xml
    from .features import custom_mission, feature_spec
    document=request.get('kml')
    if not isinstance(document,str) or len(document.encode())>1_000_000:
        raise ValueError('KML text must be at most 1 MB')
    identity=request.get('mission_id');revision=request.get('revision',1)
    if not isinstance(revision,int) or isinstance(revision,bool) or revision<1:
        raise ValueError('Revision must be a positive integer')
    mode=request.get('template') or DEFAULT_TEMPLATE
    if mode not in MODES:raise ValueError('Unknown mission template')
    # Custom missions (GCS mission builder): a feature spec composed into a tree by features.py.
    spec=feature_spec(request.get('features')) if mode=='custom' else None
    emergency=request.get('emergency_action') or (spec['emergency'] if spec else 'RTL')
    if emergency not in END_ACTIONS:raise ValueError('Emergency action must be RTL or LAND')
    if spec:spec['emergency']=emergency
    watching=mode=='human_orbit' or bool(spec and spec['type']=='watch')
    dropping=mode=='human_geotag_drop_resume' or bool(spec and spec['drop'])
    speed=float(request.get('speed_mps',.5));altitude=float(request.get('altitude_m',0))
    if not math.isfinite(speed) or not 0<speed<=2:raise ValueError('Managed flight speed must be in (0, 2] m/s')
    if not math.isfinite(altitude) or not 1<=altitude<=50:raise ValueError('Explicit common-frame altitude 1..50 m required')
    parsed=kml.parse_kml(document);areas=parsed.areas
    if request.get('area'):areas=[a for a in areas if a.name==request['area']]
    if len(areas)!=1:raise ValueError('Select exactly one KML planning area by name')
    region=parsed.region(areas[0]);fleet=request.get('fleet')
    if not isinstance(fleet,list) or not 1<=len(fleet)<=64:raise ValueError('Fleet must contain 1..64 aircraft')
    plan=coverage.partition_region(region,fleet,float(request.get('spacing_m',10)),
        float(request.get('heading_deg',0)),speed,float(request.get('corridor_m',10)))
    region_data=dict(shell=region.shell,holes=region.holes,exclusions=region.exclusions,name=region.name)
    origin=dict(latitude=parsed.projection.latitude,longitude=parsed.projection.longitude,
                altitude=0.,altitude_verified=False)
    built=bundle.build_bundles(plan,identity,revision,altitude,request.get('capabilities'))
    xmls={};reasons=[]
    if not plan['complete']:reasons.append(plan['reason'])
    if any(not isinstance(a.get('endurance_s'),(int,float)) or a['endurance_s']<=0 for a in fleet):
        reasons.append('All aircraft need a positive measured endurance budget')
    if len(plan['assignments'])!=len(fleet):reasons.append('Not enough coverage lanes for every selected aircraft')
    # Routes drawn in the KML (a LineString named after each aircraft): flown instead of the
    # computed lanes, e.g. to test or fly crossing paths. Every selected aircraft needs one.
    drawn={item['drone_id']:parsed.route(item['drone_id']) for item in built['bundles']}
    use_routes=bool(parsed.routes) and all(drawn.values())
    if parsed.routes and not use_routes:
        reasons.append('Drawn routes (%s) must name every selected aircraft (%s)'%(', '.join(sorted(parsed.routes)),
            ', '.join(sorted(drawn))))
    if use_routes and watching:
        reasons.append('Drawn routes are flown by coverage missions, not by watch stations')
    if use_routes:
        for item in built['bundles']:
            own=item['drone_id'];points=drawn[own]
            outside=[k for k,p in enumerate(points) if not region.contains(p)]
            if outside:reasons.append('%s: drawn route point(s) %s outside the area'%(own,', '.join(map(str,outside))))
            item['coverage']['waypoints']=[dict(segment_id='%s_route_%d'%(own,k),
                pose=[round(float(x),3),round(float(y),3),float(altitude)]) for k,(x,y) in enumerate(points)]
            length=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
            item['assignment']=dict(item['assignment'],pattern='route',distance_m=round(length,1),
                estimated_duration_s=round(length/speed,1))
    watch_lists={}
    if watching:
        # Mission 2 watch stations (planned + alternates the runtime may move to, watch.py).
        planned={b['drone_id']:station_for(region,b['coverage']['waypoints']) for b in built['bundles']}
        for item in built['bundles']:
            own=item['drone_id']
            watch_lists[own]=watch.encode(watch.alternates(region,item['coverage']['waypoints'],planned[own]),
                {d:p for d,p in planned.items() if d!=own})
    for item in built['bundles']:
        if watching:
            # Mission 2 does not survey: each aircraft holds one station in its share of the
            # area and watches. The single leg is the entry transit from launch.
            station=planned[item['drone_id']]
            item['coverage']['waypoints']=[dict(segment_id=item['drone_id']+watch.STATION_SUFFIX,
                pose=[round(float(station[0]),3),round(float(station[1]),3),float(altitude)])]
            item['assignment']=dict(item['assignment'],pattern='station',distance_m=0.0,
                estimated_duration_s=float(request.get('search_time_s') or 600))
        points=[w['pose'] for w in item['coverage']['waypoints']]
        if any(not region.contains_segment(a[:2],b[:2]) for a,b in zip(points,points[1:])):
            reasons.append(item['drone_id']+': a transit crosses an excluded/outside area; change partition or provide a safe corridor')
        segments=[w['segment_id'] for w in item['coverage']['waypoints']]
        if mode in VISION_MODES or spec:
            options={}
            for key in VISION_OPTIONS:
                if request.get(key) is not None:
                    value=float(request[key])
                    if not math.isfinite(value):raise ValueError(key+' must be finite')
                    options[key]=value
            common=dict(segment_ids=segments,speed=speed,fleet_size=len(fleet),area=search_area(region.shell),
                stations=watch_lists.get(item['drone_id']),**options)
            if spec:tree=custom_mission(spec,points,request.get('swarm_id') or identity,**common)
            else:tree=vision_mission(mode,points,request.get('swarm_id') or identity,emergency=emergency,**common)
        else:
            tree=mission(MODES[mode],points,request.get('swarm_id') or identity,
                request.get('target_id') or 'delivery-zone-1',segment_ids=segments,speed=speed,
                payload_confirmation=request.get('payload_confirmation','sensors'))
        xmls[item['drone_id']]=xml(tree)
        drop_mode=request.get('drop_mode','overhead' if dropping else 'beside')
        if drop_mode not in ('beside','overhead'):raise ValueError('Unknown drop geometry')
        if drop_mode=='overhead' and request.get('overhead_authorized') is not True and not dropping:
            raise ValueError('Direct-over-person delivery needs explicit mission opt-in')
        item.update(region=region_data,origin=origin,xml_sha256=hashlib.sha256(xmls[item['drone_id']].encode()).hexdigest(),
            delivery_policy=dict(drop_mode=drop_mode,overhead_authorized=request.get('overhead_authorized') is True or dropping,
                planned_waypoint=mode=='waypoint_delivery',target_id=request.get('target_id') or 'delivery-zone-1'))
        item['bundle_sha256']=bundle.body_checksum({k:v for k,v in item.items() if k!='bundle_sha256'})
    if not use_routes and not coverage.separation_ok(plan['assignments'],plan['corridor_m']):
        reasons.append('Assigned paths do not meet the requested geometric separation')
    result=dict(schema_version=1,mission_id=identity,revision=revision,area=areas[0].name,template=mode,
        altitude=kml.altitude_semantics(areas[0]),origin=origin,region=region_data,
        plan=plan,manifest=built['manifest'],bundles=built['bundles'],xml=xmls,
        complete=not reasons,reason='; '.join(reasons),features=spec,emergency_action=emergency,
        routes='kml' if use_routes else None,
        stations={b['drone_id']:b['coverage']['waypoints'][0]['pose'] for b in built['bundles']} if watching else None)
    result['manifest']['artifacts']=[dict(drone_id=b['drone_id'],bundle_sha256=b['bundle_sha256'],xml_sha256=b['xml_sha256']) for b in built['bundles']]
    result['manifest']['origin']=origin
    return result

def validate_route_bundle(value, drone_id, xml_text):
    """Bind namespace, exact XML, complete leg identities and geometry at staging."""
    import xml.etree.ElementTree as ET
    from .geometry import PlannerRegion
    bundle.verify_bundle(value)
    if value.get('drone_id')!=drone_id:raise ValueError('Route bundle belongs to another aircraft')
    if value.get('xml_sha256')!=hashlib.sha256(xml_text.encode()).hexdigest():raise ValueError('Route bundle does not match XML')
    data=value['region'];region=PlannerRegion(data['shell'],data.get('holes'),data.get('exclusions'))
    waypoints=value['coverage']['waypoints'];by_id={w['segment_id']:w for w in waypoints}
    if len(by_id)!=len(waypoints):raise ValueError('Duplicate route segment ID')
    nodes=list(ET.fromstring(xml_text).iter('SurveyWaypoint'))
    if len(nodes)!=len(waypoints):raise ValueError('XML coverage differs from route bundle')
    for node,w in zip(nodes,waypoints):
        if node.get('segment_id')!=w['segment_id'] or [float(x) for x in node.get('pose','').split(';')]!=w['pose']:
            raise ValueError('XML segment identity/pose differs from route bundle')
    points=[w['pose'] for w in waypoints]
    if any(len(p)!=3 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in p) or not 1<=p[2]<=50 for p in points):
        raise ValueError('Invalid route altitude or coordinates')
    if any(not region.contains_segment(a[:2],b[:2]) for a,b in zip(points,points[1:])):
        raise ValueError('A route transit leaves the approved region')
    # Mission 2 watch: ScanArea's planned station is a staged waypoint and every station it may
    # move the watch to lies inside the region (route_node approves only these goals).
    from .watch import watch_goals
    for node in ET.fromstring(xml_text).iter('ScanArea'):
        if node.get('pose') and [float(x) for x in node.get('pose').split(';')] not in points:
            raise ValueError('ScanArea station differs from the staged route')
    if any(not region.contains(goal[:2]) for goal in watch_goals(xml_text)):
        raise ValueError('A watch station lies outside the approved region')
    return region

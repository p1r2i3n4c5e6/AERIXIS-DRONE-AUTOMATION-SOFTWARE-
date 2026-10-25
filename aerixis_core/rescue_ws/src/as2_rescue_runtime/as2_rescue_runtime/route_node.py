"""Approve the exact requested segment from an immutable onboard-staged bundle."""
import hashlib
import json
import math
from pathlib import Path
import re
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from as2_msgs.msg import PlatformInfo
from as2_msgs.srv import GetOrigin, SetOrigin
from as2_rescue_planner.mission import validate_route_bundle
from as2_rescue_planner.watch import watch_goals
from .node_main import run as run_node

ORIGIN_REPLY_TIMEOUT_S=5.0
# The origin is set only from a steady, plausible fix: SITL 2026-10-23 set one PX4 instance's origin
# from a fix at -232 673 m altitude, and its estimator never produced a pose.
GPS_STEADY_S=3.0
GPS_MIN_FIXES=5
GPS_SPREAD_M=5.0
GPS_ALTITUDE_RANGE=(-500.0,9000.0)


def steady_fix(history,now):
    """(newest fix, '') when the last GPS_STEADY_S of fixes are valid and agree, else (None, why)."""
    recent=[(t,m) for t,m in history if 0<=now-t<=GPS_STEADY_S]
    if not recent:return None,'Waiting for a GNSS fix before setting the origin'
    fixes=[m for _,m in recent]
    for m in fixes:
        if m.status.status<0:return None,'Waiting for a GNSS fix before setting the origin'
        if not all(math.isfinite(v) for v in (m.latitude,m.longitude,m.altitude)):
            return None,'GNSS fix has non-finite values; origin not set'
        if abs(m.latitude)>90 or abs(m.longitude)>180 or not GPS_ALTITUDE_RANGE[0]<=m.altitude<=GPS_ALTITUDE_RANGE[1]:
            return None,'Implausible GNSS fix (altitude %.0f m); origin not set'%m.altitude
    if len(fixes)<GPS_MIN_FIXES:return None,'Waiting for a steady GNSS fix (%d of %d fixes)'%(len(fixes),GPS_MIN_FIXES)
    lat0=fixes[-1].latitude
    north=[(m.latitude-lat0)*111320.0 for m in fixes]
    east=[(m.longitude-fixes[-1].longitude)*111320.0*math.cos(math.radians(lat0)) for m in fixes]
    alts=[m.altitude for m in fixes]
    spread=max(max(north)-min(north),max(east)-min(east))
    if spread>GPS_SPREAD_M or max(alts)-min(alts)>GPS_SPREAD_M:
        return None,'GNSS fix not steady yet (%.1f m horizontally, %.1f m in altitude over %.0f s)'%(
            spread,max(alts)-min(alts),GPS_STEADY_S)
    return fixes[-1],''

def distance_to_segment(point,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];length=dx*dx+dy*dy
    t=0.0 if length==0 else max(0.0,min(1.0,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length))
    return math.hypot(point[0]-a[0]-t*dx,point[1]-a[1]-t*dy)

def restation_allowed(goal,stations,tolerance=1e-3):
    """A watch re-station leg may only end at a station listed in the staged tree."""
    return any(all(abs(a-b)<=tolerance for a,b in zip(goal,station)) for station in stations)

class RoutePublisher(Node):
    def __init__(self,args=None):
        super().__init__('rescue_route_publisher',cli_args=args)
        defaults=[('bundle_directory',str(Path.home()/'.local/share/as2-missions'/self.get_namespace().strip('/'))),
            ('path_clear_verified',False),('origin_verified',False),('origin_latitude',0.0),
            ('origin_longitude',0.0),('altitude_frame_verified',False),('frame_id','earth'),
            # 'estimator': the AS2 GPS estimator's own origin is the evidence. Before take-off the
            # staged plan's origin is written to it once (set_origin), then compared on every leg.
            ('origin_source','parameters'),
            # Launch points are often outside the search area (a road beside a flooded field):
            # one straight entry leg is allowed if it crosses no hole/exclusion and is short.
            ('allow_entry_transit',True),('max_entry_transit_m',500.0),
            # Once a leg is approved, the aircraft only has to stay inside this corridor
            # around it (lanes end on the boundary; GNSS/controller overshoot is normal).
            ('corridor_tolerance_m',3.0),
            # AI avoidance legs bend around other drones: they may leave the line by this much,
            # but never the planning area (avoidance.py stops pushing outward at 20 m).
            ('avoidance_corridor_m',25.0)]
        for key,value in defaults:self.declare_parameter(key,value)
        self.request=None;self.pose=None;self.cached=None;self.error='';self.legs={}
        self.estimator_origin=None;self.origin_future=None;self.origin_error='';self.gps=None;self.gps_history=[]
        if self.get_parameter('origin_source').value=='estimator':
            self.get_origin=self.create_client(GetOrigin,'get_origin');self.set_origin=self.create_client(SetOrigin,'set_origin')
            self.create_subscription(NavSatFix,'sensor_measurements/gps',self.on_gps,qos_profile_sensor_data)
            self.create_subscription(PlatformInfo,'platform/info',lambda m:setattr(self,'platform',(m,self.now())),10)
            self.create_timer(1.0,self.sync_origin)
        self.publisher=self.create_publisher(String,'rescue/route',10)
        self.health=self.create_publisher(String,'rescue/route_health',10)
        self.create_subscription(String,'rescue/route_request',self.on_request,10)
        self.create_subscription(PoseStamped,'self_localization/pose',self.on_pose,qos_profile_sensor_data)
        self.create_timer(.1,self.publish)
    def now(self):return self.get_clock().now().nanoseconds/1e9
    def staged_origin(self):
        """Origin of the mission currently staged by the onboard mission manager."""
        directory=Path(self.get_parameter('bundle_directory').value)
        # Fresh install: the mission manager has not written state.json yet = no staged plan
        # (raising here used to block the provisional origin, so AS2 had no earth frame at all).
        if not (directory/'state.json').exists():return None
        record=json.loads((directory/'state.json').read_text())
        mission=(record.get('mission') or {}).get('id')
        if not mission or not (directory/(mission+'.bundle.json')).exists():return None
        return self.load(mission)[0]['origin']
    def on_ground(self):
        info=getattr(self,'platform',None)
        return bool(info and self.now()-info[1]<2.0 and not info[0].armed)
    def sync_origin(self):
        """Keep the AERIXIS core earth origin equal to the staged plan's origin.

        Before any plan is staged a provisional origin at the current GNSS fix gives AERIXIS core an
        earth frame (positions, readiness, no TF errors); it can never fly a plan because leg
        approval requires the plan origin. A newly staged plan with another origin replaces
        it, only while the aircraft is disarmed on the ground."""
        if self.origin_future is not None:
            kind,future,target,sent=self.origin_future
            if not future.done():
                # A reply can be lost while DDS discovery settles (seen in SITL: the estimator could
                # not send the get_origin answer); never wait for it forever, ask again.
                if self.now()-sent>ORIGIN_REPLY_TIMEOUT_S:
                    self.origin_future=None;self.origin_error='AERIXIS core estimator did not answer %s_origin; retrying'%kind
                return
            self.origin_future=None;response=future.result()
            if kind=='get':
                self.origin_known=True
                self.estimator_origin=dict(latitude=response.origin.latitude,longitude=response.origin.longitude) if response and response.success else None
            elif response and response.success:self.estimator_origin=target;self.origin_error=''
            else:self.origin_error='Estimator refused set_origin (no GNSS fix yet?)'
            return
        try:
            if not self.get_origin.service_is_ready():self.origin_error='AERIXIS core estimator get_origin unavailable (use_gps must be true)';return
            if not getattr(self,'origin_known',False):
                self.origin_future=('get',self.get_origin.call_async(GetOrigin.Request()),None,self.now());return
            staged=self.staged_origin();current=self.estimator_origin
            same=lambda a,b:abs(a['latitude']-b['latitude'])<=1e-7 and abs(a['longitude']-b['longitude'])<=1e-7
            if staged is None and current is not None:self.origin_error='';return
            if staged is not None and current is not None and same(staged,current):self.origin_error='';return
            fix,why=steady_fix(self.gps_history,self.now())
            if fix is None:self.origin_error=why;return
            if staged is None:
                target=dict(latitude=float(fix.latitude),longitude=float(fix.longitude))
                self.get_logger().info('No staged plan yet: provisional AERIXIS core earth origin at the current GNSS fix')
            else:
                if current is not None and not self.on_ground():
                    self.origin_error='Staged plan has another origin; it is applied once the aircraft is disarmed on the ground';return
                target=dict(latitude=float(staged['latitude']),longitude=float(staged['longitude']))
                self.get_logger().info('Setting AERIXIS core earth origin to staged plan origin %.8f, %.8f'%(target['latitude'],target['longitude']))
            request=SetOrigin.Request();request.origin.latitude=target['latitude']
            request.origin.longitude=target['longitude'];request.origin.altitude=float(fix.altitude)
            self.origin_future=('set',self.set_origin.call_async(request),target,self.now())
        except (OSError,ValueError,KeyError,TypeError) as exc:self.origin_error=str(exc)
    def on_gps(self,message):
        now=self.now();self.gps=message
        self.gps_history=[(t,m) for t,m in self.gps_history if now-t<=GPS_STEADY_S]+[(now,message)]
    def on_pose(self,message):self.pose=message
    def on_request(self,message):
        try:
            request=json.loads(message.data)
            if not 0<=self.now()-request['stamp']<=.5:raise ValueError('Stale route request')
            if request.get('operation') not in ('SurveyWaypoint','ResumeCoverage','ApproachDeliveryZone','Restation'):raise ValueError('Unknown route operation')
            self.request=request
        except (ValueError,KeyError,TypeError):self.request=None
    def load(self,mission):
        if not isinstance(mission,str) or not re.fullmatch('[0-9a-f]{32}',mission):
            raise ValueError('Route must name an immutable onboard mission ID')
        directory=Path(self.get_parameter('bundle_directory').value)
        artifact=json.loads((directory/(mission+'.bundle.json')).read_text())
        xml=(directory/(mission+'.xml')).read_text()
        if artifact['mission_id']!=mission or artifact['xml_sha256']!=hashlib.sha256(xml.encode()).hexdigest():
            raise ValueError('Staged bundle/XML binding is invalid')
        value=artifact['bundle']
        region=validate_route_bundle(value,self.get_namespace().strip('/'),xml)
        return value,region,watch_goals(xml)
    def approval(self):
        request=self.request;now=self.now()
        if not request or not 0<=now-request['stamp']<=.5:raise ValueError('No fresh motion request')
        # Read immutable files again on a new command. Never advance on a timer.
        key=(request['mission_id'],request['command_id'])
        if not self.cached or self.cached[0]!=key:self.cached=(key,*self.load(request['mission_id']))
        bundle,region,stations=self.cached[1:];frame=self.get_parameter('frame_id').value
        if frame!=bundle['coverage']['frame_id'] or request['frame_id']!=frame:raise ValueError('Route coordinate frame mismatch')
        origin=bundle['origin']
        if not self.get_parameter('altitude_frame_verified').value:
            raise ValueError('Altitude datum (height above take-off) not verified for this aircraft profile')
        if self.get_parameter('origin_source').value=='estimator':
            measured=self.estimator_origin
            if measured is None:raise ValueError(self.origin_error or 'AERIXIS core estimator origin not set yet')
            if abs(origin['latitude']-measured['latitude'])>1e-7 or abs(origin['longitude']-measured['longitude'])>1e-7:
                raise ValueError(self.origin_error or 'Estimator origin is being switched to the KML plan origin; wait on the ground')
        else:
            if not self.get_parameter('origin_verified').value:
                raise ValueError('Aircraft origin needs verification against this plan')
            if abs(origin['latitude']-self.get_parameter('origin_latitude').value)>1e-8 or abs(origin['longitude']-self.get_parameter('origin_longitude').value)>1e-8:
                raise ValueError('Aircraft and KML planner use different geographic origins')
        if not self.pose:raise ValueError('No measured aircraft position')
        stamp=self.pose.header.stamp.sec+self.pose.header.stamp.nanosec/1e9
        if not 0<=now-stamp<=.5 or self.pose.header.frame_id!=frame:raise ValueError('Measured route origin is stale or in another frame')
        point=self.pose.pose.position;start=[point.x,point.y,point.z];goal=request['target_pose']
        if len(goal)!=3 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in start+goal) or not 1<=goal[2]<=50:
            raise ValueError('Invalid route endpoint')
        if request['operation']=='SurveyWaypoint':
            leg=next((w for w in bundle['coverage']['waypoints'] if w['segment_id']==request['segment_id']),None)
            if not leg or leg['pose']!=goal:raise ValueError('Requested waypoint differs from staged route')
        if request['operation']=='Restation' and not restation_allowed(goal,stations):
            # Mission 2 watch moved away from a peer's orbit: only to a station the staged tree lists.
            raise ValueError('Re-station goal is not a staged watch station')
        leg=self.legs.get((request['mission_id'],request['command_id']))
        if leg is not None:
            if leg[1]!=goal:raise ValueError('Approved leg goal changed')
            tolerance=self.get_parameter('corridor_tolerance_m').value
            avoiding=request.get('avoidance') is True and request['operation'] in ('SurveyWaypoint','ResumeCoverage')
            off=distance_to_segment(start[:2],leg[0][:2],leg[1][:2])
            if off>(max(tolerance,self.get_parameter('avoidance_corridor_m').value) if avoiding else tolerance):
                raise ValueError('Aircraft left the approved leg corridor')
            if avoiding and off>tolerance and not region.contains(start[:2]):
                raise ValueError('Avoidance detour left the planning area')
        elif not region.contains_segment(start[:2],goal[:2]):
            entry=(self.get_parameter('allow_entry_transit').value and request['operation']!='ApproachDeliveryZone'
                   and math.dist(start[:2],goal[:2])<=self.get_parameter('max_entry_transit_m').value
                   and region.contains_entry_segment(start[:2],goal[:2]))
            if not entry:raise ValueError('Requested transit crosses outside/excluded area')
        if leg is None:
            self.legs[(request['mission_id'],request['command_id'])]=(start,goal)
            if len(self.legs)>64:self.legs.pop(next(iter(self.legs)))
        return dict(schema_version=1,stamp=now,mission_id=request['mission_id'],command_id=request['command_id'],
            segment_id=request.get('segment_id',''),frame_id=frame,target_pose=goal,rejoin_position=goal,
            approved=True,path_clear=self.get_parameter('path_clear_verified').value,
            bundle_sha256=bundle['bundle_sha256'],
            # The area's outline for the avoidance layer: its detours bend inwards at an edge.
            area=[[round(float(x),3),round(float(y),3)] for x,y in bundle['region']['shell']])
    def publish(self):
        try:value=self.approval();self.error=''
        except (ValueError,KeyError,TypeError,OSError) as exc:
            self.error=str(exc);value=dict(stamp=self.now(),mission_id=(self.request or {}).get('mission_id'),
                command_id=(self.request or {}).get('command_id'),approved=False,path_clear=False,reason=self.error)
        self.publisher.publish(String(data=json.dumps(value,allow_nan=False)))
        self.health.publish(String(data=json.dumps(dict(stamp=self.now(),error=self.error,approved=value['approved'],
            origin=self.estimator_origin,origin_error=self.origin_error))))

def main(args=None):
    run_node(RoutePublisher, args=args)


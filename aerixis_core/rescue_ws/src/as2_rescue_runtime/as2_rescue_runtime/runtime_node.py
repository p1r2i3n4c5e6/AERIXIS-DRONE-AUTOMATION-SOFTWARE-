"""ROS bindings for the rescue state machine and asynchronous AERIXIS core motion handover."""
from collections import deque
import json
import math
from pathlib import Path
import time
import uuid
import fcntl
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu,BatteryState,NavSatFix
from geometry_msgs.msg import PoseStamped,TwistStamped
from as2_msgs.msg import PlatformInfo,ControlMode,ControllerInfo,YawMode
from as2_msgs.srv import SetControlMode
from as2_msgs.action import GoToWaypoint, Takeoff, Land
from .safety import Evidence
from .safety import Limits
from .durable import Journal
from .engine import Engine
from . import rtl
from .avoidance import AvoidanceLayer
from .vision_ops import FLIGHT_RECOVERY_OPS
from .node_main import run as run_node
try:
    from px4_msgs.msg import FailsafeFlags
except ImportError:  # px4_msgs absent: AS2 battery topic only
    FailsafeFlags = None

STATE_EVERY_TICKS=2  # rescue/state on every 2nd 20 Hz tick = 10 Hz (+ at once on any safety-relevant change)


# Vibration from the last 25 IMU samples no older than 1 s (>= 10 needed): 0.5 s at the rate gate's
# 50 Hz, as the original 50-sample buffer was at 100 Hz. A strict 0.5 s time window left the busy
# runtime under 10 samples ("vibration_rms unknown", takeoff cancelled; SITL 2026-10-25).
VIBRATION_SAMPLES=25
# A short gap in IMU samples keeps the last vibration estimate this long (2026-10-31). Gazebo
# freezing for ~1 s under load stopped the IMU of every simulated drone at once; "vibration_rms
# unknown" then withdrew readiness and the mission tree's safety gate aborted all 4 drones at
# take-off. Tilt and body rates are still checked on every sample; a longer gap is unknown again.
VIBRATION_HOLD_S=2.0


def vibration_estimate(samples, now, held=None):
    """(rms, held) from (stamp, [ax, ay, az]) samples of the last second; >= 10 samples needed.

    held = (rms, stamp) of the last real estimate: returned instead of NaN for VIBRATION_HOLD_S
    when too few samples arrived."""
    acc=[a for stamp,a in samples if 0<=now-stamp<1.0]
    if len(acc)>=10:
        mean=[sum(a[i] for a in acc)/len(acc) for i in range(3)]
        value=math.sqrt(sum(sum((a[i]-mean[i])**2 for i in range(3)) for a in acc)/len(acc))
        return value,(value,now)
    if held and math.isfinite(held[0]) and 0<=now-held[1]<=VIBRATION_HOLD_S:
        return held[0],held
    return float('nan'),held


def battery_fraction(msg, scale=100.0):
    """Remaining charge 0..1 from AS2's sensor_measurements/battery, or None when unknown.

    AS2's PX4 platform publishes BatteryState.percentage as 100 x PX4 remaining (0..100), not the
    ROS 0..1. Read as a fraction, a real battery at 18 % was 18.0 and never under the 0.25 reserve:
    the low-battery abort could not fire (PX4 v1.16 bridges battery_status; v1.14 did not, so the
    PX4 failsafe flags were used and SITL never showed it; found 2026-10-25). PX4 reports -1 (or
    connected=false) without a battery reading: that is unknown, not empty."""
    value = msg.percentage
    if not msg.present or not math.isfinite(value) or value < 0:
        return None
    return min(1.0, value / scale)


class RosDriver:
    # Velocity-control state; class defaults keep partially constructed test doubles valid.
    velocity_active=False;controller_mode=None;speed_future=None;velocity_error='';path_facing=True;speed_acked=False
    def __init__(self,node,enabled):
        self.node=node;self.enabled=enabled;self.goal=None;self.sending=None;self.result=None
        self.cancel_future=None;self.stopping=False;self.outcome=None;self.hover_future=None;self.hover_ok=False
        self.action=ActionClient(node,GoToWaypoint,'GoToBehavior')
        self.takeoff=ActionClient(node,Takeoff,'TakeoffBehavior');self.land=ActionClient(node,Land,'LandBehavior');self.result_field='go_to_success'
        self.hover=node.create_client(SetControlMode,'controller/set_control_mode')
        self.track_pub=node.create_publisher(String,'gimbal/track_request',10)
        self.release_pub=node.create_publisher(String,'payload/actuate',10)
        self.report_pub=node.create_publisher(String,'rescue/observations',10)
        self.assignment_pub=node.create_publisher(String,'swarm/request',10)
        self.route_pub=node.create_publisher(String,'rescue/route_request',10)
        self.vision_pub=node.create_publisher(String,'vision/command',10)
        self.twist_pub=node.create_publisher(TwistStamped,'motion_reference/twist',qos_profile_sensor_data)
        self.pose_pub=node.create_publisher(PoseStamped,'motion_reference/pose',qos_profile_sensor_data)
        node.create_subscription(ControllerInfo,'controller/info',self.on_controller,qos_profile_system_default)
        self.controller_mode=None;self.speed_future=None;self.velocity_active=False;self.velocity_error=''
    def on_controller(self,msg):self.controller_mode=msg.input_control_mode
    def position(self,frame):
        """The aircraft's latest measured position in `frame` (x, y, z), or None."""
        raw=getattr(self.node,'raw',{}).get('pose')
        if not raw or raw[0].header.frame_id.strip('/')!=str(frame).strip('/'):return None
        p=raw[0].pose.position
        return (p.x,p.y,p.z) if all(math.isfinite(v) for v in (p.x,p.y,p.z)) else None
    @property
    def idle(self):return self.sending is None and self.goal is None and not self.velocity_active
    def velocity(self,linear,yaw,frame):
        """Stream an earth-frame velocity with an absolute yaw (AERIXIS core SPEED + YAW_ANGLE).
        Non-blocking: the control-mode switch is requested once and awaited on later ticks."""
        if not self.enabled:raise ValueError('AERIXIS core motion adapter disabled in aircraft profile')
        if self.sending is not None or self.goal is not None:raise ValueError('A GoTo/takeoff/land action still owns motion')
        if not all(math.isfinite(v) for v in list(linear)+[yaw]) or math.hypot(linear[0],linear[1])>3.0 or abs(linear[2])>1.0:
            raise ValueError('Velocity reference outside the 3 m/s horizontal, 1 m/s vertical envelope')
        if self.velocity_error:error=self.velocity_error;self.velocity_error='';raise ValueError(error)
        self.stopping=False;self.hover_ok=False;self.velocity_active=True
        if self.hover_future is not None:return False
        mode=self.controller_mode
        # Publish only after the controller acknowledged OUR switch to SPEED for this motion: a mode
        # cached from the previous velocity motion can be stale while HOVER/POSITION is still active.
        if not self.speed_acked or mode is None or mode.control_mode!=ControlMode.SPEED or mode.yaw_mode!=ControlMode.YAW_ANGLE:
            if self.speed_future is None:
                if not self.hover.service_is_ready():raise ValueError('AERIXIS core controller set_control_mode unavailable')
                request=SetControlMode.Request();request.control_mode.control_mode=ControlMode.SPEED
                request.control_mode.yaw_mode=ControlMode.YAW_ANGLE;self.speed_future=self.hover.call_async(request)
            return False
        stamp=self.node.get_clock().now().to_msg()
        twist=TwistStamped();twist.header.stamp=stamp;twist.header.frame_id=frame
        twist.twist.linear.x,twist.twist.linear.y,twist.twist.linear.z=map(float,linear)
        pose=PoseStamped();pose.header.stamp=stamp;pose.header.frame_id=frame
        # The pose carries the yaw; its position is the aircraft's own, never (0, 0, 0). SITL
        # 2026-10-23: a controller still in POSITION mode read a zero position as "fly to the origin"
        # (no saturation) and PX4 got a 41 m/s velocity setpoint for one cycle (224 deg/s, abort).
        here=self.position(frame)
        if here is None:return False
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=here
        pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
        self.pose_pub.publish(pose);self.twist_pub.publish(twist);return True
    def vision_command(self,value):self.emit(self.vision_pub,value)
    def emit(self,publisher,value):
        value['stamp']=self.node.get_clock().now().nanoseconds/1e9
        publisher.publish(String(data=json.dumps(value,allow_nan=False)))
    def move(self,pose,frame,speed,facing=None):
        if not self.enabled:raise ValueError('AERIXIS core motion adapter disabled in aircraft profile')
        if not self.idle or self.hover_future is not None:raise ValueError('Previous AERIXIS core motion/hold acknowledgement pending')
        if not self.action.server_is_ready():raise ValueError('AERIXIS core GoToBehavior unavailable')
        self.stopping=False;self.hover_ok=False;self.outcome=None
        goal=GoToWaypoint.Goal();goal.target_pose.header.frame_id=frame
        goal.target_pose.header.stamp=self.node.get_clock().now().to_msg()
        goal.target_pose.point.x,goal.target_pose.point.y,goal.target_pose.point.z=map(float,pose)
        # PATH_FACING: the nose (and the body-mounted gimbal) looks where the drone flies, so a
        # forward/down search camera sees the lane ahead instead of behind on return legs.
        # Return-to-launch passes facing=False: turning the nose ~180 deg while accelerating out
        # of an orbit made a sharp roll/pitch transient that tripped the stability monitor.
        facing=self.path_facing if facing is None else facing
        goal.max_speed=float(speed);goal.yaw.mode=YawMode.PATH_FACING if facing else YawMode.KEEP_YAW
        self.result_field='go_to_success';self.sending=self.action.send_goal_async(goal)
    def vertical(self,operation,height,speed):
        if not self.enabled or not self.idle or self.hover_future is not None:raise ValueError('Motion ownership unavailable')
        client=self.takeoff if operation=='ManagedTakeoff' else self.land
        if not client.server_is_ready():raise ValueError('AERIXIS core vertical behavior unavailable')
        if operation=='ManagedTakeoff':
            goal=Takeoff.Goal();goal.takeoff_height=float(height);goal.takeoff_speed=float(speed);self.result_field='takeoff_success'
        else:
            goal=Land.Goal();goal.land_speed=float(speed);self.result_field='land_success'
        self.stopping=False;self.hover_ok=False;self.outcome=None;self.sending=client.send_goal_async(goal)
    def poll(self):
        if self.sending is not None and self.sending.done():
            self.goal=self.sending.result();self.sending=None
            if not self.goal.accepted:self.goal=None;self.outcome='FAILURE'
            else:self.result=self.goal.get_result_async()
        if self.stopping and self.goal is not None and self.cancel_future is None:
            self.cancel_future=self.goal.cancel_goal_async()
        if self.result is not None and self.result.done():
            value=self.result.result();self.outcome='SUCCESS' if value.status==4 and getattr(value.result,self.result_field,False) else 'FAILURE'
            self.goal=None;self.result=None;self.cancel_future=None
        if self.speed_future is not None and self.speed_future.done():
            response=self.speed_future.result();self.speed_future=None
            if response is None or not response.success:self.velocity_error='AERIXIS core controller refused SPEED/YAW_ANGLE mode'
            else:self.speed_acked=True
        if self.hover_future is not None and self.hover_future.done():
            self.hover_ok=bool(self.hover_future.result().success);self.hover_future=None
    def abandon(self):
        """Forget a motion that never handed back after its cancel was sent (ManagedLand takes
        over). The action server still finishes the old goal; its late result is ignored."""
        self.sending=None;self.goal=None;self.result=None;self.cancel_future=None
        self.velocity_active=False;self.hover_future=None;self.stopping=False;self.outcome=None
    def stop(self):
        if not self.stopping:self.hover_ok=False
        if self.velocity_active:self.speed_acked=False   # the next velocity motion switches again
        if self.velocity_active and self.controller_mode is not None and self.controller_mode.control_mode==ControlMode.SPEED:
            stop=TwistStamped();stop.header.stamp=self.node.get_clock().now().to_msg();stop.header.frame_id='earth'
            self.twist_pub.publish(stop)  # zero velocity until HOVER is accepted
        self.velocity_active=False
        self.stopping=True
    def reanchor(self):
        """Ask for HOVER again: AERIXIS core's controller freezes the hover reference at the measured
        position on every HOVER request (used while, and right after, another mode flew)."""
        self.hover_ok=False;self.stopping=True
    def hold(self):
        self.stopping=True
        if not self.enabled or not self.idle:return False
        if self.hover_ok:return True
        if self.hover_future is None and self.hover.service_is_ready():
            request=SetControlMode.Request();request.control_mode.control_mode=ControlMode.HOVER
            request.control_mode.yaw_mode=ControlMode.NONE;self.hover_future=self.hover.call_async(request)
        return False
    def motion_result(self):return self.outcome
    def request_route(self,command,pose,frame):
        self.emit(self.route_pub,dict(mission_id=command['mission_id'],command_id=command['command_id'],
            operation=command['operation'],segment_id=command.get('segment_id',''),target_pose=pose,frame_id=frame,
            avoidance=bool(command.get('avoid'))))
    def track(self,track_id):self.emit(self.track_pub,dict(enabled=track_id is not None,track_id=track_id))
    def release(self,transaction,payload_id):self.emit(self.release_pub,dict(transaction_id=transaction,payload_id=payload_id))
    def report(self,value):self.emit(self.report_pub,value)
    def assignment(self,mission,target,operation):
        self.emit(self.assignment_pub,dict(mission_id=mission,target_id=target,operation=operation,owner=self.node.get_namespace().strip('/')))

class RescueRuntime(Node):
    def __init__(self,args=None):
        super().__init__('rescue_runtime',cli_args=args)
        self.declare_parameter('enable_motion',False);self.declare_parameter('state_dir',str(Path.home()/'.local/share/as2-rescue'/self.get_namespace().strip('/')))
        self.clock=lambda:self.get_clock().now().nanoseconds/1e9
        self.evidence=Evidence(self.clock);self.raw={};self.acceleration=deque(maxlen=VIBRATION_SAMPLES);self.vibration_held=None
        # Diagnostics for "readiness lost" lines: IMU arrival times and this node's tick gaps.
        self.imu_rx=deque(maxlen=200);self.tick_at=deque(maxlen=60)
        folder=Path(self.get_parameter('state_dir').value);folder.mkdir(parents=True,exist_ok=True,mode=0o700)
        self.owner_lock=open(folder/'runtime.lock','a+');fcntl.flock(self.owner_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.driver=RosDriver(self,self.get_parameter('enable_motion').value)
        self.engine=Engine(self.evidence,Journal(folder/'rescue.json'),self.driver,self.clock)
        self.declare_parameter('overhead_envelope_verified',False)
        self.declare_parameter('allow_operator_load',False)
        self.declare_parameter('min_overhead_agl_m',8.0);self.declare_parameter('max_overhead_agl_m',20.0)
        self.declare_parameter('geotag_exclusion_radius_m',6.0);self.declare_parameter('claim_exclusion_radius_m',8.0)
        self.declare_parameter('path_facing_legs',True)
        # Nadir search (2026-10-31): gimbal straight down, swinging this far to each side of the
        # lane over this period while nobody is locked (0 = fixed nadir); a person further ahead
        # along the lane than approach_ahead_m is responded to only once the lane brings them abeam.
        self.declare_parameter('search_sweep_deg',30.0);self.declare_parameter('search_sweep_period_s',8.0)
        self.declare_parameter('approach_ahead_m',3.0);self.declare_parameter('respond_radius_m',15.0)
        # Own return height per aircraft (rtl.py): system_id comes from the onboard launch.
        self.declare_parameter('system_id',0)
        # Same value as swarm_agent's minimum_separation_m (the profile sets both; 3 m only with RTK).
        self.declare_parameter('minimum_separation_m',10.0)
        self.declare_parameter('return_altitude_base_m',rtl.RETURN_BASE_M)
        self.declare_parameter('return_altitude_step_m',-1.0)   # <= 0: the fleet separation
        self.driver.path_facing=bool(self.get_parameter('path_facing_legs').value)
        self.engine.vision.exclusion_radius_m=float(self.get_parameter('geotag_exclusion_radius_m').value)
        self.engine.vision.claim_radius_m=float(self.get_parameter('claim_exclusion_radius_m').value)
        self.engine.vision.sweep_deg=max(0.0,min(60.0,float(self.get_parameter('search_sweep_deg').value)))
        self.engine.vision.sweep_period_s=max(2.0,float(self.get_parameter('search_sweep_period_s').value))
        self.engine.vision.approach_ahead_m=max(0.0,float(self.get_parameter('approach_ahead_m').value))
        self.engine.vision.respond_radius_m=max(3.0,float(self.get_parameter('respond_radius_m').value))
        slot=rtl.slot_for(self.get_parameter('system_id').value,self.get_namespace().strip('/'))
        step=float(self.get_parameter('return_altitude_step_m').value)
        step=step if step>0 else float(self.get_parameter('minimum_separation_m').value)
        self.engine.vision.return_altitude_m=rtl.return_altitude(slot,float(self.get_parameter('return_altitude_base_m').value),step)
        self.get_logger().info('Return altitude %.0f m above take-off (slot %d)'%(self.engine.vision.return_altitude_m,slot))
        # AI avoidance: OFF unless the GCS switches it on (mission manager -> avoidance/enabled,
        # latched) or the profile says so for a test. OFF = every leg is the usual GoTo.
        self.declare_parameter('avoidance_enabled',False);self.declare_parameter('avoidance_model','')
        model=self.get_parameter('avoidance_model').value or None
        self.engine.avoidance=AvoidanceLayer(model,float(self.get_parameter('minimum_separation_m').value))
        self.engine.avoidance.set_enabled(self.get_parameter('avoidance_enabled').value)
        self.get_logger().info('AI avoidance: %s, %s%s'%('ON' if self.engine.avoidance.enabled else 'off',
            self.engine.avoidance.mode,(' — '+self.engine.avoidance.error) if self.engine.avoidance.error else ''))
        self.create_subscription(String,'avoidance/enabled',self.avoidance_switch,QoSProfile(depth=1,
            reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.engine.identity=self.get_namespace().strip('/')
        self.engine.safety.limits=Limits(overhead_envelope_verified=self.get_parameter('overhead_envelope_verified').value,
            allow_operator_load=self.get_parameter('allow_operator_load').value,
            min_overhead_agl_m=self.get_parameter('min_overhead_agl_m').value,
            max_overhead_agl_m=self.get_parameter('max_overhead_agl_m').value)
        self.boot=uuid.uuid4().hex;self.sequence=0
        self.state_pub=self.create_publisher(String,'rescue/state',10);self.result_pub=self.create_publisher(String,'rescue/result',100)
        self.create_subscription(String,'rescue/command',self.command,20)
        self.create_service(Trigger,'rescue/reconcile_ground',self.reconcile_ground)
        for key,topic in [('camera','camera/health'),('fix','rescue/target_fix'),
            ('zone','delivery/zone'),('payload','payload/status'),('swarm','swarm/status'),('route','rescue/route')]:
            self.create_subscription(String,topic,lambda m,k=key:self.json_sample(k,m),10)
        # Camera: vision/target + gimbal/state + perception/tracks of one frame arrive as one
        # vision/frame (A8 bridge, SITL sim vision): same data and rate, one Python callback per frame
        # instead of three (Pi 5, 2026-10-23). camera_topics:=separate for a source without it.
        self.declare_parameter('camera_topics','frame')
        # BatteryState.percentage scale of the platform: 100 = AS2 PX4 platform (0..100), 1 = ROS 0..1.
        self.declare_parameter('battery_percentage_scale',100.0)
        if self.get_parameter('camera_topics').value=='separate':
            for key,topic in [('gimbal','gimbal/state'),('vision','vision/target')]:
                self.create_subscription(String,topic,lambda m,k=key:self.json_sample(k,m),10)
            self.create_subscription(String,'perception/tracks',self.tracks,10)
        else:
            self.create_subscription(String,'vision/frame',self.vision_frame,10)
        # FC guardian state (2 Hz and on change): PX4's flight mode name, for the takeover reason.
        self.guardian=None
        self.create_subscription(String,'fc_guardian/state',self.guardian_state,qos_profile_sensor_data)
        for key,topic,kind in [('pose','self_localization/pose',PoseStamped),('twist','self_localization/twist',TwistStamped),
            ('imu','sensor_measurements/imu',Imu),('battery','sensor_measurements/battery',BatteryState),('platform','platform/info',PlatformInfo)]:
            self.create_subscription(kind,topic,lambda m,k=key:self.raw_sample(k,m),qos_profile_sensor_data)
        self.create_subscription(NavSatFix,'sensor_measurements/gps',self.gps,qos_profile_sensor_data)
        # PX4 v1.14 does not bridge battery_status over uXRCE-DDS, so AS2's battery topic stays
        # silent. PX4's failsafe battery warning is then the battery evidence (either prefix).
        self.failsafe=None
        if FailsafeFlags is not None:
            for topic in {'/fmu/out/failsafe_flags',self.get_namespace().rstrip('/')+'/fmu/out/failsafe_flags'}:
                self.create_subscription(FailsafeFlags,topic,lambda m:setattr(self,'failsafe',(m,self.clock())),qos_profile_sensor_data)
        self.create_timer(.05,self.tick)
    def gps(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not 0<=self.clock()-stamp<=1.0:return
        valid=msg.status.status>=0 and all(math.isfinite(v) for v in (msg.latitude,msg.longitude,msg.altitude))
        sigma=math.sqrt(max(msg.position_covariance[0],msg.position_covariance[4])) if msg.position_covariance_type>0 else 3.0
        self.evidence.put('gps',dict(valid=bool(valid),latitude=msg.latitude,longitude=msg.longitude,altitude=msg.altitude,
            horizontal_sigma_m=sigma if math.isfinite(sigma) else 3.0),stamp)
    def publish_result(self,result):
        if result.get('status')=='FAILURE':
            self.get_logger().warning('%s failed: %s'%(result.get('operation') or result.get('command_id','')[-6:],result.get('reason','')))
        self.result_pub.publish(String(data=json.dumps(result)))
    def publish_outbox(self):
        while self.engine.outbox:
            result=self.engine.outbox.pop(0)
            if result:self.publish_result(result)
    def raw_sample(self,key,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not 0<=self.clock()-stamp<=1.0:return
        self.raw[key]=(msg,stamp)
        if key=='platform':
            # PX4's own OFFBOARD flag (vehicle_control_mode): a change in flight that AS2 did not
            # ask for is a takeover (Engine.mode_guard).
            mode=self.guardian[0] if self.guardian and 0<=self.clock()-self.guardian[1]<=1.5 else None
            self.evidence.put('platform',dict(armed=bool(msg.armed),offboard=bool(msg.offboard),connected=bool(msg.connected),
                state=msg.status.state,px4_mode=mode),stamp)
        if key=='imu':
            self.acceleration.append((stamp,[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]))
            self.imu_rx.append(self.clock())
    def input_diagnostics(self):
        """' [IMU n/s received, newest x s old; longest tick gap y s]': tells a starved runtime
        (few IMU messages handled, long tick gaps) from a silent flight controller."""
        now=self.clock();rx=[t for t in self.imu_rx if 0<=now-t<=1.0]
        newest=now-self.imu_rx[-1] if self.imu_rx else float('nan')
        ticks=list(self.tick_at);gap=max((b-a for a,b in zip(ticks,ticks[1:])),default=float('nan'))
        return ' [IMU %d/s received, newest %.2f s old; longest tick gap %.2f s]'%(len(rx),newest,gap)
    def vision_frame(self,msg):
        try:frame=json.loads(msg.data)
        except ValueError:return
        if not isinstance(frame,dict):return
        # Same order and handling as the separate topics were published and handled.
        for key in ('vision','gimbal'):
            if isinstance(frame.get(key),dict):self.json_value(key,frame[key])
        if isinstance(frame.get('tracks'),dict):self.tracks_value(frame['tracks'])
    def json_sample(self,key,msg):
        try:self.json_value(key,json.loads(msg.data))
        except ValueError:pass
    def json_value(self,key,value):
        try:
            if key=='camera':
                if not value.get('fresh',False):return
                stamp=self.clock() # Camera health is a locally generated liveness heartbeat.
            else:stamp=value['stamp']
            self.evidence.put(key,value,stamp)
        except (ValueError,TypeError,KeyError):pass
    def guardian_state(self,msg):
        try:
            mode=json.loads(msg.data)['px4']['nav']
            if mode is not None:self.guardian=(str(mode),self.clock())
        except (ValueError,TypeError,KeyError):pass
    def tracks(self,msg):
        try:self.tracks_value(json.loads(msg.data))
        except ValueError:pass
    def tracks_value(self,data):
        try:
            stamp=data.get('image_stamp_ns',0)/1e9
            tracks=[t for t in data.get('tracks',[]) if t.get('confirmed') and not t.get('predicted')]
            current=self.evidence.get('track');wanted=(self.engine.active or {}).get('track_id') or (current or {}).get('track_id')
            choices=[t for t in tracks if t.get('track_id')==wanted] if wanted is not None else tracks
            if choices:self.evidence.put('track',max(choices,key=lambda t:t.get('confidence',0)),stamp)
            else:self.evidence.put('track',{},self.clock())
        except (ValueError,TypeError,KeyError):pass
    def vehicle(self):
        if any(k not in self.raw or not 0<=self.clock()-self.raw[k][1]<=1.0 for k in ('pose','twist','imu','platform')):return
        pose=self.raw['pose'][0];twist=self.raw['twist'][0];imu=self.raw['imu'][0];platform=self.raw['platform'][0]
        fraction=None
        if 'battery' in self.raw and 0<=self.clock()-self.raw['battery'][1]<=.5:
            fraction=battery_fraction(self.raw['battery'][0],float(self.get_parameter('battery_percentage_scale').value));battery_source='as2'
        if fraction is None and self.failsafe and self.clock()-self.failsafe[1]<=1.0:
            # No battery reading from AS2: PX4's own battery failsafe state is the evidence.
            flags=self.failsafe[0];battery_source='px4_failsafe'
            fraction=0.0 if flags.battery_unhealthy or flags.battery_warning>=2 else .2 if flags.battery_warning==1 or flags.battery_low_remaining_time else 1.0
        if fraction is None:return
        q=pose.pose.orientation;norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        tilt=math.degrees(math.acos(max(-1,min(1,1-2*(q.x*q.x+q.y*q.y)))))
        vibration,self.vibration_held=vibration_estimate(self.acceleration,self.clock(),self.vibration_held)
        v=twist.twist.linear;w=imu.angular_velocity;p=pose.pose.position
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        # Earth-frame velocity (AS2's twist is in the body frame), for in-flight avoidance.
        velocity=[v.x*math.cos(yaw)-v.y*math.sin(yaw),v.x*math.sin(yaw)+v.y*math.cos(yaw),v.z]
        value=dict(position=[p.x,p.y,p.z],yaw=yaw,velocity=velocity,frame_id=pose.header.frame_id,position_valid=abs(norm-1)<.01 and bool(pose.header.frame_id) and all(math.isfinite(x) for x in (p.x,p.y,p.z)),
            connected=platform.connected,battery=fraction,battery_source=battery_source,speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),tilt_deg=tilt,
            body_rate_dps=math.degrees(math.sqrt(w.x*w.x+w.y*w.y+w.z*w.z)),tilt_rate_dps=math.degrees(math.hypot(w.x,w.y)),vibration_rms=vibration,
            hold_ack=self.driver.hover_ok,motion_idle=self.driver.idle,armed=platform.armed,platform_state=platform.status.state,
            offboard=platform.offboard)
        self.evidence.put('vehicle',value,min(self.raw[k][1] for k in ('pose','twist','imu','platform')))
    def reconcile_ground(self,request,response):
        try:
            self.engine.reconcile_ground();response.success=True
            response.message='Runtime reconciled on ground; payload history retained'
        except ValueError as exc:response.success=False;response.message=str(exc)
        return response
    def command(self,msg):
        try:
            value=json.loads(msg.data);result=self.engine.submit(value);self.publish_outbox()
            if result:self.publish_result(result)
        except (ValueError,TypeError,KeyError) as exc:
            self.get_logger().warning(str(exc))
            if isinstance(locals().get('value'),dict):
                result={k:value.get(k,'') for k in ('command_id','mission_id','run_id')};result.update(status='FAILURE',reason=str(exc))
                self.publish_result(result)
    def avoidance_switch(self,msg):
        try:
            data=json.loads(msg.data) if msg.data.strip().startswith('{') else dict(enabled=msg.data.strip().lower() in ('1','true','on'))
            wanted=bool(data.get('enabled'))
        except (ValueError,AttributeError):
            self.get_logger().warning('Ignored avoidance switch %r'%msg.data);return
        if wanted!=self.engine.avoidance.enabled:
            ok=self.engine.avoidance.set_enabled(wanted)
            if wanted and not ok:self.get_logger().error('AI avoidance can not be switched on: '+self.engine.avoidance.error)
            elif ok:self.get_logger().warning('AI avoidance ON (%s): legs bend around other drones'%self.engine.avoidance.mode)
            else:self.get_logger().info('AI avoidance off: legs are the usual GoTo')
    def response_hold(self):
        """The separation hold of the operation still running (never a stale one)."""
        hold=self.engine.vision.hold
        return hold if hold and (self.engine.active or {}).get('operation')==hold.get('op') else None
    def tick(self):
        self.tick_at.append(self.clock())
        try:
            self.driver.poll();self.vehicle();flags=self.engine.flags()
            if self.engine.safety.abort_reason!=getattr(self,'logged_reason',''):
                self.logged_reason=self.engine.safety.abort_reason
                if self.logged_reason:
                    v=self.evidence.get('vehicle') or {}
                    self.get_logger().error('ABORT %s: tilt=%s tilt_rate=%s rate=%s vibration=%s battery=%s(%s) position_valid=%s'%(
                        self.logged_reason,*[v.get(k) for k in ('tilt_deg','tilt_rate_dps','body_rate_dps','vibration_rms','battery','battery_source','position_valid')]))
            recovering=(self.engine.active or {}).get('operation') in FLIGHT_RECOVERY_OPS
            if not flags['VehicleHealthy'] and not self.driver.idle and not recovering:self.driver.stop()
            if self.driver.stopping:self.driver.hold()
            result=self.engine.tick();self.publish_outbox()
            if result:self.publish_result(result)
            if self.engine.external!=getattr(self,'logged_external',False):
                self.logged_external=self.engine.external
                if self.engine.external:self.get_logger().error(self.engine.mode_change)
                else:self.get_logger().warning('PX4 back in OFFBOARD after an external mode change: AERIXIS core holds here; '
                    'the old operation is not resumed (only return/land are accepted)')
            self.engine.vision.housekeeping(self.clock())
            self.sequence+=1
            state=dict(schema_version=1,stamp=self.clock(),boot_id=self.boot,sequence=self.sequence,flags=self.engine.flags(),
                mission_id=self.engine.owner,active_operation=(self.engine.active or {}).get('operation'),target_id=self.engine.target,
                motion_enabled=self.driver.enabled,hold_pending=self.driver.stopping and not self.engine.flags()['HoldConfirmed'],
                payload_transaction=self.engine.last_release,checkpoint=self.journal_checkpoint(),
                geotags=self.engine.geotags()[-10:],home=self.engine.home,
                incident_label=self.engine.incident_label,handled_labels=sorted(self.engine.handled_labels),
                claim=self.engine.vision.claim_state(),yielded=self.engine.vision.yielded,
                handled=self.engine.vision.handled_points[-10:],served=self.engine.vision.served_points[-10:],watch=self.engine.vision.watch,
                hold=self.response_hold(),avoidance=self.engine.avoidance.status,waiting=self.engine.waiting,
                mode_change=self.engine.mode_change,external_mode=self.engine.external)
            note=(self.engine.vision.watch or {}).get('note')
            if note and note!=getattr(self,'watch_note',None):
                # Mission 2 watch moved, held or stopped short: one log line per change.
                # One call site per severity: rclpy raises if a call site changes severity, and a
                # tick exception latches an abort.
                line='Watch %s: %s'%(self.engine.vision.watch['state'],note)
                if self.engine.vision.watch['state'] in ('holding','stopped short'):self.get_logger().warning(line)
                else:self.get_logger().info(line)
            self.watch_note=note
            health=state['flags'].get('health') or ''
            if health!=getattr(self,'health_note',''):
                # Why VehicleHealthy is false (SafetyGate fails the flight on it): one line per change.
                if health:self.get_logger().warning('Vehicle readiness lost: '+health+self.input_diagnostics())
                else:self.get_logger().info('Vehicle readiness back')
            self.health_note=health
            hold=(state['hold'] or {}).get('note') or ''
            if hold!=getattr(self,'hold_note',''):
                if hold:self.get_logger().warning('Person response '+hold)
                else:self.get_logger().info('Person response separation hold ended')
            self.hold_note=hold
            state['payload']=self.evidence.get('payload')
            v=self.evidence.get('vehicle')
            state['vehicle']=None if v is None else {k:(round(v[k],3) if isinstance(v.get(k),float) and math.isfinite(v[k]) else v.get(k) if not isinstance(v.get(k),float) else None)
                for k in ('battery','battery_source','tilt_deg','tilt_rate_dps','body_rate_dps','vibration_rms','speed','connected','armed','position_valid','platform_state','offboard')}
            if v is not None:  # earth frame = KML plan frame once the plan origin is set (map display)
                state['vehicle'].update(position=[round(x,2) for x in v['position']],yaw=round(v.get('yaw') or 0.0,3))
            gps=self.evidence.get('gps',1.0)
            if gps and gps.get('valid'):state['gps']=dict(latitude=gps['latitude'],longitude=gps['longitude'],altitude=gps['altitude'])
            # Published at STATE_PERIOD_S, and at once whenever anything a safety reader acts on
            # changes (flags, operation, hold, abort/mode change, payload transaction), so those
            # reach the payload node / delivery zone / guardian as fast as at 20 Hz. Every reader
            # accepts state up to 0.5 s old (guardian 3 s). Pi 5, 2026-10-23: at 20 Hz to 6
            # subscribers this JSON cost ~1 CPU core and starved the person detector.
            urgent=json.dumps([state['flags'],state['active_operation'],state['hold_pending'],state['mission_id'],
                state['mode_change'],state['external_mode'],state['payload_transaction'],state['target_id']],
                sort_keys=True,default=str)
            self.unpublished_ticks=getattr(self,'unpublished_ticks',0)+1
            if urgent!=getattr(self,'published_urgent',None) or self.unpublished_ticks>=STATE_EVERY_TICKS:
                self.state_pub.publish(String(data=json.dumps(state,allow_nan=False)))
                self.published_urgent,self.unpublished_ticks=urgent,0
        except Exception as exc:
            self.driver.stop();self.driver.track(None);self.engine.aborted=True;self.get_logger().error(str(exc))
    def journal_checkpoint(self):return self.engine.journal.state['missions'].get(self.engine.owner)

def stop_and_hover(node):
    """On the way out (ROS still up): stop motion, stop tracking, wait up to 3 s for hover."""
    node.driver.stop();node.driver.track(None)
    end=time.monotonic()+3
    while time.monotonic()<end and not node.driver.idle:rclpy.spin_once(node,timeout_sec=.05)
def main(args=None):
    run_node(RescueRuntime,args=args,graceful=stop_and_hover,cleanup=lambda node:node.owner_lock.close())

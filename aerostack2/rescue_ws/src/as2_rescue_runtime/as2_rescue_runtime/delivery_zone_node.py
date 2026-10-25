"""Convert a calibrated target fix and explicit delivery policy into a bounded zone.

This consumes an operator's surveyed-zone clearance; it does not infer that an
image contains no obstacles or that a package impact envelope is safe.
"""
import json
import math
from pathlib import Path
import re
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from as2_rescue_planner.bundle import verify_bundle
from .node_main import run as run_node

class DeliveryZone(Node):
    def __init__(self):
        super().__init__('rescue_delivery_zone')
        for key,value in [('clearance_verified',False),('beside_offset_m',5.0),
                ('mission_directory',str(Path.home()/'.local/share/as2-missions'/self.get_namespace().strip('/')))]:self.declare_parameter(key,value)
        self.fix=None;self.pose=None;self.supervisor=None
        self.pub=self.create_publisher(String,'delivery/zone',10)
        self.create_subscription(String,'rescue/target_fix',self.on_fix,10)
        self.create_subscription(String,'rescue/state',self.on_state,10)
        self.create_subscription(PoseStamped,'self_localization/pose',lambda m:setattr(self,'pose',m),qos_profile_sensor_data)
        self.create_timer(.1,self.tick)
    def on_fix(self,msg):
        try:self.fix=json.loads(msg.data)
        except ValueError:self.fix=None
    def on_state(self,msg):
        try:self.supervisor=json.loads(msg.data)
        except ValueError:self.supervisor=None
    def tick(self):
        now=self.get_clock().now().nanoseconds/1e9;value=dict(stamp=now,approved=False,clear=False,path_clear=False)
        try:
            state=self.supervisor
            if not state or not 0<=now-state['stamp']<=.5:raise ValueError('Supervisor is stale')
            mission=state['mission_id']
            if not isinstance(mission,str) or not re.fullmatch('[0-9a-f]{32}',mission):raise ValueError('No staged mission selected')
            artifact=json.loads((Path(self.get_parameter('mission_directory').value)/(mission+'.bundle.json')).read_text())
            bundle=artifact['bundle'];verify_bundle(bundle)
            if artifact['mission_id']!=mission or bundle['drone_id']!=self.get_namespace().strip('/'):raise ValueError('Delivery policy identity mismatch')
            policy=bundle.get('delivery_policy',{});mode=policy.get('drop_mode','beside')
            if mode not in ('beside','overhead'):raise ValueError('Unknown delivery geometry')
            if not self.get_parameter('clearance_verified').value:raise ValueError('Delivery corridor/zone has not been surveyed and cleared')
            if not self.pose or not 0<=now-(self.pose.header.stamp.sec+self.pose.header.stamp.nanosec/1e9)<=.5:raise ValueError('Stale aircraft pose')
            if policy.get('planned_waypoint'):
                goal=bundle['coverage']['waypoints'][-1]['pose'];target=policy['target_id'];frame=bundle['coverage']['frame_id'];clearance=self.get_parameter('beside_offset_m').value
            else:
                fix=self.fix
                if not fix or not 0<=now-fix['stamp']<=.5 or fix.get('valid') is not True or fix.get('sigma_m',99)>1.5:raise ValueError('No fresh calibrated target fix')
                if not fix.get('target_id'):raise ValueError('A shared target identity is required')
                frame=fix['frame_id'];target=fix['target_id'];goal=list(fix['position']);goal[2]=self.pose.pose.position.z
                clearance=0. if mode=='overhead' else self.get_parameter('beside_offset_m').value;goal[0]+=clearance
            if self.pose.header.frame_id!=frame or not all(math.isfinite(x) for x in goal) or not 1<=goal[2]<=50:raise ValueError('Invalid delivery frame/altitude')
            value.update(frame_id=frame,target_id=target,approach_position=goal,drop_mode=mode,
                overhead_authorized=policy.get('overhead_authorized') is True,person_clearance_m=clearance,
                approved=True,clear=True,path_clear=True,clearance_source='operator_survey',
                kind='planned_waypoint' if policy.get('planned_waypoint') else 'detected_person',requires_visual_tracking=not policy.get('planned_waypoint',False))
        except (ValueError,KeyError,TypeError,OSError) as exc:value['reason']=str(exc)
        self.pub.publish(String(data=json.dumps(value,allow_nan=False)))

def main(args=None):
    run_node(DeliveryZone, args=args)


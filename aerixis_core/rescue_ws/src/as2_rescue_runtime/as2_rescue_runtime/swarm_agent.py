"""Per-drone peer freshness/separation, responder-lease monitoring and person de-confliction
(this drone's person claim and geotags ride in its peer frames; peers' come back in swarm/status)."""
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from .swarm import (PeerPeople, keep_out_zones, person_claim, recent_served, recent_tags, separation_safe,
                    supervisor_allows_renewal)
from .node_main import run as run_node

class Agent(Node):
    def __init__(self):
        super().__init__('swarm_agent')
        self.declare_parameter('expected_peers',['']);self.declare_parameter('minimum_separation_m',10.0)
        self.declare_parameter('max_peer_age_s',1.0);self.declare_parameter('default_sigma_m',2.5)
        self.declare_parameter('claim_ttl_s',3.0);self.people=PeerPeople(self.get_parameter('claim_ttl_s').value)
        # Extra clearance around peers and their orbits that a watching drone keeps out of.
        self.declare_parameter('keep_out_buffer_m',3.0);self.declare_parameter('default_orbit_radius_m',10.0)
        self.twist=None;self.gps_sigma=None;self.own=None;self.peers={};self.assignment=None;self.claim=None;self.supervisor=None;self.claim_owner=None
        self.identity=self.get_namespace().strip('/')
        if not self.identity:raise ValueError('A drone namespace is required')
        self.peer_pub=self.create_publisher(String,'swarm/peer_tx',10);self.request_pub=self.create_publisher(String,'swarm/request_tx',10)
        self.status_pub=self.create_publisher(String,'swarm/status',10)
        # AS2 publishes pose and twist (not odom); position uncertainty comes from the GNSS fix.
        self.create_subscription(PoseStamped,'self_localization/pose',self.pose,qos_profile_sensor_data)
        self.create_subscription(TwistStamped,'self_localization/twist',lambda m:setattr(self,'twist',m),qos_profile_sensor_data)
        self.create_subscription(NavSatFix,'sensor_measurements/gps',self.gps,qos_profile_sensor_data)
        self.create_subscription(String,'swarm/peer_rx',self.peer,10);self.create_subscription(String,'swarm/assignments',self.assignments,10)
        self.create_subscription(String,'swarm/request',self.request,10);self.create_subscription(String,'rescue/state',self.safety,10)
        self.create_timer(.2,self.tick);self.renewed=0
    def now(self):return self.get_clock().now().nanoseconds/1e9
    def safety(self,msg):
        try:self.supervisor=json.loads(msg.data)
        except ValueError:self.supervisor=None
    def gps(self,msg):
        cov=msg.position_covariance
        if msg.position_covariance_type>0 and max(cov[0],cov[4])>0 and math.isfinite(max(cov[0],cov[4])):
            self.gps_sigma=math.sqrt(max(cov[0],cov[4]))
    def pose(self,msg):
        p=msg.pose.position;stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z)):return
        speed=0.0;velocity=[0.0,0.0]
        if self.twist is not None:
            v=self.twist.twist.linear;speed=math.sqrt(v.x**2+v.y**2+v.z**2)
            # AS2's twist is in the body frame: turn it into the earth frame with the yaw.
            q=msg.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            velocity=[v.x*math.cos(yaw)-v.y*math.sin(yaw),v.x*math.sin(yaw)+v.y*math.cos(yaw)]
        # Without a reported GNSS covariance the configured default is used, deliberately conservative.
        sigma=self.gps_sigma if self.gps_sigma is not None else self.get_parameter('default_sigma_m').value
        self.own=dict(owner=self.identity,position=[p.x,p.y,p.z],frame_id=msg.header.frame_id,stamp=stamp,sigma_m=sigma,speed=speed,
                      velocity=velocity)
    def peer(self,msg):
        try:
            value=json.loads(msg.data)
            if value.get('owner')!=self.identity and 0<=self.now()-value['stamp']<=2:
                old=self.peers.get(value['owner'])
                if not old or value['stamp']>old['stamp']:self.peers[value['owner']]=value;self.people.update(value['owner'],value,self.now())
        except (ValueError,KeyError,TypeError):pass
    def assignments(self,msg):
        try:
            value=json.loads(msg.data)
            if 0<=self.now()-value['stamp']<=1:self.assignment=value
        except (ValueError,KeyError,TypeError):pass
    def request(self,msg):
        try:
            value=json.loads(msg.data)
            if value.get('owner')!=self.identity or not 0<=self.now()-value['stamp']<=1:return
            if value['operation']=='claim':
                if not supervisor_allows_renewal(self.supervisor,self.now()):return
                self.claim=value;self.claim_owner=self.supervisor['mission_id']
            elif value['operation']=='finish':
                held=self.supervisor and 0<=self.now()-self.supervisor['stamp']<=.5 and self.supervisor['flags']['HoldConfirmed']
                if not held:return
                value['hold_verified']=True;self.request_pub.publish(String(data=json.dumps(value)));self.claim=None
        except (ValueError,KeyError,TypeError):pass
    def tick(self):
        now=self.now();expected=[p for p in self.get_parameter('expected_peers').value if p and p!=self.identity]
        age=self.get_parameter('max_peer_age_s').value
        own_fresh=bool(self.own and 0<=now-self.own['stamp']<=age)
        peers=[self.peers.get(p) for p in expected]
        fresh=own_fresh and bool(expected) and all(p and 0<=now-p['stamp']<=age for p in peers)
        safe=False
        if fresh:
            # Per peer: separation + both sigmas + both speeds x (real message age + link latency).
            safe=separation_safe(self.own,peers,now,self.get_parameter('minimum_separation_m').value,age)
        # Person de-confliction (not the responder lease above): the person this drone responds to.
        person=person_claim(self.supervisor,now);tags=recent_tags(self.supervisor) if self.supervisor else []
        served=recent_served(self.supervisor) if self.supervisor else []
        if own_fresh:
            # Centimetres / milliseconds are plenty, and every byte counts on a LoRa channel.
            frame=dict(self.own,position=[round(v,2) for v in self.own['position']],stamp=round(self.own['stamp'],3),
                       sigma_m=round(self.own['sigma_m'],2),speed=round(self.own['speed'],2),
                       velocity=[round(v,2) for v in self.own['velocity']])
            if person:frame['claim']=person
            if tags:frame['tags']=tags
            if served:frame['served']=served
            self.peer_pub.publish(String(data=json.dumps(frame)))
        authority_fresh=bool(self.assignment and 0<=now-self.assignment['stamp']<=1)
        owned=False
        supervisor_ok=supervisor_allows_renewal(self.supervisor,now) and self.supervisor.get('mission_id')==self.claim_owner
        if self.claim and authority_fresh and supervisor_ok:
            owned=any(a['owner']==self.identity and a['mission']==self.claim['mission_id'] and a['target']==self.claim['target_id'] and a['state']=='OWNED' and a['expires']>now for a in self.assignment['assignments'])
        if self.claim and supervisor_ok and now-self.renewed>=1:
            self.request_pub.publish(String(data=json.dumps(dict(self.claim,stamp=now))));self.renewed=now
        value=dict(stamp=now,required=bool(expected),peers_fresh=bool(fresh),separation_safe=bool(safe),owned=owned,
            mission_id=(self.claim or {}).get('mission_id'),target_id=(self.claim or {}).get('target_id'),authority_fresh=authority_fresh,
            peers=[dict(owner=p,age_s=now-self.peers[p]['stamp'] if p in self.peers else None) for p in expected],
            person_claim=person,peer_claims=self.people.claims(now),peer_tags=self.people.tags(now),
            peer_served=self.people.served(now))
        heard=[self.peers[p] for p in expected if p in self.peers]
        value['keep_out']=keep_out_zones(self.own,heard,value['peer_claims'],now,
            self.get_parameter('minimum_separation_m').value,age,self.get_parameter('keep_out_buffer_m').value,
            self.get_parameter('default_orbit_radius_m').value,self.get_parameter('default_sigma_m').value,
            self.get_parameter('claim_ttl_s').value) if own_fresh else []
        # Peers as the avoidance layer needs them (newest frame each; position, velocity, age).
        frame=(self.own or {}).get('frame_id')
        value['neighbours']=[dict(owner=p['owner'],xy=p['position'][:2],z=p['position'][2],velocity=p.get('velocity') or [0.0,0.0],
            sigma_m=p.get('sigma_m',self.get_parameter('default_sigma_m').value),age_s=round(now-p['stamp'],3))
            for p in heard if p.get('frame_id','earth')==frame and len(p.get('position') or [])==3]
        self.status_pub.publish(String(data=json.dumps(value,allow_nan=False)))

def main(args=None):
    run_node(Agent, args=args)


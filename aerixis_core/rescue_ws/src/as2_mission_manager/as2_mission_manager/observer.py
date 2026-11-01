"""Read the actual namespaced ROS graph and platform state, without commanding flight."""
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from as2_msgs.msg import PlatformInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json


class RosObserver:
    def __init__(self, drone_id):
        self.node = Node('mission_manager_observer',namespace=drone_id)
        self.prefix = '/'+drone_id
        self.lock = threading.Lock()
        self.nodes = None
        self.updated = None
        self.platform = None
        self.platform_at = None
        self.ground_since = None
        self.error = ''
        self.rescue = None
        self.rescue_at = None
        self.node.create_subscription(String,'rescue/state',self._rescue,10)
        # Compact link/vision summaries for the GCS status poll (sent over the radio).
        self.extra={};self.extra_at={}
        for key,topic in (('vision','vision/target'),('mesh','swarm/radio_health'),('swarm','swarm/status'),('fc','fc/health'),
                          ('guardian','fc_guardian/state'),('payload','payload/status')):
            self.node.create_subscription(String,topic,lambda m,k=key:self._extra(k,m),10)
        self.subscription = self.node.create_subscription(PlatformInfo,'platform/info',self._platform,10)
        # Latched: a restarted FC guardian gets the mission's emergency action again.
        self.guardian_action = self.node.create_publisher(String,'fc_guardian/set_action',QoSProfile(
            depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # AI avoidance switch for the rescue runtime (latched: a restarted runtime gets it again).
        self.avoidance_pub = self.node.create_publisher(String,'avoidance/enabled',QoSProfile(
            depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Operator emergency to the FC guardian (PX4's own RTL/LAND; the guardian ignores it on the ground).
        self.guardian_request = self.node.create_publisher(String,'fc_guardian/request',10)
        self.node.create_timer(0.5,self._graph)
        self.runtime_reconcile=self.node.create_client(Trigger,'rescue/reconcile_ground')
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._spin,daemon=True)
        self.thread.start()

    def _rescue(self,msg):
        try:
            data=json.loads(msg.data)
            age=self.node.get_clock().now().nanoseconds/1e9-data['stamp']
            if not 0<=age<=.5:return
            with self.lock:self.rescue=data;self.rescue_at=time.monotonic()
        except (ValueError,TypeError,KeyError):pass

    def _extra(self,key,msg):
        try:data=json.loads(msg.data)
        except ValueError:return
        if key=='vision':
            gimbal=data.get('gimbal') or {}
            data=dict(fresh=data.get('fresh'),locked=data.get('locked'),label=data.get('label'),
                      lock_state=data.get('lock_state'),suppressed=data.get('suppressed'),fps=data.get('fps'),
                      source=data.get('source'),gimbal_yaw=gimbal.get('yaw'),gimbal_pitch=gimbal.get('pitch'),
                      hfov_deg=data.get('hfov_deg'),vfov_deg=data.get('vfov_deg'))
        elif key=='swarm':
            # Person de-confliction: who this drone and its peers are responding to.
            claims=[dict(owner=c.get('owner'),label=c.get('label')) for c in data.get('peer_claims') or [] if isinstance(c,dict)]
            data=dict(peers_fresh=data.get('peers_fresh'),separation_safe=data.get('separation_safe'),
                      required=data.get('required'),peers=data.get('peers'),
                      person_claim=(data.get('person_claim') or {}).get('label'),peer_claims=claims[:8])
        elif key=='payload':
            # Payload set-up and what is left (GCS plans deliveries from it; sent over the radio).
            data=dict(capacity=data.get('capacity'),loaded=data.get('loaded'),remaining=data.get('remaining'),
                      servos=[dict(set=s.get('set'),drops=s.get('drops')) for s in (data.get('servos') or [])[:6] if isinstance(s,dict)],
                      truth=data.get('truth'),next=data.get('payload_id'))
        elif key=='guardian':
            emergency=data.get('emergency') or {}
            data=dict(state=data.get('state'),action=data.get('action'),detail=str(data.get('detail',''))[:80],
                      as2_alive=(data.get('as2') or {}).get('alive'),
                      airborne=(data.get('px4') or {}).get('airborne'),nav=(data.get('px4') or {}).get('nav'),
                      emergency=dict(action=emergency.get('action'),state=emergency.get('state'),
                                     source=emergency.get('source')) if emergency else None)
        with self.lock:self.extra[key]=data;self.extra_at[key]=time.monotonic()

    def _platform(self,msg):
        with self.lock:
            now = time.monotonic()
            on_ground = msg.connected and not msg.armed and msg.status.state in (0,1)
            if not on_ground:
                self.ground_since = None
            elif self.ground_since is None or self.platform_at is None or now-self.platform_at > 2:
                self.ground_since = now
            self.platform = dict(connected=msg.connected,armed=msg.armed,offboard=msg.offboard,
                                 state=msg.status.state)
            self.platform_at = now

    def _graph(self):
        with self.lock:
            self.nodes = sorted(set((ns.rstrip('/')+'/'+name) for name,ns in self.node.get_node_names_and_namespaces()
                                    if ns == self.prefix or ns.startswith(self.prefix+'/')))
            self.updated = time.monotonic()

    def _spin(self):
        try:
            while not self.stop.is_set() and rclpy.ok():
                rclpy.spin_once(self.node,timeout_sec=0.2)
        except Exception as exc:
            with self.lock:self.error=str(exc)

    def set_emergency_action(self,action):
        self.guardian_action.publish(String(data=action))

    def set_avoidance(self,enabled):
        self.avoidance_pub.publish(String(data=json.dumps(dict(enabled=bool(enabled),stamp=time.time()))))

    def runtime_avoidance(self):
        """The runtime's own report of the switch ({} without fresh rescue state)."""
        with self.lock:
            if self.rescue_at is None or time.monotonic()-self.rescue_at > 2 or not isinstance(self.rescue,dict):
                return {}
            return dict(self.rescue.get('avoidance') or {})

    def runtime_abort(self):
        """Reason the rescue runtime holds a latched abort ('' when fresh state shows none)."""
        with self.lock:
            if self.rescue_at is None or time.monotonic()-self.rescue_at > 2 or not isinstance(self.rescue,dict):
                return ''
            flags=self.rescue.get('flags') or {}
            return str(flags.get('reason') or 'ABORT_LATCHED') if flags.get('AbortRequested') else ''

    def request_emergency(self,action,source):
        self.guardian_request.publish(String(data=json.dumps(dict(stamp=time.time(),action=action,source=source))))

    def reconcile_runtime(self,timeout=5.0):
        """Clear the rescue runtime's latched abort too (it re-checks ground evidence itself)."""
        if not self.runtime_reconcile.service_is_ready():
            return 'rescue runtime not running; nothing to reconcile'
        future=self.runtime_reconcile.call_async(Trigger.Request())
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:time.sleep(0.05)
        if not future.done():raise ValueError('Rescue runtime did not answer the reconcile request')
        response=future.result()
        if not response.success:raise ValueError('Rescue runtime refused reconcile: '+response.message)
        return response.message

    def safe_on_ground(self):
        with self.lock:
            now=time.monotonic()
            return bool(self.ground_since is not None and self.platform_at is not None and
                        now-self.platform_at < 2 and now-self.ground_since >= 1)

    def snapshot(self):
        with self.lock:
            age=None if self.updated is None else time.monotonic()-self.updated
            available=age is not None and age < 3 and not self.error
            return dict(available=available,nodes=self.nodes if available else None,
                        count=len(self.nodes) if available else None,age_s=age,error=self.error,
                        rescue=dict(fresh=self.rescue_at is not None and time.monotonic()-self.rescue_at<1,data=self.rescue),
                        platform=self.platform,
                        links={k:dict(v,age_s=round(time.monotonic()-self.extra_at[k],2)) for k,v in self.extra.items()},
                        platform_age_s=None if self.platform_at is None else time.monotonic()-self.platform_at)

    def close(self):
        self.stop.set();self.thread.join(timeout=2);self.node.destroy_node()

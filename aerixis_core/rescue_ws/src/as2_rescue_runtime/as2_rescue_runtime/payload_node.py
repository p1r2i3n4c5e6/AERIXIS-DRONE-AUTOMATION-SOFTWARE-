"""PX4 AUX adapter with durable one-shot actuation and independent physical feedback.
Map an AUX output to Offboard Actuator Set N in PX4; N is not the AUX pin number.

Several packages (2026-11-01): `servos` lists the servos as "set:drops,set:drops" (e.g. "1:1,2:3":
one package on actuator set 1, a three-step dropper on set 2). A servo with N drops moves, for its
k-th drop, to hold_value + (release_value - hold_value) * k / N; drops go servo by servo, step
by step. Empty `servos` = the single servo actuator_set (one drop at release_value), as before.
The operator confirms how many packages are loaded at upload (<mission>.payload.json `count`);
each drop has its own package ID "<payload_id>#k" and its own one-shot journal transaction.
"""
import json
import math
import re
from pathlib import Path
import fcntl
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from px4_msgs.msg import VehicleCommand,VehicleCommandAck
from .durable import Journal
from .node_main import run as run_node

class Payload(Node):
    def __init__(self):
        super().__init__('payload_aux_adapter')
        for k,v in [('enabled',False),('actuator_set',0),('release_value',0.0),('target_system',0),
            ('servos',''),('hold_value',-1.0),
            ('allow_operator_load',False),('fmu_prefix',''),('mission_directory',str(Path.home()/'.local/share/as2-missions'/self.get_namespace().strip('/'))),
            ('state_dir',str(Path.home()/'.local/share/as2-payload'/self.get_namespace().strip('/')))]:self.declare_parameter(k,v)
        folder=Path(self.get_parameter('state_dir').value);folder.mkdir(parents=True,exist_ok=True,mode=0o700)
        self.lock=open(folder/'actuator.lock','a+');fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.journal=Journal(folder/'actuator.json');self.sensor=None;self.supervisor=None;self.active=None;self.ack=None
        # '' = real FC (/fmu/...), '/drone0' = namespaced SITL (/drone0/fmu/...).
        prefix=str(self.get_parameter('fmu_prefix').value).strip().rstrip('/');fmu=prefix+'/fmu'
        self.pub=self.create_publisher(VehicleCommand,fmu+'/in/vehicle_command',qos_profile_sensor_data)
        self.status=self.create_publisher(String,'payload/status',10)
        self.create_subscription(String,'payload/sensors',self.sensors,10)
        self.create_subscription(String,'rescue/state',self.safety,10)
        self.create_subscription(String,'payload/actuate',self.release,10)
        self.create_subscription(VehicleCommandAck,fmu+'/out/vehicle_command_ack',self.command_ack,qos_profile_sensor_data)
        self.create_timer(.1,self.tick)
    def now(self):return self.get_clock().now().nanoseconds/1e9
    def slots(self):
        """Release steps from the profile; [] (adapter disabled) if the profile is invalid."""
        try:return release_slots(self.get_parameter('servos').value,self.get_parameter('actuator_set').value,
                             self.get_parameter('release_value').value,self.get_parameter('hold_value').value)
        except (ValueError,TypeError):return []
    def sensors(self,msg):
        try:
            data=json.loads(msg.data)
            if not 0<=self.now()-data['stamp']<=.5:return
            if self.sensor and data['stamp']<=self.sensor['stamp']:return
            self.sensor=data
        except (ValueError,KeyError,TypeError):pass
    def safety(self,msg):
        try:self.supervisor=json.loads(msg.data)
        except ValueError:self.supervisor=None
    def release(self,msg):
        try:
            request=json.loads(msg.data);now=self.now();state=self.supervisor;sensor=self.sensor
            target=self.get_parameter('target_system').value;slots=self.slots()
            if not self.get_parameter('enabled').value or not slots or not 1<=target<=255:raise ValueError('AUX adapter/profile not explicitly configured')
            if not 0<=now-request['stamp']<=.5 or not state or not 0<=now-state['stamp']<=.5:raise ValueError('Stale release authority')
            flags=state['flags']
            # Two authorities: the delivery-zone release, or an overhead drop that the
            # runtime authorises only while the gimbal is at nadir over the geotagged person.
            if not ((flags['SafeToRelease'] and flags['AssignmentOwned']) or flags.get('OverheadDropReady') is True):
                raise ValueError('Onboard release interlock denied')
            operator=self.operator_load()
            if not operator and (not sensor or not 0<=now-sensor['stamp']<=.5 or not all(sensor.get(k) is True for k in ('verified','present','latched'))):raise ValueError('Load confirmation unavailable')
            mission=state['mission_id'];zone=state['target_id'];payload=request['payload_id'];transaction=request['transaction_id']
            if (operator or sensor).get('payload_id')!=payload or transaction!=mission+':'+zone+':'+payload:raise ValueError('Payload transaction does not match supervisor identity')
            number=package_number(payload)
            if not 1<=number<=len(slots):raise ValueError('Package %s has no servo step (%d configured)'%(payload,len(slots)))
            index,value=slots[number-1]['set'],slots[number-1]['value']
            fresh,prior=self.journal.reserve_release(mission,zone,payload,transaction)
            if not fresh:return # No resend after loss, retry, restart, or uncertain ACK.
            self.active=dict(transaction_id=transaction,payload_id=payload,stamp=now,operator_only=bool(operator),command_sent=False);self.ack=None
            command=VehicleCommand();command.timestamp=int(now*1e6);command.command=VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR
            for i in range(1,7):setattr(command,'param'+str(i),float('nan'))
            setattr(command,'param'+str(index),float(value));command.param7=0.0
            command.target_system=target;command.target_component=1;command.source_system=target;command.source_component=191;command.from_external=True
            self.pub.publish(command)
            self.active['command_sent']=True
        except (ValueError,TypeError,KeyError) as exc:self.get_logger().error(str(exc))
    def command_ack(self,msg):
        if self.active and msg.command==VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR and msg.target_component==191 and msg.target_system==self.get_parameter('target_system').value:
            # ACK cannot identify a package release or prove that the servo moved.
            self.ack=int(msg.result)
    def operator_load(self):
        if not self.get_parameter('allow_operator_load').value or not self.supervisor:return None
        mission=self.supervisor.get('mission_id')
        import re
        if not isinstance(mission,str) or not re.fullmatch('[0-9a-f]{32}',mission):return None
        try:
            value=json.loads((Path(self.get_parameter('mission_directory').value)/(mission+'.payload.json')).read_text())
            if value['mission_id']!=mission or value.get('confirmed_loaded') is not True:return None
            return loaded_package(value,self.journal.state['releases'],mission,len(self.slots()))
        except (ValueError,KeyError,OSError):return None
    def tick(self):
        now=self.now();sensor=self.sensor if self.sensor and 0<=now-self.sensor['stamp']<=.5 else {}
        operator=self.operator_load()
        confirmed=False
        if self.active:
            confirmed=bool(sensor.get('verified') is True and sensor.get('payload_id')==self.active['payload_id'] and
                sensor.get('present') is False and sensor.get('released') is True and sensor.get('latched') is False and sensor['stamp']>self.active['stamp'])
            if confirmed:self.journal.release_result(self.active['transaction_id'],True)
            elif now-self.active['stamp']>3:self.journal.release_result(self.active['transaction_id'],False)
        result=dict(stamp=now,verified=bool(sensor.get('verified')),present=sensor.get('present'),latched=sensor.get('latched'),
            payload_id=sensor.get('payload_id'),released=confirmed,transaction_id=(self.active or {}).get('transaction_id'),command_ack=self.ack,
            transaction_state=self.journal.state['releases'].get((self.active or {}).get('transaction_id'),{}).get('state','NONE'))
        result.update(operator_confirmed=bool(operator),consumed=bool(self.active),
            command_sent=bool((self.active or {}).get('command_sent')),operator_only=bool((self.active or {}).get('operator_only')),
            truth='PHYSICALLY_CONFIRMED' if confirmed else 'COMMAND_SENT_UNVERIFIED' if self.active else 'OPERATOR_CONFIRMED_LOADED' if operator else 'SENSOR' if sensor else 'UNKNOWN')
        slots=self.slots()
        loaded=operator['count'] if operator else 0
        remaining=operator['count']-operator['used'] if operator else 0
        # For the GCS (fetched with the drone status): what this drone can carry and has left.
        result.update(capacity=len(slots),loaded=loaded,remaining=remaining,consumed=bool(self.active) and not operator,
            servos=[dict(set=s['set'],drops=s['drops']) for s in slots if s['step']==1])
        if operator:result['payload_id']=operator['payload_id']
        elif self.active:result['payload_id']=self.active['payload_id']
        self.status.publish(String(data=json.dumps(result)))

def servo_list(spec,actuator_set=0):
    """[(actuator set 1..6, drops)] from "set:drops,set:drops"; '' = the single actuator_set."""
    out=[]
    for part in str(spec or '').replace(' ','').split(','):
        if not part:continue
        match=re.fullmatch(r'([1-6])(?::(\d{1,2}))?',part)
        if not match:raise ValueError('Servo "%s": use set:drops, set 1..6'%part)
        drops=int(match.group(2) or 1)
        if not 1<=drops<=20:raise ValueError('Servo %s: 1..20 drops'%match.group(1))
        if any(s==int(match.group(1)) for s,_ in out):raise ValueError('Actuator set %s listed twice'%match.group(1))
        out.append((int(match.group(1)),drops))
    if not out and 1<=int(actuator_set or 0)<=6:out=[(int(actuator_set),1)]
    return out


def release_slots(spec,actuator_set,release_value,hold_value=-1.0):
    """Ordered release steps: servo by servo, step k of N at hold + (release - hold) * k / N."""
    release,hold=float(release_value),float(hold_value)
    if not (math.isfinite(release) and math.isfinite(hold) and -1<=release<=1 and -1<=hold<=1):
        raise ValueError('Servo values must be -1..1')
    try:servos=servo_list(spec,actuator_set)
    except ValueError:return []
    return [dict(set=s,step=k,drops=n,value=round(hold+(release-hold)*k/n,4)) for s,n in servos for k in range(1,n+1)]


def loaded_package(value,releases,mission,capacity):
    """The operator-confirmed load of this mission: value with the next package ID ("<id>#k" when
    several are loaded), count and used; None when all are dropped, none fit (capacity 0), or the
    package ID was already used in an earlier mission (each load needs a new ID). Drops are
    counted per mission: a reused ID must not eat this load's packages (SITL 2026-11-01)."""
    if capacity<1:return None
    count=max(1,min(int(value.get('count',1)),capacity))
    base=str(value['payload_id'])
    ours=[r for r in releases.values() if r['payload_id']==base or str(r['payload_id']).startswith(base+'#')]
    if any(r.get('mission')!=mission for r in ours):return None
    used=len(ours)
    if used>=count:return None
    # One package ID per drop when several are loaded; a single package keeps its ID.
    return dict(value,payload_id=base if count==1 else '%s#%d'%(base,used+1),count=count,used=used)


def package_number(payload_id):
    """k of "<id>#k" (a package of several); 1 for a single package ID."""
    match=re.search(r'#(\d+)$',str(payload_id))
    return int(match.group(1)) if match else 1


def main(args=None):
    run_node(Payload, args=args, cleanup=lambda node: node.lock.close())


"""PX4 AUX adapter with durable one-shot actuation and independent physical feedback.
Map an AUX output to Offboard Actuator Set N in PX4; N is not the AUX pin number.
"""
import json
import math
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
            index=self.get_parameter('actuator_set').value;value=self.get_parameter('release_value').value;target=self.get_parameter('target_system').value
            if not self.get_parameter('enabled').value or not 1<=index<=6 or not 1<=target<=255 or not math.isfinite(value) or not -1<=value<=1:raise ValueError('AUX adapter/profile not explicitly configured')
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
            if any(r['payload_id']==value['payload_id'] for r in self.journal.state['releases'].values()):return None
            return value
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
        if operator:result['payload_id']=operator['payload_id']
        elif self.active:result['payload_id']=self.active['payload_id']
        self.status.publish(String(data=json.dumps(result)))

def main(args=None):
    run_node(Payload, args=args, cleanup=lambda node: node.lock.close())


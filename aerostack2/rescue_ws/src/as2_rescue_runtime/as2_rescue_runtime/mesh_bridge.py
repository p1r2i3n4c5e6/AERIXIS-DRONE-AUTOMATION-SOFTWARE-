"""Drone-to-drone swarm link over a LoRa UART modem or an IP mesh / base station network.
Not an RF mesh/routing driver: modem addressing/routing is configured on the radios.
The laptop GCS is never part of this network.

transport:=serial  device + baud of a transparent UART modem (LoRa, SiK in mesh mode)
transport:=udp     IP mesh radios or a base-station Wi-Fi/Ethernet network:
                   bind_port receives; peers lists "ip:port" (or a broadcast address)
transport:=both    the base station (UDP) as the main link and the LoRa modem as a fallback that is
                   always on: every frame goes over UDP at full rate and over LoRa at the modem's
                   rate, so when the base-station network drops (out of its 0.5-1 km range) the
                   peers keep hearing each other without a switch-over. A frame that arrives on both
                   is accepted once (Receiver drops completed transfers); without a working modem the
                   UDP link carries on and the modem is retried every 10 s.
Every frame is authenticated with the fleet key file; the payload format is identical
for all transports.
"""
import base64
from collections import deque
import json
from pathlib import Path
import socket
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .radio import encode,Receiver
from .swarm import compact_peer,expand_peer


class SerialTransport:
    def __init__(self,device,baud):
        import serial
        self.port=serial.Serial(device,baud,timeout=0,write_timeout=.2);self.buffer=bytearray()
    def read_frames(self):
        self.buffer.extend(self.port.read(min(4096,self.port.in_waiting)))
        if len(self.buffer)>8192:self.buffer.clear();raise ValueError('Radio framing overflow')
        frames=[]
        for _ in range(16):
            if b'\n' not in self.buffer:break
            line,_,remaining=self.buffer.partition(b'\n');self.buffer=bytearray(remaining);frames.append(line)
        return frames
    def write(self,line):self.port.write(line+b'\n')
    def close(self):self.port.close()


class UdpTransport:
    def __init__(self,bind_port,peers):
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
        self.sock.bind(('0.0.0.0',int(bind_port)));self.sock.setblocking(False)
        self.peers=[]
        for item in peers:
            host,_,port=item.strip().rpartition(':')
            if not host:host,port=item.strip(),bind_port
            self.peers.append((host,int(port)))
        if not self.peers:raise ValueError('UDP mesh needs at least one peer or broadcast address')
    def read_frames(self):
        frames=[]
        for _ in range(32):
            try:data,_=self.sock.recvfrom(8192)
            except (BlockingIOError,OSError):break
            frames.extend(line for line in data.split(b'\n') if line)
        return frames
    def write(self,line):
        for peer in self.peers:
            try:self.sock.sendto(line,peer)
            except OSError:pass  # An unreachable peer must not block the others.
    def close(self):self.sock.close()


class MeshBridge(Node):
    def __init__(self):
        super().__init__('swarm_radio_bridge')
        for k,v in [('transport','serial'),('device',''),('baud',57600),('bind_port',24800),('peers',['']),
                    ('key_file',''),('identity',''),('authority_id','base'),('authority',False),('frames_per_second',2.0)]:
            self.declare_parameter(k,v)
        identity=self.get_parameter('identity').value or self.get_namespace().strip('/')
        config=json.loads(Path(self.get_parameter('key_file').value).read_text())
        self.identity=identity;self.authority=self.get_parameter('authority').value;self.authority_id=self.get_parameter('authority_id').value
        if not identity:raise ValueError('Explicit radio identity (or drone namespace) required')
        transport=self.get_parameter('transport').value
        if transport not in ('serial','udp','both'):raise ValueError('transport must be serial, udp or both')
        self.links={}            # name -> transport; 'udp' first: the main link when both run
        self.queues={}           # name -> frames waiting to go out on that link
        self.last_send={}
        self.heard={}            # name -> {sender: last time a frame from it arrived on this link}
        self.link_error={}
        if transport in ('udp','both'):
            # UDP is fast enough for more frames than a LoRa duty cycle allows.
            self.links['udp']=UdpTransport(self.get_parameter('bind_port').value,[p for p in self.get_parameter('peers').value if p])
        if transport in ('serial','both'):
            if not self.get_parameter('device').value:raise ValueError('Serial mesh needs an explicit radio device')
            self.serial_retry=0.0;self.open_serial(strict=transport=='serial')
        for name in ('udp','serial'):self.queues[name]=deque();self.last_send[name]=0.0;self.heard[name]={}
        self.transport=transport
        self.key=config['send_key'].encode();self.receivers={sender:Receiver(key.encode(),time.time) for sender,key in config['receive_keys'].items()}
        self.pending={};self.received={};self.error=''
        self.outputs={kind:self.create_publisher(String,topic,10) for kind,topic in [('peer','swarm/peer_rx'),('assignment','swarm/assignments'),('request','swarm/request')]}
        self.health=self.create_publisher(String,'swarm/radio_health',10)
        routes=[('assignment','swarm/assignments')] if self.authority else [('peer','swarm/peer_tx'),('request','swarm/request_tx')]
        for kind,topic in routes:self.create_subscription(String,topic,lambda m,k=kind:self.queue(k,m),10)
        self.create_timer(.02,self.tick)
        self.get_logger().info('Swarm mesh bridge %s as %s (%s)'%(transport,identity,'authority' if self.authority else 'drone'))
    def open_serial(self,strict=False):
        try:
            self.links['serial']=SerialTransport(self.get_parameter('device').value,self.get_parameter('baud').value)
            self.link_error.pop('serial',None)
        except Exception as exc:     # 'both': the base station carries on without the modem
            if strict:raise
            self.links.pop('serial',None);self.link_error['serial']='LoRa modem unavailable: %s'%exc
            self.serial_retry=time.time()+10.0
    def queue(self,kind,msg):
        try:
            data=json.loads(msg.data)
            if kind=='request' and data.get('owner')!=self.identity:return
            key=kind+':'+str(data.get('target_id',''))
            payload=compact_peer(data) if kind=='peer' else data
            if len(self.pending)<16 or key in self.pending:self.pending[key]=(time.time(),dict(sender=self.identity,kind=kind,payload=payload))
        except ValueError:pass
    def accept(self,line,now,link='udp'):
        try:packet=base64.b64decode(line,validate=True)
        except ValueError:return
        for sender,receiver in self.receivers.items():
            if sender==self.identity:continue  # our own broadcast echo
            value=receiver.accept(packet)
            if not value:continue
            kind=value.get('kind');data=value.get('payload')
            if value.get('sender')!=sender or not isinstance(data,dict):continue
            if kind=='peer':data=expand_peer(data)
            if kind=='assignment' and (self.authority or sender!=self.authority_id):continue
            if kind=='request' and (not self.authority or data.get('owner')!=sender):continue
            if kind=='peer' and data.get('owner')!=sender:continue
            if kind not in self.outputs:continue
            self.received[sender]=now;self.heard[link][sender]=now;self.outputs[kind].publish(String(data=json.dumps(data)))
    def tick(self):
        now=time.time()
        if self.transport=='both' and 'serial' not in self.links and now>=self.serial_retry:self.open_serial()
        for name,link in list(self.links.items()):
            try:
                for line in link.read_frames():self.accept(line,now,name)
                self.link_error.pop(name,None)
            except (OSError,ValueError) as exc:
                self.link_error[name]=str(exc)
                if name=='serial' and self.transport=='both':
                    try:link.close()
                    except Exception:pass
                    self.links.pop('serial',None);self.serial_retry=now+10.0
        try:
            # A new message is encoded once and offered to every link (the same frames, so a copy
            # that arrives on the second link is dropped by the receivers).
            busy=any(self.queues[name] for name in self.links)
            if not busy and self.pending:
                key=next(iter(self.pending));queued,value=self.pending.pop(key)
                if now-queued<2:
                    parts=encode(value,self.key,now,ttl=5)
                    if len(parts)<=8:
                        for name in self.links:
                            self.queues[name].extend(parts)
                            while len(self.queues[name])>8:self.queues[name].popleft()   # LoRa: newest only
                    else:self.error='Swarm message exceeds low-bandwidth profile'
            rate=self.get_parameter('frames_per_second').value
            if not 0<rate<=50:raise ValueError('Radio frame rate must be in (0,50]')
            for name,link in list(self.links.items()):
                link_rate=max(rate,20.0) if name=='udp' else rate
                frames=self.queues[name]
                while frames and now-self.last_send[name]>=1/link_rate:
                    link.write(base64.b64encode(frames.popleft()));self.last_send[name]=now
                    if name=='serial':break
            if int(now*5)!=getattr(self,'health_tick',None):
                self.health_tick=int(now*5)
                self.health.publish(String(data=json.dumps(dict(stamp=now,identity=self.identity,transport=self.transport,
                    peers={k:round(now-t,2) for k,t in self.received.items()},
                    links={name:dict(up=name in self.links,peers={k:round(now-t,2) for k,t in self.heard[name].items()},
                                     error=self.link_error.get(name,'')) for name in (('udp','serial') if self.transport=='both' else (self.transport,))},
                    error=self.error or '; '.join(self.link_error.values()),
                    queued_frames=sum(len(q) for q in self.queues.values())))))
        except (OSError,ValueError) as exc:self.error=str(exc)

def main(args=None):
    rclpy.init(args=args);node=MeshBridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        for link in node.links.values():link.close()
        node.destroy_node();rclpy.try_shutdown()

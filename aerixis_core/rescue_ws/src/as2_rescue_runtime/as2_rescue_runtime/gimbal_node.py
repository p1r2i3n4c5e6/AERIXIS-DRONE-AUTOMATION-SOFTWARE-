"""Measured A8 feedback + rate-limited image centering. Never commands aircraft motion."""
from collections import deque
import json
import math
import socket
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from .siyi import encode,decode,attitude,rotation,CenteringController
from .node_main import run as run_node

class Gimbal(Node):
    def __init__(self):
        super().__init__('a8_gimbal')
        for key,value in [('camera_ip','192.168.144.25'),('sdk_port',37260),('enable_control',False),
            ('calibration_verified',False),('yaw_sign',1.0),('pitch_sign',-1.0)]:self.declare_parameter(key,value)
        self.address=(self.get_parameter('camera_ip').value,self.get_parameter('sdk_port').value)
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.connect(self.address);self.sock.setblocking(False)
        self.sequence=0;self.pending={};self.state=None;self.at=0;self.tracks=None;self.request=None;self.camera=None
        self.controller=CenteringController();self.history=deque(maxlen=20);self.latched=False;self.last_command=(0,0)
        self.output=self.create_publisher(String,'gimbal/state',10)
        self.create_subscription(String,'perception/tracks',self.on_tracks,10)
        self.create_subscription(String,'gimbal/track_request',self.on_request,10)
        self.create_subscription(CameraInfo,'camera/camera_info',lambda m:setattr(self,'camera',m),10)
        self.create_timer(.05,self.tick)
    def on_tracks(self,msg):
        try:self.tracks=(json.loads(msg.data),self.get_clock().now().nanoseconds/1e9)
        except ValueError:self.tracks=None
    def on_request(self,msg):
        try:
            value=json.loads(msg.data)
            if abs(self.get_clock().now().nanoseconds/1e9-value['stamp'])>.5:return
            self.request=(value,time.monotonic())
        except (ValueError,KeyError,TypeError):self.request=None
    def send(self,cmd,data=b''):
        seq=self.sequence;self.sequence=(seq+1)&65535
        self.sock.send(encode(cmd,data,seq))
        if cmd==0x0d:self.pending[seq]=time.monotonic()
    def tick(self):
        now=time.monotonic();rosnow=self.get_clock().now().nanoseconds/1e9
        try:
            for _ in range(20):
                try:packet=decode(self.sock.recv(2048))
                except BlockingIOError:break
                except ValueError:continue
                sent=self.pending.pop(packet['sequence'],None)
                if packet['command']==0x0d and packet['control']&2 and sent is not None and now-sent<.25:
                    self.state=attitude(packet['payload']);self.at=now
                    self.history.append(self.state['pitch_rate'])
            self.pending={s:t for s,t in self.pending.items() if now-t<.5}
            self.send(0x0d)
            fresh=self.state is not None and now-self.at<.25
            rates=list(self.history);flips=sum(a*b<0 and min(abs(a),abs(b))>3 for a,b in zip(rates,rates[1:]))
            if fresh and (flips>=6 or max(abs(self.state[k]) for k in ('yaw_rate','pitch_rate'))>45):self.latched=True
            error=(0,0);tracking=False;target_id=None
            if self.request and now-self.request[1]<.3 and self.request[0].get('enabled') and self.tracks and self.camera:
                data,received=self.tracks
                image_age=rosnow-data.get('image_stamp_ns',0)/1e9
                matches=[t for t in data.get('tracks',[]) if str(t['track_id'])==str(self.request[0].get('track_id')) and t.get('confirmed') and not t.get('predicted')]
                if 0<=rosnow-received<.3 and 0<=image_age<.3 and len(matches)==1 and self.camera.width>0 and self.camera.height>0:
                    t=matches[0];x1,y1,x2,y2=t['bbox_pixels'];target_id=str(t['track_id'])
                    error=((x1+x2)/self.camera.width-1,(y1+y2)/self.camera.height-1);tracking=True
            valid=fresh and not self.latched and self.get_parameter('calibration_verified').value
            yaw,pitch=self.controller.update(error,valid and tracking)
            yaw=round(yaw*self.get_parameter('yaw_sign').value);pitch=round(pitch*self.get_parameter('pitch_sign').value)
            if self.get_parameter('enable_control').value:
                self.send(0x07,rotation(yaw,pitch));self.last_command=(yaw,pitch)
            value=dict(schema_version=1,stamp=rosnow,verified=bool(valid),fresh=fresh,oscillating=self.latched,
                tracking=tracking and valid,track_id=target_id,center_error=list(error),control_enabled=self.get_parameter('enable_control').value,
                feedback_age_s=None if self.state is None else now-self.at,**(self.state or {}))
            self.output.publish(String(data=json.dumps(value,allow_nan=False)))
        except (OSError,ValueError,KeyError,TypeError) as exc:
            self.get_logger().warning(str(exc));self.at=0
            self.output.publish(String(data=json.dumps(dict(stamp=rosnow,verified=False,fresh=False,error=str(exc)))))
    def close(self):
        if self.get_parameter('enable_control').value:
            try:self.send(0x07,rotation(0,0))
            except OSError:pass
        self.sock.close();self.destroy_node()

def main(args=None):
    run_node(Gimbal, args=args, cleanup=lambda node: node.sock.close())

"""Timestamped optical-ray geolocation using calibrated TF and measured surface data."""
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from .geolocation import locate
from .node_main import run as run_node

class Geolocator(Node):
    def __init__(self):
        super().__init__('rescue_geolocator')
        for k,v in [('world_frame','earth'),('rectified_input',False),('calibration_verified',False),
            ('position_sigma_m',1.0),('angle_sigma_rad',.02),('origin_verified',False),
            ('origin_latitude',0.0),('origin_longitude',0.0),('origin_altitude',0.0),('shared_target_id','')]:self.declare_parameter(k,v)
        self.camera=None;self.surface=None;self.last_stamp=0
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pub=self.create_publisher(String,'rescue/target_fix',10)
        self.create_subscription(CameraInfo,'camera/camera_info',lambda m:setattr(self,'camera',m),10)
        self.create_subscription(String,'rescue/surface',self.on_surface,10)
        self.create_subscription(String,'perception/tracks',self.on_tracks,10)
    def now(self):return self.get_clock().now().nanoseconds/1e9
    def on_surface(self,msg):
        try:self.surface=json.loads(msg.data)
        except ValueError:self.surface=None
    def on_tracks(self,msg):
        output=dict(stamp=self.now(),valid=False)
        try:
            data=json.loads(msg.data);stamp=data.get('image_stamp_ns',0)
            if stamp<=self.last_stamp or not 0<=self.now()-stamp/1e9<=.3:raise ValueError('Stale or repeated image')
            self.last_stamp=stamp
            camera=self.camera;surface=self.surface;frame=self.get_parameter('world_frame').value
            if not camera or not self.get_parameter('calibration_verified').value or not self.get_parameter('rectified_input').value:raise ValueError('Verified rectified camera calibration required')
            if camera.header.frame_id!=data['frame_id'] or camera.k[0]<=0 or camera.k[4]<=0:raise ValueError('Camera calibration/frame mismatch')
            if not surface or not 0<=self.now()-surface['stamp']<=.5 or surface['frame_id']!=frame:raise ValueError('Fresh surface measurement in the world frame required')
            tracks=[t for t in data['tracks'] if t.get('confirmed') and not t.get('predicted')]
            if len(tracks)!=1:raise ValueError('Select one unambiguous confirmed track for geolocation')
            track=tracks[0];box=track['bbox_pixels']
            transform=self.buffer.lookup_transform(frame,camera.header.frame_id,Time(nanoseconds=stamp),timeout=Duration(seconds=0))
            t=transform.transform.translation;q=transform.transform.rotation
            if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.01:raise ValueError('Invalid camera transform quaternion')
            x,y,z,w=q.x,q.y,q.z,q.w
            rotation=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                      [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                      [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
            # Use bottom centre as the person/surface contact point, not assumed human height.
            fix=locate([(box[0]+box[2])/2,box[3]],[camera.k[0],camera.k[4],camera.k[2],camera.k[5]],rotation,[t.x,t.y,t.z],
                surface['surface_z'],self.get_parameter('position_sigma_m').value,self.get_parameter('angle_sigma_rad').value,surface['sigma_m'],surface.get('verified') is True)
            output.update(fix,valid=True,frame_id=frame,track_id=track['track_id'],image_stamp_ns=stamp,
                target_id=self.get_parameter('shared_target_id').value or None,global_valid=False)
            if self.get_parameter('origin_verified').value:
                lat=self.get_parameter('origin_latitude').value;lon=self.get_parameter('origin_longitude').value;alt=self.get_parameter('origin_altitude').value
                if not -85<lat<85 or not -180<=lon<=180:raise ValueError('Invalid geographic origin')
                east,north,up=fix['position'];output.update(latitude=lat+math.degrees(north/6378137),longitude=lon+math.degrees(east/(6378137*math.cos(math.radians(lat)))),altitude=alt+up,global_valid=True)
        except Exception as exc:output.update(valid=False,reason=str(exc))
        self.pub.publish(String(data=json.dumps(output,allow_nan=False)))

def main(args=None):
    run_node(Geolocator, args=args)


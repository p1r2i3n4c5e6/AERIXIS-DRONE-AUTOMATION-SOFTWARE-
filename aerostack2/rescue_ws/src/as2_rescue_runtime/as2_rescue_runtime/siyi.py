"""SIYI A8 SDK framing, checked against manufacturer v1.2 example packets.
Source: https://www.rcdrive.ru/doc/SIYI-A8-mini-instrukciya.pdf (SIYI-authored manual).
"""
import binascii
import struct

def encode(command,payload=b'',sequence=0):
    if len(payload)>1024:raise ValueError('Payload too long')
    frame=b'\x55\x66'+struct.pack('<BHHB',1,len(payload),sequence & 65535,command)+payload
    return frame+struct.pack('<H',binascii.crc_hqx(frame,0))

def decode(frame):
    if len(frame)<10 or frame[:2]!=b'\x55\x66':raise ValueError('Invalid SIYI header')
    control,length,sequence,command=struct.unpack('<BHHB',frame[2:8])
    if length>1024 or len(frame)!=length+10:raise ValueError('Invalid SIYI length')
    if binascii.crc_hqx(frame[:-2],0)!=struct.unpack('<H',frame[-2:])[0]:raise ValueError('Invalid SIYI CRC')
    return dict(control=control,sequence=sequence,command=command,payload=frame[8:-2])

def attitude(payload):
    if len(payload)!=12:raise ValueError('Attitude must contain six signed 16-bit values')
    return dict(zip(('yaw','pitch','roll','yaw_rate','pitch_rate','roll_rate'),[x/10 for x in struct.unpack('<6h',payload)]))

def rotation(yaw,pitch):
    if not all(isinstance(x,int) and -100<=x<=100 for x in (yaw,pitch)):raise ValueError('Rotation outside SDK range')
    return struct.pack('<bb',yaw,pitch)

class CenteringController:
    def __init__(self,gain=20,limit=15,slew=3,deadband=.04):
        self.gain,self.limit,self.slew,self.deadband=gain,limit,slew,deadband;self.previous=[0,0]
    def update(self,error,healthy=True):
        import math
        if not healthy or len(error)!=2 or not all(math.isfinite(x) for x in error):
            self.previous=[0,0];return (0,0)
        output=[]
        for old,x in zip(self.previous,error):
            target=0 if abs(x)<self.deadband else max(-self.limit,min(self.limit,self.gain*x))
            output.append(round(max(old-self.slew,min(old+self.slew,target))))
        self.previous=output;return tuple(output)

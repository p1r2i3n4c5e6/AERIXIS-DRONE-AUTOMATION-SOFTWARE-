"""Bounded authenticated fragmentation for low-bandwidth radios.
Transport-neutral: a serial/LoRa modem or MAVLink extension carries each frame.
"""
import hashlib
import hmac
import json
import secrets
import struct
import zlib

HEADER=struct.Struct('<4sQHHIH')
CHUNK=180
MAX_PARTS=800

def encode(value,key,now,ttl=60):
    if len(key)<32:raise ValueError('Radio authentication key must contain at least 32 bytes')
    raw=json.dumps(value,allow_nan=False,separators=(',',':')).encode()
    if len(raw)>1_000_000:raise ValueError('Radio object too large')
    data=zlib.compress(raw);parts=[data[i:i+CHUNK] for i in range(0,len(data),CHUNK)]
    if len(parts)>MAX_PARTS or not 1<=ttl<=300:raise ValueError('Transfer exceeds radio bounds')
    transfer=secrets.randbits(64);frames=[]
    for index,part in enumerate(parts):
        frame=HEADER.pack(b'A2R1',transfer,index,len(parts),int(now+ttl),len(part))+part
        frames.append(frame+hmac.new(key,frame,hashlib.sha256).digest()[:16])
    return frames

class Receiver:
    def __init__(self,key,clock):
        if len(key)<32:raise ValueError('Radio authentication key too short')
        self.key=key;self.clock=clock;self.pending={};self.completed={}
    def accept(self,packet):
        now=self.clock();self.pending={k:v for k,v in self.pending.items() if v['expires']>=now};self.completed={k:v for k,v in self.completed.items() if v>=now}
        if len(packet)<HEADER.size+16:return None
        magic,identity,index,count,expires,length=HEADER.unpack(packet[:HEADER.size])
        if magic!=b'A2R1' or not 1<=count<=MAX_PARTS or index>=count or not 0<length<=CHUNK or not now<=expires<=now+301:return None
        size=HEADER.size+length
        if len(packet)<size+16 or not hmac.compare_digest(packet[size:size+16],hmac.new(self.key,packet[:size],hashlib.sha256).digest()[:16]):return None
        if identity in self.completed:return None
        if identity not in self.pending:
            if len(self.pending)>=4:return None
            self.pending[identity]=dict(count=count,expires=expires,parts={})
        transfer=self.pending[identity]
        if transfer['count']!=count or transfer['expires']!=expires:return None
        old=transfer['parts'].get(index)
        part=packet[HEADER.size:size]
        if old is not None and old!=part:self.pending.pop(identity,None);return None
        transfer['parts'][index]=part
        if len(transfer['parts'])!=count:return None
        self.pending.pop(identity)
        try:
            stream=zlib.decompressobj();raw=stream.decompress(b''.join(transfer['parts'][i] for i in range(count)),1_000_001)
            if len(raw)>1_000_000 or stream.unconsumed_tail or not stream.eof or stream.unused_data:return None
            value=json.loads(raw)
            if not isinstance(value,dict):return None
            self.completed[identity]=expires
            if len(self.completed)>2048:self.completed.pop(next(iter(self.completed)))
            return value
        except (ValueError,zlib.error):return None

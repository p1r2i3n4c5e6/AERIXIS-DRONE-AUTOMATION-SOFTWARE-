"""Authenticated request/reply over bounded radio fragments; no automatic command retries."""
from concurrent.futures import ThreadPoolExecutor
import json
import threading
import time
import uuid
import urllib.parse
import urllib.request
import urllib.error
try:
    from .radio import encode,Receiver
except ImportError:
    from radio_codec import encode,Receiver
MESSAGE_TYPE=42001

class RadioClient:
    def __init__(self,send):
        self.send=send;self.lock=threading.RLock();self.bus=threading.Lock();self.pending={}
    def receive(self,system,frame):
        with self.lock:
            for key,item in list(self.pending.items()):
                if item['system']!=system:continue
                value=item['receiver'].accept(frame)
                if value and value.get('request_id')==key:
                    item['response']=value;item['event'].set()
    def request(self,profile,path,body=None,timeout=25):
        system=int(profile['system_id']);key=profile['token'].encode();identity=uuid.uuid4().hex
        item=dict(system=system,event=threading.Event(),receiver=Receiver(key,time.time),response=None)
        request=dict(request_id=identity,drone_id=profile['drone_id'],path=path,body=body)
        frames=encode(request,key,time.time(),ttl=60)
        if len(frames)>200:raise ValueError('XML/request too large for this radio profile; reduce mission size')
        # One in-flight transfer on a shared serial transport; polling cannot interleave uploads.
        with self.bus:
            with self.lock:self.pending[identity]=item
            try:
                for frame in frames:self.send(system,frame);time.sleep(.04)
                if not item['event'].wait(timeout):raise ValueError('Radio acknowledgement timed out; inspect onboard status before retrying')
                response=item['response']
                if response.get('drone_id')!=profile['drone_id']:raise ValueError('Radio drone identity mismatch')
                if 'error' in response:raise ValueError(response['error'])
                return response['result']
            finally:
                with self.lock:self.pending.pop(identity,None)

class RadioRelay:
    ALLOWED={'/api/status','/api/registry','/api/mission/stage','/api/mission/start','/api/mission/cancel','/api/mission/reconcile','/api/avoidance'}
    def __init__(self,drone_id,key,send,endpoint='http://127.0.0.1:8790'):
        parsed=urllib.parse.urlparse(endpoint)
        if parsed.scheme!='http' or parsed.hostname not in ('127.0.0.1','localhost') or parsed.path or parsed.username or parsed.password or parsed.query or parsed.fragment:
            raise ValueError('Radio relay only forwards to a loopback mission manager')
        self.drone_id=drone_id;self.key=key;self.send=send;self.endpoint=endpoint
        self.receiver=Receiver(key,time.time);self.executor=ThreadPoolExecutor(max_workers=1);self.lock=threading.Lock();self.pending=set();self.cache={}
    def receive(self,frame):
        with self.lock:
            value=self.receiver.accept(frame)
            if not value:return
            identity=value.get('request_id')
            if not isinstance(identity,str) or len(identity)!=32 or value.get('drone_id')!=self.drone_id:return
            fingerprint=json.dumps(value,sort_keys=True)
            if identity in self.cache:
                old,response=self.cache[identity]
                if old==fingerprint:self.executor.submit(self.reply,response)
                return
            if identity in self.pending or len(self.pending)>=4:return
            self.pending.add(identity);self.executor.submit(self.forward,value,fingerprint)
    def reply(self,value):
        frames=encode(value,self.key,time.time(),ttl=60)
        if len(frames)>200:frames=encode(dict(request_id=value['request_id'],drone_id=self.drone_id,error='Response exceeds radio profile limit'),self.key,time.time())
        for frame in frames:self.send(frame);time.sleep(.04)
    def forward(self,value,fingerprint):
        response=dict(request_id=value['request_id'],drone_id=self.drone_id)
        try:
            path=value['path'];body=value.get('body');base,_,query=path.partition('?')
            # Only the status poll takes a query (its delta cursors); nothing else may.
            if base not in self.ALLOWED or (query and base!='/api/status') or (base in ('/api/status','/api/registry'))!=(body is None):raise ValueError('Unsupported radio API request')
            req=urllib.request.Request(self.endpoint+path,data=None if body is None else json.dumps(body).encode(),headers={'Authorization':'Bearer '+self.key.decode(),'Content-Type':'application/json'})
            try:
                with urllib.request.urlopen(req,timeout=10) as stream:result=json.load(stream)
            except urllib.error.HTTPError as exc:
                result=json.loads(exc.read(4096));raise ValueError(result.get('error','Onboard HTTP failure')) from None
            response['result']=result
        except Exception as exc:response['error']=str(exc)[:300]
        with self.lock:
            self.pending.discard(value['request_id']);self.cache[value['request_id']]=(fingerprint,response)
            if len(self.cache)>256:self.cache.pop(next(iter(self.cache)))
        self.reply(response)
    def close(self):self.executor.shutdown(wait=False,cancel_futures=True)

"""Laptop-side fleet directory; requests execution on authenticated onboard endpoints."""
from concurrent.futures import ThreadPoolExecutor
import copy
import ipaddress
import json
import os
from pathlib import Path
import re
import ssl
import threading
import time
import urllib.error
import urllib.parse
import urllib.request


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        raise ValueError('Onboard redirects are not allowed')


BT_STATUS={'I':'IDLE','R':'RUNNING','S':'SUCCESS','F':'FAILURE','K':'SKIPPED'}
BT_DISPLAY={'p':'pending','r':'running','s':'success','f':'failed','k':'skipped','c':'cancelled','u':'unknown'}


def merge_status(previous,delta):
    """Rebuild the full status shape from an onboard lite delta and the last full view."""
    if not delta.get('lite'):return delta
    previous=previous or {}
    data=dict(delta)
    seen=set();logs=[]
    for entry in (previous.get('logs') or [])+delta.get('logs',[]):
        if entry.get('id') in seen:continue
        seen.add(entry.get('id'))
        # Onboard folds repeats into one line ("… (×N)") under a new id: replace, don't append.
        if entry.get('base') and logs and logs[-1].get('base',logs[-1].get('message'))==entry['base']:logs[-1]=entry
        else:logs.append(entry)
    data['logs']=logs[-200:]
    if 'bt_nodes' not in delta:
        nodes=copy.deepcopy(previous.get('bt_nodes') or [])
        codes=delta.get('bt_status','')
        for index,node in enumerate(sorted(nodes,key=lambda n:n['node_uid'])):
            if 2*index+1<len(codes):
                node['status']=BT_STATUS.get(codes[2*index],'IDLE');node['display_status']=BT_DISPLAY.get(codes[2*index+1],'pending')
        data['bt_nodes']=nodes
    for key in ('mission','execution','registry_hash','execution_location'):
        if key not in delta and key in previous:data[key]=previous[key]
    ros=dict(delta.get('ros') or {})
    if 'nodes' not in ros:ros['nodes']=(previous.get('ros') or {}).get('nodes')
    data['ros']=ros
    return data


def status_path(previous):
    """Ask only for what changed since the last merged view (cursors are opaque to the GCS)."""
    previous=previous or {}
    query={'lite':'1','log':previous.get('log_cursor',''),
           'bt':previous.get('bt_key','') if previous.get('bt_nodes') else '',
           'ros':(previous.get('ros') or {}).get('nodes_key','') if (previous.get('ros') or {}).get('nodes') is not None else '',
           'st':previous.get('static_key','') if 'execution' in previous else ''}
    return '/api/status?'+urllib.parse.urlencode(query)


class Fleet:
    def __init__(self, path, radio=None):
        self.radio=radio
        self.path=Path(path)
        self.lock=threading.RLock()
        self.profiles={}
        self.states={}
        self.stop=threading.Event()
        if self.path.exists():
            self.profiles=json.loads(self.path.read_text())
        self.pool=ThreadPoolExecutor(max_workers=8)
        self.thread=threading.Thread(target=self._poll,daemon=True)
        self.thread.start()

    def _save(self):
        self.path.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
        temp=self.path.with_suffix('.tmp')
        with open(temp,'w') as stream:
            os.chmod(temp,0o600);json.dump(self.profiles,stream)
        os.replace(temp,self.path)

    def configure(self,data):
        drone_id=data.get('drone_id','')
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]{0,63}',drone_id):raise ValueError('Use a ROS drone ID such as drone0')
        transport=data.get('transport','https')
        if transport not in ('https','mavlink'):raise ValueError('Unsupported mission transport')
        system_id=int(data.get('system_id') or 0)
        if transport=='mavlink':
            if self.radio is None or not 1<=system_id<=254:raise ValueError('MAVLink requires an available telemetry link and unique system ID 1–254')
            url=f'mavlink://{system_id}'
        else:
            url=data.get('endpoint','').rstrip('/')
        parsed=urllib.parse.urlparse(url)
        if transport!='mavlink' and (parsed.scheme not in ('http','https') or not parsed.hostname or parsed.username or
                parsed.password or parsed.path or parsed.query or parsed.fragment):
            raise ValueError('Use an onboard base URL with host and port only')
        if parsed.scheme == 'http' and parsed.hostname not in ('localhost','127.0.0.1','::1'):
            raise ValueError('Remote aircraft require HTTPS; HTTP is supported only for local bench tests')
        token=data.get('token','')
        if not isinstance(token,str) or len(token)<32:raise ValueError('Enter the onboard token (at least 32 characters)')
        label=str(data.get('label') or drone_id)[:60]
        with self.lock:
            if transport=='mavlink' and any(p.get('system_id')==system_id and p.get('transport')=='mavlink' and key!=drone_id for key,p in self.profiles.items()):
                raise ValueError('MAVLink system ID is already assigned to another aircraft')
            old=self.states.get(drone_id,{})
            if (old.get('data') or {}).get('active'):raise ValueError('Cannot change connection while that drone is executing')
            self.profiles[drone_id]=dict(drone_id=drone_id,label=label,endpoint=url,token=token,transport=transport,system_id=system_id)
            self.states[drone_id]={'connection':'CONNECTING','last_ok':None,'error':'','data':None}
            self._save()
        return {'ok':True}

    def _request(self,profile,path,body=None):
        if profile.get('transport')=='mavlink':
            # Status polls share one radio with every aircraft: an absent drone must not
            # hold the link for the full command timeout and freeze everyone else's status.
            return self.radio.request(profile,path,body,timeout=6 if path.startswith('/api/status') else 25)
        context=ssl.create_default_context()
        ca_file=os.environ.get('AS2_FLEET_CA_FILE')
        if ca_file:context.load_verify_locations(cafile=ca_file)
        opener=urllib.request.build_opener(NoRedirect(),urllib.request.HTTPSHandler(context=context))
        request=urllib.request.Request(profile['endpoint']+path,
            data=None if body is None else json.dumps(body).encode(),
            headers={'Authorization':'Bearer '+profile['token'],'Content-Type':'application/json'},
            method='GET' if body is None else 'POST')
        try:
            with opener.open(request,timeout=3) as response:
                raw=response.read(2_000_001)
                if len(raw)>2_000_000:raise ValueError('Onboard response exceeded size limit')
                return json.loads(raw)
        except urllib.error.HTTPError as exc:
            try:message=json.loads(exc.read(4096)).get('error',str(exc.code))
            except Exception:message=f'Onboard HTTP {exc.code}'
            raise ValueError(message) from None

    def _one(self,profile):
        now=time.monotonic()
        with self.lock:
            state=self.states.get(profile['drone_id'],{})
            # Back off aircraft that are not answering; retry every 15 s.
            if state.get('connection') in ('ERROR','DISCONNECTED') and now-state.get('attempt',0)<15:return
            # The radio is shared: poll a flying aircraft every second, an idle one every 2.5 s.
            active=(state.get('data') or {}).get('active')
            if state.get('connection')=='CONNECTED' and now-state.get('attempt',0)<(0.9 if active else 2.4):return
            self.states.setdefault(profile['drone_id'],{'connection':'CONNECTING','last_ok':None,'error':'','data':None})['attempt']=now
            previous=state.get('data')
        try:
            data=merge_status(previous,self._request(profile,status_path(previous)))
            if data.get('drone_id')!=profile['drone_id']:raise ValueError('Connected endpoint reports a different drone ID')
            update={'connection':'CONNECTED','last_ok':time.monotonic(),'error':'','data':data}
        except Exception as exc:
            update={'connection':'ERROR' if isinstance(exc,ValueError) else 'DISCONNECTED','error':str(exc)[:240]}
        with self.lock:
            if self.profiles.get(profile['drone_id'])==profile:
                previous=self.states.get(profile['drone_id'],{'last_ok':None,'data':None})
                self.states[profile['drone_id']]={**previous,**update}

    def _poll(self):
        while not self.stop.is_set():
            with self.lock:profiles=copy.deepcopy(list(self.profiles.values()))
            list(self.pool.map(self._one,profiles))
            self.stop.wait(0.3)

    def snapshot(self):
        with self.lock:
            rows=[]
            for key,p in self.profiles.items():
                state=copy.deepcopy(self.states.get(key,{'connection':'CONNECTING','last_ok':None,'error':'','data':None}))
                last=state.pop('last_ok',None)
                age=None if last is None else time.monotonic()-last
                state['age_s']=age
                state.setdefault('connection','CONNECTING');state.pop('attempt',None)
                state['fresh']=age is not None and age<6 and state['connection']=='CONNECTED'
                if not state['fresh'] and state['connection']=='CONNECTED':state['connection']='STALE'
                rows.append(dict(drone_id=key,label=p['label'],endpoint=p['endpoint'],**state))
            return {'drones':rows,'server_time':time.time()}

    def command(self,data):
        key=data.get('drone_id')
        with self.lock:
            if key not in self.profiles:raise ValueError('Add and connect this aircraft first')
            profile=copy.deepcopy(self.profiles[key])
        action=data.get('action')
        paths={'stage':'/api/mission/stage','start':'/api/mission/start',
               'cancel':'/api/mission/cancel','reconcile':'/api/mission/reconcile',
               'avoidance':'/api/avoidance'}
        if action not in paths:raise ValueError('Unsupported fleet operation')
        body={k:v for k,v in data.items() if k not in ('action',)}
        identity=self._request(profile,'/api/status?lite=1&log=&bt=&ros=')
        if identity.get('drone_id') != key:
            raise ValueError('Connected endpoint reports a different drone ID; command rejected')
        result=self._request(profile,paths[action],body)
        if action=='stage':
            with self.lock:
                state=self.states.setdefault(key,{'connection':'CONNECTING','last_ok':None,'error':'','data':None})
                state['xml_bytes']=len(body.get('xml') or '');state['bundle_bytes']=len(json.dumps(body.get('route_bundle') or {}))
        self._one(profile)
        return result

    def use(self,path):
        """Switch to another aircraft list (Simulation and Field keep separate lists, 2026-10-31:
        both use drone0, and the simulated drone's token must not replace the real one)."""
        path=Path(path)
        with self.lock:
            if path==self.path:return {'ok':True,'changed':False,'list':path.name}
            if any((s.get('data') or {}).get('active') for s in self.states.values()):
                raise ValueError('An aircraft is executing a mission: switch after it has finished')
            self.path=path
            self.profiles=json.loads(path.read_text()) if path.exists() else {}
            self.states={}
        return {'ok':True,'changed':True,'list':path.name}

    def remove(self,drone_id):
        with self.lock:
            if drone_id not in self.profiles:raise ValueError('Unknown aircraft')
            if (self.states.get(drone_id,{}).get('data') or {}).get('active'):raise ValueError('Cannot remove an aircraft while it is executing')
            self.profiles.pop(drone_id);self.states.pop(drone_id,None);self._save()
        return {'ok':True}

    def close(self):
        self.stop.set();self.thread.join(timeout=5);self.pool.shutdown(wait=False,cancel_futures=True)

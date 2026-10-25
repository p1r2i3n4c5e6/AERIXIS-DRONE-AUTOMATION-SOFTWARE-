"""Flight-independent durable checkpoints and fail-closed payload transactions."""
import copy
import json
from pathlib import Path
from as2_mission_manager.manager import atomic_json

class Journal:
    def __init__(self,path):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
        self.state=json.loads(self.path.read_text()) if self.path.exists() else {'missions':{},'releases':{}}
        for item in self.state['releases'].values():
            if item['state']=='REQUESTED':item['state']='UNCERTAIN'
        self.save()
    def save(self):atomic_json(self.path,self.state)
    def checkpoint(self,mission,segment,pose=None,completed=False):
        record=self.state['missions'].setdefault(mission,{'completed':[],'segment':None})
        record.update(segment=segment,pose=pose)
        if completed and segment not in record['completed']:record['completed'].append(segment)
        self.save();return copy.deepcopy(record)
    def reserve_release(self,mission,target,payload_id,transaction):
        if not all(isinstance(v,str) and v and len(v)<=200 for v in (mission,target,payload_id,transaction)):
            raise ValueError('Release requires mission, target, physical payload ID and transaction ID')
        old=self.state['releases'].get(transaction)
        if old:
            if (old['mission'],old['target'],old['payload_id'])!=(mission,target,payload_id):raise ValueError('Release ID identity mismatch')
            return False,copy.deepcopy(old)
        for item in self.state['releases'].values():
            if item['payload_id']==payload_id or (item['mission']==mission and item['target']==target):
                raise ValueError('Payload or target already has a release transaction; reconcile instead of retrying')
        item=dict(mission=mission,target=target,payload_id=payload_id,state='REQUESTED')
        self.state['releases'][transaction]=item;self.save();return True,copy.deepcopy(item)
    def release_result(self,transaction,confirmed=False):
        item=self.state['releases'][transaction]
        if item['state']=='CONFIRMED':return
        item['state']='CONFIRMED' if confirmed else 'UNCERTAIN';self.save()

    def mark_flight_authority(self, mission):
        record = self.state.get('runtime', {})
        if record.get('needs_reconcile') and record.get('mission_id') != mission:
            raise ValueError('Previous runtime authority requires ground reconciliation')
        if not record.get('needs_reconcile'):
            self.state['runtime'] = dict(needs_reconcile=True, mission_id=mission)
            self.save()
    def reconcile_runtime(self):
        # Payload transactions are deliberately retained, including UNCERTAIN ones.
        self.state['runtime'] = dict(needs_reconcile=False, mission_id=None)
        self.save()

"""Durable mission ownership and execution on one companion computer."""
from collections import Counter, deque
import copy
import fcntl
import hashlib
import json
import re
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

from .events import EventState, PREFIX
from .validation import IDENTIFIER, validate_xml


def atomic_json(path, value):
    temporary = path.with_suffix('.tmp')
    with open(temporary, 'w', encoding='utf-8') as handle:
        os.chmod(temporary, 0o600)
        json.dump(value, handle, allow_nan=False)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)
    directory = os.open(str(path.parent), os.O_DIRECTORY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def load_registry(runner):
    result = subprocess.run([str(runner),'--ros-args','-p','describe_nodes:=true'],
                            capture_output=True,text=True,timeout=20)
    for line in result.stdout.splitlines():
        if line.startswith('AS2_BT_REGISTRY ') and result.returncode == 0:
            return json.loads(line.split(' ',1)[1])
    raise RuntimeError('Cannot read node registry from installed runner; rebuild as2_behavior_tree')


# The executor's human-readable "[time]: Node   RUNNING -> SUCCESS" lines duplicate its events.
TRANSITION = re.compile(r'\]: \S+\s+\S*(IDLE|RUNNING|SUCCESS|FAILURE|SKIPPED)\S*\s+->\s+')
TRANSITION_ANSI = re.compile(r'\x1b\[[0-9;]*m')


class MissionManager:
    def __init__(self, drone_id, state_dir, runner, registry, observer=None, allow_flight=False):
        if not IDENTIFIER.fullmatch(drone_id):
            raise ValueError('Invalid drone namespace')
        self.drone_id, self.runner = drone_id, str(Path(runner).resolve())
        self.directory = Path(state_dir).resolve()
        self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.owner = open(self.directory/'owner.lock', 'a+')
        try:
            fcntl.flock(self.owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.owner.close()
            raise RuntimeError('Another mission manager owns this state directory')
        self.lock = threading.RLock()
        self.registry, self.observer, self.allow_flight = registry, observer, allow_flight
        self.registry_hash = hashlib.sha256(json.dumps(registry,sort_keys=True).encode()).hexdigest()
        self.process = None
        self.worker_active = False
        self.worker = None
        self.auto_release_retry = 0.0
        self.events = None
        self.logs = deque(maxlen=200)
        self.file = self.directory/'state.json'
        self.record = dict(drone_id=drone_id, mission=None, execution=None, hold_lockout=False)
        if self.file.exists():
            self.record = json.loads(self.file.read_text())
            if self.record.get('drone_id') != drone_id:
                raise ValueError('State directory belongs to another aircraft')
            execution = self.record.get('execution')
            if execution and execution.get('state') in ('STARTING','RUNNING','CANCELLING'):
                execution.update(state='INTERRUPTED', detail='Manager restarted; execution must be reconciled')
                # Never assume the child or vehicle stopped when its owner disappeared.
                self.record['hold_lockout'] = True
                atomic_json(self.file,self.record)
        # AI avoidance starts OFF after every (re)start: switching it on is a deliberate GCS choice
        # per session. The latched topic also tells a restarted runtime.
        self.record['avoidance']=False
        if self.observer is not None and hasattr(self.observer,'set_avoidance'):
            self.observer.set_avoidance(self.record['avoidance'])
        self.log('INFO','Onboard mission manager ready')

    def log(self, level, message):
        with self.lock:
            now = time.time()
            last = self.logs[-1] if self.logs else None
            # A reactive tree can repeat the same transition many times a second: keep one
            # line with a count (fewer radio bytes, and the real reason stays visible).
            if last and last['level'] == level and last.get('base', last['message']) == message and now - last['time'] < 5:
                last['repeat'] = last.get('repeat', 1) + 1
                last['base'] = message
                last['message'] = f'{message} (×{last["repeat"]})'
                last['time'] = now
                last['id'] = uuid.uuid4().hex  # a changed line must reach the GCS delta again
                return
            self.logs.append(dict(id=uuid.uuid4().hex, time=now, level=level, message=message))

    def active(self):
        return self.worker_active or (self.process is not None and self.process.poll() is None)

    def stage(self, data):
        with self.lock:
            if self.active():
                raise ValueError('Cannot replace a mission while its tree is running')
            if data.get('drone_id') != self.drone_id:
                raise ValueError('Mission is addressed to another drone')
            summary = validate_xml(data.get('xml'),self.registry)
            if data.get('sha256') != summary['sha256']:
                raise ValueError('XML checksum mismatch')
            route_bundle=data.get('route_bundle')
            operator_payload=data.get('operator_payload')
            # Per-mission emergency choice for the onboard FC guardian (AS2 lost / GCS emergency).
            emergency_action=data.get('emergency_action')
            if emergency_action is not None:
                emergency_action=str(emergency_action).strip().upper()
                if emergency_action not in ('RTL','LAND'):raise ValueError('emergency_action must be RTL or LAND')
            if operator_payload is not None:
                if not isinstance(operator_payload,dict) or operator_payload.get('confirmed_loaded') is not True:
                    raise ValueError('Explicit operator load confirmation is required')
                payload_id=operator_payload.get('payload_id')
                if not isinstance(payload_id,str) or not IDENTIFIER.fullmatch(payload_id):raise ValueError('Enter a unique payload ID')
                if not self.observer or not self.observer.safe_on_ground():raise ValueError('Operator load confirmation requires fresh disarmed ground state')
            if route_bundle is not None:
                from as2_rescue_planner.mission import validate_route_bundle
                validate_route_bundle(route_bundle,self.drone_id,data['xml'])
            mission_id = uuid.uuid4().hex
            xml_path = self.directory/(mission_id+'.xml')
            with open(xml_path,'x',encoding='utf-8') as stream:
                os.chmod(xml_path,0o600)
                stream.write(data['xml']); stream.flush(); os.fsync(stream.fileno())
            mission = dict(id=mission_id,revision=summary['sha256'],registry_hash=self.registry_hash,
                           state='VALIDATED',uploaded_at=time.time(),**summary)
            if route_bundle is not None:
                atomic_json(self.directory/(mission_id+'.bundle.json'),dict(
                    mission_id=mission_id,xml_sha256=summary['sha256'],bundle=route_bundle))
                mission['route_bundle_sha256']=route_bundle['bundle_sha256']
            if emergency_action is not None:
                mission['emergency_action']=emergency_action
            if operator_payload is not None:
                atomic_json(self.directory/(mission_id+'.payload.json'),dict(
                    mission_id=mission_id,payload_id=operator_payload['payload_id'],confirmed_loaded=True,
                    confirmed_at=time.time(),source='operator',physical_confirmation=False))
                mission['operator_payload_id']=operator_payload['payload_id']
            self.record['mission'] = mission
            atomic_json(self.file,self.record)
            self.log('INFO','Mission received, checksum verified and node schema validated')
            return copy.deepcopy(mission)

    def start(self,data):
        with self.lock:
            mission = self.record.get('mission')
            if not mission or data.get('mission_id') != mission['id'] or data.get('revision') != mission['revision']:
                raise ValueError('Start does not match the staged mission revision')
            command_id = data.get('command_id','')
            if not isinstance(command_id,str) or not 8 <= len(command_id) <= 100:
                raise ValueError('A unique command_id is required')
            previous = self.record.get('execution')
            if previous and previous.get('mission_id') == mission['id']:
                if previous.get('command_id') == command_id:
                    return copy.deepcopy(previous)
                raise ValueError('This mission revision has already been started; upload a new mission to rerun')
            if self.active() or self.record['hold_lockout']:
                raise ValueError('Execution or physical-state reconciliation is still pending')
            if mission['registry_hash'] != self.registry_hash:
                raise ValueError('Node registry changed; upload and validate again')
            path = self.directory/(mission['id']+'.xml')
            if hashlib.sha256(path.read_bytes()).hexdigest() != mission['revision']:
                raise ValueError('Stored XML checksum mismatch')
            if mission.get('route_bundle_sha256'):
                from as2_rescue_planner.mission import validate_route_bundle
                artifact=json.loads((self.directory/(mission['id']+'.bundle.json')).read_text())
                if artifact['bundle']['bundle_sha256']!=mission['route_bundle_sha256']:
                    raise ValueError('Stored route bundle revision changed')
                validate_route_bundle(artifact['bundle'],self.drone_id,path.read_text())
            if mission['requires_flight']:
                if not self.allow_flight:
                    raise ValueError('This executor is in observation-only mode; flight is disabled')
                if self.observer is None or not self.observer.safe_on_ground():
                    raise ValueError('Fresh connected, disarmed ground-state telemetry is required')
                abort = self.observer.runtime_abort() if hasattr(self.observer,'runtime_abort') else ''
                if abort:
                    raise ValueError(f'AERIXIS core holds an abort ({abort}): check the aircraft, then Reconcile')
            execution = dict(mission_id=mission['id'],revision=mission['revision'],command_id=command_id,
                             state='STARTING', started_at=time.time(), detail='Starting onboard BT')
            self.record['execution'] = execution
            self.record['hold_lockout'] = mission['requires_flight']
            atomic_json(self.file,self.record)
            self.events = EventState(mission['id'])
            if mission.get('emergency_action') and self.observer is not None and hasattr(self.observer,'set_emergency_action'):
                self.observer.set_emergency_action(mission['emergency_action'])
                self.log('INFO','FC guardian emergency action for this mission: '+mission['emergency_action'])
            try:
                self.process = subprocess.Popen(
                    [self.runner,'--ros-args','-r',f'__ns:=/{self.drone_id}',
                     '-p',f'tree:={path}','-p',f'mission_id:={mission["id"]}'],
                    stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,
                    start_new_session=True)
                self.worker_active = True
                execution['pid'] = self.process.pid
                atomic_json(self.file,self.record)
                self.worker = threading.Thread(target=self._consume,args=(self.process,self.events),daemon=True)
                self.worker.start()
            except Exception as exc:
                execution.update(state='ERROR',detail=str(exc))
                atomic_json(self.file,self.record)
                raise
            return copy.deepcopy(execution)

    def _consume(self, process, events):
        try:
            # Persist raw events for post-flight inspection; constrain retention operationally.
            path = self.directory/(events.mission_id+'.events.jsonl')
            with open(path,'a',encoding='utf-8') as journal:
                os.chmod(path,0o600)
                for line in process.stdout:
                    with self.lock:
                        if events.accept(line.strip()):
                            journal.write(line[len(PREFIX):]);journal.flush()
                            execution = self.record['execution']
                            if execution['state'] == 'STARTING':
                                execution['state'] = 'RUNNING'
                            if events.outcome:
                                execution.update(state=events.outcome,detail=events.detail)
                                atomic_json(self.file,self.record)
                            event = json.loads(line[len(PREFIX):])
                            if event['event'] == 'finished':
                                self.log('INFO' if events.outcome=='SUCCESS' else 'ERROR',events.detail)
                            elif event.get('status')=='FAILURE' and event.get('previous_status')=='RUNNING':
                                # One line per failed node; the event journal keeps every transition.
                                self.log('WARN',f'{event["node_name"]} failed')
                        elif line.strip() and not TRANSITION.search(line):
                            # Executor diagnostics, e.g. "Rescue operation failed: <reason>".
                            text=TRANSITION_ANSI.sub('',line.strip())[-300:]
                            self.log('WARN' if ('fail' in text.lower() or '[WARN]' in text or '[ERROR]' in text) else 'INFO',text)
            result = process.wait()
            with self.lock:
                events.finish_process(result)
                self.record['execution'].update(state=events.outcome,detail=events.detail,finished_at=time.time())
                atomic_json(self.file,self.record)
        except Exception as exc:
            with self.lock:
                self.record['execution'].update(state='ERROR',detail=str(exc))
                self.record['hold_lockout'] = True
                atomic_json(self.file,self.record)
                self.log('ERROR',str(exc))
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
        finally:
            process.stdout.close()
            with self.lock:self.worker_active = False

    def cancel(self,data):
        with self.lock:
            execution = self.record.get('execution')
            if not execution or data.get('mission_id') != execution['mission_id'] or data.get('revision') != execution['revision']:
                raise ValueError('Cancel does not match the executing mission revision')
            if self.active():
                execution.update(state='CANCELLING',detail='Cancel requested; physical stop not verified')
                atomic_json(self.file,self.record)
                try:self.process.send_signal(signal.SIGINT)
                except ProcessLookupError:pass
                self.log('WARN','Cancellation requested; awaiting executor result')
                self._recover_after_cancel()
            return copy.deepcopy(execution)

    def set_avoidance(self,data):
        """GCS AI avoidance switch. Allowed at any time: OFF takes effect at once (the rest of the
        current leg is the usual GoTo), ON from the next leg."""
        if not isinstance(data.get('enabled'),bool):
            raise ValueError('enabled must be true or false')
        with self.lock:
            self.record['avoidance']=data['enabled']
            atomic_json(self.file,self.record)
            if self.observer is not None and hasattr(self.observer,'set_avoidance'):
                self.observer.set_avoidance(data['enabled'])
            self.log('WARN' if data['enabled'] else 'INFO','AI avoidance '+('ON from the next leg' if data['enabled'] else 'off'))
            return dict(enabled=data['enabled'])

    def _recover_after_cancel(self):
        """A cancelled flight must not stay hovering: ask the FC guardian for the mission's
        emergency action (PX4's own RTL or LAND). On the ground the guardian does nothing."""
        if self.observer is None or not hasattr(self.observer,'request_emergency'):
            return
        if self.observer.safe_on_ground():
            return
        action = (self.record.get('mission') or {}).get('emergency_action') or 'RTL'
        self.observer.request_emergency(action,'cancel')
        self.log('WARN',f'Cancelled in flight: FC guardian asked to {action}')

    def reconcile(self, note=''):
        with self.lock:
            if self.active() or self.observer is None or not self.observer.safe_on_ground():
                raise ValueError('Cannot clear lockout: require no active child and fresh disarmed ground telemetry')
            # A restarted owner cannot safely cancel a process based on a reused PID.
            prior = self.record.get('execution') or {}
            pid = prior.get('pid')
            if self.process is None and pid:
                try:
                    os.kill(pid,0)
                except ProcessLookupError:pass
                else:raise ValueError('Previous process may still exist; inspect onboard before reconciliation')
            runtime = self.observer.reconcile_runtime() if hasattr(self.observer,'reconcile_runtime') else ''
            self.record['hold_lockout'] = False
            atomic_json(self.file,self.record)
            self.log('INFO','Ground-state reconciliation complete'+note+'. '+runtime)
            return {'ok':True,'runtime':runtime}

    def _release_after_normal_landing(self):
        """Clear the lockout by itself after a mission that ended SUCCESS once the FC reports the
        aircraft disarmed on the ground. Failed, cancelled or interrupted runs keep the lockout for
        the operator's Reconcile: they need a look at the aircraft first."""
        execution = self.record.get('execution') or {}
        if not (self.record['hold_lockout'] and execution.get('state') == 'SUCCESS' and not self.active()
                and self.observer is not None and self.observer.safe_on_ground()):
            return
        now = time.monotonic()
        if now < self.auto_release_retry:
            return
        try:
            self.reconcile(' (automatic: mission ended normally, FC reports disarmed on the ground)')
        except ValueError as exc:
            self.auto_release_retry = now + 5.0
            self.log('WARN','Automatic ground reconciliation not possible yet: '+str(exc))

    def snapshot(self):
        with self.lock:
            self._release_after_normal_landing()
            state = copy.deepcopy(self.record)
            state.update(server_time=time.time(),flight_enabled=self.allow_flight,
                         execution_location='onboard',registry_hash=self.registry_hash,active=self.active(),
                         registry_counts=dict(Counter(n['category'] for n in self.registry.values())),
                         bt_nodes=list(copy.deepcopy(self.events.nodes).values()) if self.events else [],
                         logs=list(self.logs))
            execution = state.get('execution') or {}
            state['bt_running_count'] = sum(n['status']=='RUNNING' for n in state['bt_nodes']) if self.active() else 0
            state['ros'] = self.observer.snapshot() if self.observer else {'available':False,'nodes':None,'count':None,'error':'ROS observer unavailable'}
            state['can_start'] = bool(state['mission'] and not self.active() and not state['hold_lockout'] and
                                      execution.get('mission_id') != state['mission']['id'] and
                                      (not state['mission']['requires_flight'] or
                                       (self.allow_flight and self.observer and self.observer.safe_on_ground() and
                                        not (hasattr(self.observer,'runtime_abort') and self.observer.runtime_abort()))))
            return state

    def close(self):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                try:self.process.send_signal(signal.SIGINT)
                except ProcessLookupError:pass
        if self.process is not None:
            try:self.process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                self.log('ERROR','Executor still active after shutdown request; reconciliation required')
        if self.worker is not None:self.worker.join(timeout=2)
        self.owner.close()

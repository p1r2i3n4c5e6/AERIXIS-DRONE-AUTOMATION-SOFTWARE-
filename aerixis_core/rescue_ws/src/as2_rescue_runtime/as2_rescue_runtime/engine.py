"""Interruptible rescue operations. Driver methods must acknowledge measured outcomes."""
import math
from as2_rescue_planner.watch import station_segment
from .safety import Safety,finite
from .vision_ops import VisionOps,VISION_OPS,FLIGHT_RECOVERY_OPS,LONG_OPS,locked,overhead,limit_yaw

PREEMPTIBLE=('SurveyWaypoint','ScanArea','ResumeCoverage','TrackTarget','OrbitTarget','ApproachOverhead')
PREEMPTING=('CheckpointCoverage','PauseCoverage','ConfirmHold','RequestRecovery','ReturnToLaunch')
# as2_msgs/PlatformStatus DISARMED, LANDED: PX4 leaving OFFBOARD there is not a flight takeover.
ON_GROUND_STATES=(0,1)
MOTION_WAIT_S=5.0     # a vertical action waits this long for the previous motion to hand back
ARM_WAIT_S=8.0        # take-off fails when the FC is still disarmed after this
CLIMB_WAIT_S=20.0     # ... or has not climbed CLIMB_MIN_M after this
CLIMB_MIN_M=0.5
ROUTE_REFUSED_S=5.0   # a leg the route check keeps refusing fails with the reason after this
MOTION_STUCK_S=15.0   # a leg fails when the previous motion has not handed back after this
# While another mode flies the aircraft, the AS2 hover reference is re-frozen at the measured
# position this often, so a switch back to OFFBOARD holds near where the aircraft is.
REANCHOR_S=2.0

class Engine:
    def __init__(self,evidence,journal,driver,clock):
        self.evidence=evidence;self.journal=journal;self.driver=driver;self.clock=clock
        self.safety=Safety(evidence);self.active=None;self.results={};self.owner=None;self.target=None
        self.recovery_required=bool(journal.state.get('runtime',{}).get('needs_reconcile'))
        self.swarm_id='';self.incident_track=None;self.last_segment='';self.last_release=None;self.aborted=False;self.handled=set()
        self.handled_labels=set();self.incident_label=None;self.outbox=[];self.vision=VisionOps(self)
        self.home=None;self.leg=None;self.last_goal=None
        self.waiting=''   # what the active operation is waiting for (shown in rescue/state)
        # In-flight avoidance (avoidance.py); None or switched off = every leg is the usual GoTo.
        self.avoidance=None
        self.identity=''  # drone ID (namespace): fleet person-claim tie-break
        # External mode change: PX4 left OFFBOARD in flight without AS2 asking (RC pilot, FC
        # guardian, GCS RTL/LAND, PX4 failsafe). mode_change is the latched reason (cleared by
        # reconcile_ground); external is true while PX4 is still out of OFFBOARD.
        self.offboard_seen=False;self.external=False;self.mode_change=None;self.reanchored=None
    def mode_guard(self):
        """Watch PX4's OFFBOARD flag (platform/info) and hand the aircraft to whoever took it.

        On the change the running operation fails and motion stops; the mission stays aborted,
        so switching back to OFFBOARD never resumes a leg: AERIXIS core then only holds where the
        aircraft is and accepts return/land operations. Returns True while PX4 is out of
        OFFBOARD after a takeover. Stale platform evidence keeps the last decision."""
        p=self.evidence.get('platform',self.safety.limits.vehicle_max_age_s)
        if p is None:return self.external
        now=self.clock()
        if not p.get('armed'):
            self.offboard_seen=False;self.external=False;return False
        if p.get('offboard'):
            if self.external:
                # Back in OFFBOARD: freeze the hover reference here, not where the takeover began.
                self.external=False;self.driver.reanchor();self.reanchored=now
            self.offboard_seen=True;return False
        if not self.offboard_seen or p.get('state') in ON_GROUND_STATES:return self.external
        if not self.external:
            self.external=True;self.aborted=True;self.reanchored=now
            mode=p.get('px4_mode')
            self.mode_change=('EXTERNAL_MODE_CHANGE: PX4 left OFFBOARD%s; AERIXIS core stopped its operation and '
                'does not resume it'%(' (now %s)'%mode if mode and mode!='OFFBOARD' else ''))
            self.driver.stop();self.driver.track(None)
        elif now-self.reanchored>=REANCHOR_S:
            self.driver.reanchor();self.reanchored=now
        return True
    def geotags(self):
        return self.journal.state.setdefault('geotags',{}).get(self.owner or '',[])
    def add_geotag(self,tag):
        self.journal.state.setdefault('geotags',{}).setdefault(self.owner or '',[]).append(tag);self.journal.save()
    def set_home(self,vehicle):
        self.home=dict(position=list(vehicle['position']),yaw=vehicle.get('yaw'),frame_id=vehicle.get('frame_id'))
        self.journal.state.setdefault('home',{})[self.owner or '']=self.home;self.journal.save()
    def flags(self):
        flags=self.safety.evaluate();swarm=self.evidence.get('swarm');payload=self.evidence.get('payload')
        flags.update(AssignmentOwned=bool(swarm and swarm.get('owned') and swarm.get('target_id')==self.target and swarm.get('mission_id')==self.swarm_id),
            PeersFresh=bool(swarm and swarm.get('peers_fresh')),SeparationSafe=bool(swarm and swarm.get('separation_safe')),
            PayloadReleased=bool(self.last_release and self.journal.state['releases'][self.last_release]['state']=='CONFIRMED'))
        track=self.evidence.get('track')
        flags['RescuePending']=bool(flags['TrackConfirmed'] and track and track['track_id'] not in self.handled)
        vision=self.evidence.get('vision');vehicle=self.evidence.get('vehicle',self.safety.limits.vehicle_max_age_s)
        loaded=bool(payload and ((self.safety.limits.allow_operator_load and payload.get('operator_confirmed') is True and payload.get('consumed') is False)
            or (payload.get('present') is True and payload.get('latched') is True and payload.get('verified') is True)))
        flags.update(VisionHealthy=bool(vision and vision.get('fresh')),TargetLocked=locked(vision),
            HumanDetected=self.vision.human_detected(),OverheadTarget=overhead(vision),PayloadAvailable=loaded,
            FlightAvailable=bool(vehicle and vehicle.get('position_valid') is True and vehicle.get('connected') is True),
            # The one release authority the payload adapter accepts besides the delivery zone: an
            # overhead drop at nadir over a geotagged person (DropPayload), or over the claimed
            # delivery target steady at its geotag (DropAtTarget, mission 7).
            OverheadDropReady=bool(self.active and self.safety.limits.overhead_envelope_verified and (
                (self.active['operation']=='DropPayload' and overhead(vision)) or
                (self.active['operation']=='DropAtTarget' and self.vision.over_target()))))
        if flags['AbortRequested']:self.aborted=True
        if self.aborted or self.recovery_required:
            flags.update(AbortRequested=True,VehicleHealthy=False,SafeToRelease=False,
                reason='RESTART_RECONCILIATION_REQUIRED' if self.recovery_required else self.safety.abort_reason or self.mode_change or flags['reason'] or 'ABORT_LATCHED')
        return flags
    def resume_point(self,position):
        """Where coverage resumes: the measured position projected onto the planned leg."""
        leg=getattr(self,'leg',None)
        if not leg or not position:return position
        if getattr(self,'leg_is_entry',False):
            # Interrupted on the entry transit: resume at its end (the first lane's start, inside
            # the area). A point on the transit line may lie outside the area, and the route
            # check then refuses the resume (PX4 1.16 Gazebo SITL, launch 41 m outside, 2026-10-24).
            return list(leg[1])
        (ax,ay,_),(bx,by,bz)=leg;t=self.leg_fraction(position)
        return [ax+t*(bx-ax),ay+t*(by-ay),bz]
    def leg_fraction(self,position):
        """How far along the current planned leg a point is, 0 (start) .. 1 (end)."""
        (ax,ay,_),(bx,by,_)=self.leg;dx,dy=bx-ax,by-ay;length=dx*dx+dy*dy
        return 0.0 if length==0 else max(0.0,min(1.0,((position[0]-ax)*dx+(position[1]-ay)*dy)/length))
    def resume_ahead(self,saved,position):
        """Where ResumeCoverage rejoins the lane (2026-10-31): the point of the interrupted leg
        abeam of the aircraft now, or the saved interruption point if that is further along.
        Coverage never flies backwards along the lane to where it left it: the part between
        was in view while the aircraft went to the person beside the lane."""
        leg=getattr(self,'leg',None)
        if not leg or not saved or not position or getattr(self,'leg_is_entry',False):return saved
        (ax,ay,_),(bx,by,_)=leg;t=max(self.leg_fraction(saved),self.leg_fraction(position))
        return [ax+t*(bx-ax),ay+t*(by-ay),saved[2]]
    def motion_ready(self):
        return self.driver.idle and getattr(self.driver,'hover_future',None) is None
    def finish(self,status,reason=''):
        command=self.active;self.waiting=''
        if command:
            result=dict(command_id=command['command_id'],mission_id=command['mission_id'],run_id=command['run_id'],status=status,reason=reason,operation=command['operation'])
            self.results[command['command_id']]=result
            self.active=None
            return result
    def submit(self,command):
        now=self.clock()
        # Replays are refused by command identity; the stamp only rejects stale commands. 3 s
        # covers delivery on a loaded companion computer (a late valid command must not fail).
        if command.get('schema_version')!=1 or not finite(command.get('stamp')) or not -0.5<=now-command['stamp']<=3:
            raise ValueError('Expired or malformed rescue command (%.1fs old)'%(now-command['stamp']) if finite(command.get('stamp')) else 'Malformed rescue command')
        for field in ('command_id','mission_id','run_id','operation'):
            if not isinstance(command.get(field),str) or not 0<len(command[field])<=200:raise ValueError('Invalid command identity')
        identity=command['command_id'];mission=command['mission_id']
        if command['operation']=='CancelOperation':
            if self.active and identity==self.active['command_id'] and mission==self.active['mission_id'] and command['run_id']==self.active['run_id']:
                self.driver.stop();self.driver.track(None);return self.finish('FAILURE','CANCELLED; stop verification pending')
            return None
        if identity in self.results:
            result=self.results[identity]
            if result['mission_id']!=mission or result['run_id']!=command['run_id']:raise ValueError('Command ID reused across identities')
            return result
        if self.active and self.active['operation'] in PREEMPTIBLE and command['operation'] in PREEMPTING:
            # A detection interrupts coverage: the BT starts the pause before it halts the
            # running waypoint, so hand the runtime over instead of rejecting the pause.
            self.driver.stop();self.outbox.append(self.finish('FAILURE','PREEMPTED by '+command['operation']))
        if self.active:raise ValueError('Another rescue operation owns the runtime')
        if self.owner and self.owner!=mission:
            # A previous mission that ended normally hands over on the ground by itself; after an
            # abort or restart the operator's explicit reconcile is still required.
            if not self.aborted and not self.recovery_required:
                try:self.reconcile_ground()
                except ValueError:pass
            if self.owner and self.owner!=mission:
                raise ValueError('Runtime belongs to another mission; reconcile/restart on the ground before reassignment')
        if self.owner!=mission:self.vision.search_area=None;self.vision.release_claim();self.vision.yielded=None;self.vision.handled_points=[];self.vision.served_points=[];self.vision.delivered=None
        if command['operation']!='ScanArea':self.vision.watch=None  # a stopped-short station leg hands its note to ScanArea
        if command['operation'] in ('ReturnToLaunch','ManagedLand','RequestRecovery'):self.vision.release_claim()
        self.owner=mission
        timeout=command.get('timeout_s',30)
        limit=1800 if command['operation'] in LONG_OPS else 300
        if not finite(timeout) or not 0<timeout<=limit:raise ValueError('Invalid operation timeout')
        self.active=dict(command,deadline=now+timeout,started=False)
        self.target=command.get('target_id') or self.target
        self.swarm_id=command.get('swarm_id') or self.swarm_id
        return None
    def tick(self):
        external=self.mode_guard()
        flags=self.flags();cmd=self.active
        if not cmd:return None
        op=cmd['operation'];now=self.clock()
        try:
            if external and op!='ReportObservation':
                # Another mode flies the aircraft: no AS2 motion of any kind, recovery included.
                self.driver.stop();self.driver.track(None);return self.finish('FAILURE',self.mode_change)
            if now>cmd['deadline']:
                self.driver.stop();self.driver.track(None)
                if op in ('ReleasePayload','VerifyPayloadRelease') and self.last_release:
                    self.journal.release_result(self.last_release);self.aborted=True
                return self.finish('FAILURE','DEADLINE_EXCEEDED; release never automatically retried')
            recovery=op in ('RequestRecovery','PauseCoverage','ConfirmHold','ReportObservation')
            if self.aborted and not self.recovery_required and op in FLIGHT_RECOVERY_OPS:recovery=True
            if (self.aborted or self.recovery_required) and not recovery:
                self.driver.stop();self.driver.track(None);return self.finish('FAILURE',flags['reason'] or 'ABORT_LATCHED')
            if op in VISION_OPS:return self.vision.tick(cmd,flags,now)
            if op=='CheckpointCoverage':
                v=self.evidence.get('vehicle',self.safety.limits.vehicle_max_age_s) or {}
                self.journal.checkpoint(self.owner,self.last_segment,self.resume_point(v.get('position')));return self.finish('SUCCESS')
            if op in ('PauseCoverage','ConfirmHold','RequestRecovery'):
                if not cmd['started']:
                    self.driver.stop();self.driver.track(None);cmd['started']=True
                    if op=='RequestRecovery':self.aborted=True
                if self.driver.hold() and self.flags()['HoldConfirmed']:return self.finish('SUCCESS','Measured hold verified')
            elif op=='TrackPerson':
                if not flags['VehicleHealthy'] or not flags['HoldConfirmed']:raise ValueError('Tracking requires healthy aircraft and confirmed hold')
                track=self.evidence.get('track')
                if not track or not flags['TrackConfirmed']:raise ValueError('Target missing or predicted only')
                if not cmd['started']:cmd['track_id']=track['track_id'];self.incident_track=track['track_id'];cmd['started']=True
                if cmd['track_id']!=track['track_id']:raise ValueError('Track identity changed')
                self.driver.track(cmd['track_id'])
                g=self.evidence.get('gimbal')
                if g and g.get('oscillating'):raise ValueError('Gimbal oscillation latched')
                if g and flags['GimbalStable'] and g.get('tracking') and str(g.get('track_id'))==str(cmd['track_id']) and max(abs(x) for x in g.get('center_error',[99,99]))<.04:
                    cmd.setdefault('settled',now)
                    if now-cmd['settled']>=2:return self.finish('SUCCESS')
                else:cmd.pop('settled',None)
            elif op=='GeotagTarget':
                if not all(flags[k] for k in ('GeolocationValid','GimbalStable','HoldConfirmed')):raise ValueError('Geotag lacks stable, fresh, calibrated geometry')
                fix=self.evidence.get('fix')
                if not fix.get('global_valid'):raise ValueError('Verified geographic origin required for geotagging')
                self.target=fix.get('target_id') or self.target;self.driver.report(dict(kind='geotag',mission_id=self.owner,target_id=self.target,**fix));return self.finish('SUCCESS')
            elif op in ('ManagedTakeoff','ManagedLand'):
                ground=self.evidence.get('vehicle',self.safety.limits.vehicle_max_age_s) or {}
                if op=='ManagedLand' and not cmd['started'] and ground.get('armed') is False and ground.get('speed',1.0)<0.5:
                    # PX4 says disarmed (e.g. it never took off): there is nothing to land.
                    self.driver.stop();return self.finish('SUCCESS','Already on the ground (FC disarmed)')
                if not flags['VehicleHealthy' if op=='ManagedTakeoff' else 'FlightAvailable']:
                    # Short evidence gaps (e.g. right after arming) are waited out, not failed.
                    cmd.setdefault('unready_since',now)
                    if now-cmd['unready_since']<3.0:return None
                    raise ValueError('Vertical flight readiness unavailable')
                cmd.pop('unready_since',None)
                if op=='ManagedTakeoff' and not cmd['started']:
                    vehicle=self.evidence.get('vehicle',self.safety.limits.vehicle_max_age_s)
                    if not vehicle or not vehicle.get('position_valid'):
                        # Home needs one fresh position; a sample a moment too old is waited out.
                        cmd.setdefault('home_wait',now)
                        if now-cmd['home_wait']<3.0:return None
                        raise ValueError('No fresh valid position to record home before take-off')
                    self.set_home(vehicle)
                height=cmd.get('height',2);speed=cmd.get('speed',.5)
                if not finite(height) or not 1<=height<=50 or not finite(speed) or not 0<speed<=1:raise ValueError('Vertical flight envelope exceeded')
                if not cmd['started']:
                    if not self.motion_ready():
                        # The previous motion (e.g. an avoidance return flown by velocity) is still
                        # handing back to HOVER: wait for it instead of failing the landing.
                        self.driver.stop();cmd.setdefault('motion_wait',now)
                        if now-cmd['motion_wait']<MOTION_WAIT_S:return None
                        if op=='ManagedLand' and hasattr(self.driver,'abandon'):
                            # Landing is the last safety step (2026-11-01): a take-off cancelled on
                            # the ground did not hand back within MOTION_WAIT_S on a loaded computer,
                            # the landing was refused and the drone stayed ARMED on the ground. Its
                            # cancel is already sent: forget it here and land anyway (AERIXIS core
                            # lands from EMERGENCY; the land behaviour disarms after touchdown).
                            self.driver.abandon()
                        else:
                            raise ValueError('Motion ownership unavailable: previous motion still active')
                    self.journal.mark_flight_authority(self.owner)
                    self.driver.vertical(op,height,speed);cmd['started']=True;cmd['vertical_at']=now
                elif op=='ManagedTakeoff' and self.home:
                    # SITL 2026-10-23: PX4 refused to arm (pre-flight checks) but the AS2 platform
                    # reported ARM and the take-off behaviour ran for 90 s on the ground. Use the
                    # FC's own state: not armed, or no climb, ends the take-off within seconds.
                    waited=now-cmd.get('vertical_at',now)
                    if ground.get('armed') is False and waited>=ARM_WAIT_S:
                        raise ValueError('FC did not arm within %.0f s (PX4 pre-flight checks failed or arming refused): '
                                         'check the FC health before starting again'%ARM_WAIT_S)
                    position=ground.get('position')
                    if position and waited>=CLIMB_WAIT_S and position[2]-self.home['position'][2]<CLIMB_MIN_M:
                        raise ValueError('No climb %.0f s after take-off (%.1f m): take-off stopped'
                                         %(CLIMB_WAIT_S,position[2]-self.home['position'][2]))
                result=self.driver.motion_result()
                if result=='FAILURE':raise ValueError('AERIXIS core vertical action failed')
                if result=='SUCCESS':
                    v=self.evidence.get('vehicle',self.safety.limits.vehicle_max_age_s) or {}
                    if op=='ManagedLand' and v.get('platform_state') not in (0,1):
                        # The behaviour can succeed a moment before the platform reports LANDED.
                        cmd.setdefault('landed_wait',now)
                        if now-cmd['landed_wait']<5.0:return None
                        raise ValueError('Landing result lacks landed-state evidence')
                    return self.finish('SUCCESS')
            elif op in ('SurveyWaypoint','ApproachDeliveryZone','ResumeCoverage'):
                if not flags['VehicleHealthy']:raise ValueError('Vehicle readiness lost')
                swarm=self.evidence.get('swarm')
                if cmd.get('swarm_required') or (swarm and swarm.get('required')):
                    if not flags['PeersFresh'] or not flags['SeparationSafe']:raise ValueError('Peer freshness/separation unavailable')
                if op=='ApproachDeliveryZone':
                    if not all(flags[k] for k in ('GimbalStable','TrackConfirmed','GeolocationValid','AssignmentOwned','DeliveryZoneClear')):raise ValueError('Approach gate lost; stop and reacquire')
                    zone=self.evidence.get('zone');pose=zone.get('approach_position');frame=zone.get('frame_id')
                    if zone.get('target_id')!=self.target or not zone.get('path_clear'):raise ValueError('Delivery target/path not validated')
                    self.driver.track(self.evidence.get('track')['track_id'])
                else:
                    frame=self.evidence.get('vehicle',self.safety.limits.vehicle_max_age_s)['frame_id']
                    if op=='ResumeCoverage':
                        saved=self.journal.state['missions'].get(self.owner)
                        if not saved:raise ValueError('No persistent coverage checkpoint')
                        if 'resume_pose' not in cmd:
                            # Chosen once: the goal must stay the same for the whole leg (route check).
                            here=(self.evidence.get('vehicle',self.safety.limits.vehicle_max_age_s) or {}).get('position')
                            cmd['resume_pose']=self.resume_ahead(saved.get('pose'),here)
                        pose=cmd['resume_pose']
                    else:
                        segment=cmd.get('segment_id')
                        if not segment:raise ValueError('Route segment ID missing')
                        self.last_segment=segment
                        if segment in self.journal.state['missions'].get(self.owner,{}).get('completed',[]):return self.finish('SUCCESS','Segment already completed')
                        pose=[float(x) for x in cmd.get('pose','').split(';')]
                vehicle=self.evidence.get('vehicle',self.safety.limits.vehicle_max_age_s)
                if frame!=vehicle['frame_id'] or not isinstance(pose,list) or len(pose)!=3 or not all(finite(x) for x in pose):raise ValueError('Invalid target pose/frame')
                if op=='SurveyWaypoint' and station_segment(cmd.get('segment_id')):
                    # Mission 2: never fly into a peer's orbit around a claimed person on the way to
                    # the watch station. Stop where the aircraft is (still clear of it); ScanArea
                    # then moves the watch. The segment is not recorded as completed.
                    zone=self.vision.station_leg_blocker(vehicle['position'],pose)
                    if zone:
                        self.driver.stop();self.leg=None
                        return self.finish('SUCCESS',self.vision.station_blocked(zone,vehicle['position'],pose))
                speed=cmd.get('speed',.5)
                if not finite(speed) or not 0<speed<=2 or not 1<=pose[2]<=50:raise ValueError('Motion envelope exceeded')
                if not cmd['started'] and not self.motion_ready():
                    # A cancelled GoTo or a pending hover switch is still winding down: wait for
                    # it instead of failing the leg, but not for ever.
                    self.driver.stop();self.waiting='previous motion still handing back'
                    cmd.setdefault('motion_wait',now)
                    if now-cmd['motion_wait']>MOTION_STUCK_S:raise ValueError('Previous motion did not hand back within %.0f s'%MOTION_STUCK_S)
                    return None
                self.driver.request_route(cmd,pose,frame)
                route=self.evidence.get('route')
                approved=bool(route and route.get('approved') is True and route.get('path_clear') is True
                    and route.get('mission_id')==self.owner and route.get('command_id')==cmd['command_id']
                    and route.get('target_pose')==pose and route.get('frame_id')==frame)
                if not approved:
                    reason=str((route or {}).get('reason') or 'waiting for route approval')
                    self.waiting='route: '+reason
                    if not cmd['started'] and route and route.get('command_id')==cmd['command_id'] and route.get('approved') is False:
                        # SITL 2026-10-23: a refused leg hovered silently until the operator pulled
                        # the drone home. A refusal that persists ends the leg with its reason.
                        cmd.setdefault('refused_since',now)
                        if now-cmd['refused_since']>ROUTE_REFUSED_S:raise ValueError('Route refused: '+reason)
                    if cmd['started']:
                        # Tolerate a stale/late route message; fail only if approval stays lost.
                        cmd.setdefault('unapproved_since',now)
                        if now-cmd['unapproved_since']>1.0:
                            raise ValueError('Route approval lost during motion: '+str((route or {}).get('reason','not approved')))
                        return None
                    return None  # Wait for the request's reply; the operation deadline stays bounded.
                if not cmd['started']:
                    if not self.motion_ready():return None
                    self.journal.mark_flight_authority(self.owner)
                    if self.avoidance is not None and op!='ApproachDeliveryZone':
                        self.avoidance.start_leg(cmd)    # AI avoidance on: this leg is flown by it
                    cmd['leg_start']=list(self.last_goal if op=='SurveyWaypoint' and self.last_goal else vehicle.get('position'))
                    if not cmd.get('avoid'):self.driver.move(pose,frame,speed)
                    cmd['started']=True
                    if op=='SurveyWaypoint':
                        # Planned lane endpoints were validated against the area; measured
                        # positions at lane corners can sit centimetres outside it.
                        self.leg=(list(self.last_goal or vehicle.get('position')),list(pose))
                        # No earlier survey goal: this leg is the transit from the launch point to the
                        # first lane (often from outside the area). Nothing is surveyed on it.
                        self.leg_is_entry=self.last_goal is None
                    if op!='ResumeCoverage':
                        # ResumeCoverage's goal IS the saved resume point: re-saving it from the current
                        # position moved the goal mid-leg ("Approved leg goal changed", PX4 1.16 SITL).
                        self.journal.checkpoint(self.owner,self.last_segment,self.resume_point(vehicle.get('position')))
                cmd.pop('unapproved_since',None);cmd.pop('refused_since',None);self.waiting=''
                if cmd.get('avoid'):
                    if not self.avoidance.enabled:
                        # Switched off mid-leg: the rest of the leg is the usual GoTo.
                        cmd['avoid']=False;self.driver.move(pose,frame,speed);return None
                    gps=self.evidence.get('gps',2.0) or {}
                    velocity,heading,reached=self.avoidance.leg_command(cmd,cmd['leg_start'],pose,speed,vehicle,
                        self.evidence.get('swarm',1.5),float(gps.get('horizontal_sigma_m',2.5)),now,area=(route or {}).get('area'))
                    if not reached:
                        self.driver.velocity(velocity,limit_yaw(vehicle['yaw'],heading),frame);return None
                    self.driver.stop();result='SUCCESS'
                else:
                    result=self.driver.motion_result()
                if result=='FAILURE':raise ValueError('AERIXIS core motion action failed')
                if result=='SUCCESS':
                    if op=='SurveyWaypoint':
                        self.journal.checkpoint(self.owner,self.last_segment,pose,True);self.last_goal=list(pose);self.leg=None
                    return self.finish('SUCCESS')
            elif op=='VerifyDeliveryZone':
                zone=self.evidence.get('zone')
                if not flags['DeliveryZoneClear'] or not zone or zone.get('target_id')!=self.target:raise ValueError('Delivery zone not independently cleared')
                return self.finish('SUCCESS')
            elif op=='ReleasePayload':
                if not cmd['started']:
                    if not flags['SafeToRelease'] or not flags['AssignmentOwned']:raise ValueError('Release interlocks not satisfied')
                    zone=self.evidence.get('zone');vehicle=self.evidence.get('vehicle',self.safety.limits.vehicle_max_age_s);payload=self.evidence.get('payload')
                    if zone['target_id']!=self.target or math.dist(vehicle['position'],zone['approach_position'])>.5:raise ValueError('Not at the approved delivery position')
                    transaction=self.owner+':'+self.target+':'+str(payload.get('payload_id',''))
                    self.journal.mark_flight_authority(self.owner)
                    fresh,prior=self.journal.reserve_release(self.owner,self.target,str(payload.get('payload_id','')),transaction)
                    self.last_release=transaction;cmd['started']=True
                    if not fresh:
                        return self.finish('SUCCESS' if prior['state']=='CONFIRMED' else 'FAILURE','Existing payload transaction; never re-actuated')
                    self.driver.release(transaction,payload['payload_id'])
                payload=self.evidence.get('payload')
                if (payload and self.safety.limits.allow_operator_load and payload.get('operator_only') is True
                        and payload.get('transaction_id')==self.last_release and payload.get('command_sent') is True):
                    self.journal.release_result(self.last_release,False)
                    return self.finish('SUCCESS','COMMAND_SENT_UNVERIFIED: physical delivery is unknown')
                if payload and payload.get('transaction_id')==self.last_release and payload.get('verified') is True and payload.get('present') is False and payload.get('released') is True:
                    self.journal.release_result(self.last_release,True);return self.finish('SUCCESS','Physical release confirmed')
            elif op=='VerifyPayloadRelease':
                if flags['PayloadReleased']:return self.finish('SUCCESS')
                raise ValueError('Payload outcome unconfirmed; inhibit repeat release')
            elif op=='ReportPayloadOutcome':
                payload=self.evidence.get('payload')
                if not payload or payload.get('transaction_id')!=self.last_release or not payload.get('command_sent'):
                    raise ValueError('No matching payload dispatch report')
                self.driver.report(dict(kind='payload_outcome',mission_id=self.owner,transaction_id=self.last_release,
                    truth=payload.get('truth'),physically_confirmed=flags['PayloadReleased']))
                return self.finish('SUCCESS','Outcome reported; no physical-delivery claim')
            elif op=='AcquireAssignment':
                if not self.target or not self.swarm_id:raise ValueError('A shared target ID and swarm mission ID are required')
                self.driver.assignment(self.swarm_id,self.target,'claim')
                if flags['AssignmentOwned']:return self.finish('SUCCESS')
            elif op=='ReleaseAssignment':
                if not flags['HoldConfirmed']:raise ValueError('Verify hold before releasing responder ownership')
                self.driver.assignment(self.swarm_id,self.target,'finish');return self.finish('SUCCESS','Completion reported; authority confirmation remains in swarm status')
            elif op=='CompleteRescue':
                if not self.driver.idle:raise ValueError('Rejoin has not completed')
                if self.incident_track is not None:self.handled.add(self.incident_track)
                if self.incident_label is not None:self.handled_labels.add(self.incident_label);self.incident_label=None
                self.vision.release_claim()
                self.driver.track(None);return self.finish('SUCCESS')
            elif op=='ReportObservation':
                self.driver.track(None);self.driver.report(dict(kind='observation',mission_id=self.owner,target_id=self.target,flags=flags))
                return self.finish('SUCCESS')
            else:raise ValueError('Unsupported rescue operation: '+op)
        except (ValueError,KeyError,TypeError,IndexError) as exc:
            self.driver.stop();self.driver.track(None)
            if op=='ReleasePayload' and self.last_release:
                self.journal.release_result(self.last_release);self.aborted=True
            return self.finish('FAILURE',str(exc))

    def reconcile_ground(self):
        vehicle = self.evidence.get('vehicle',self.safety.limits.vehicle_max_age_s)
        if (self.active or not self.driver.idle or not vehicle
                or vehicle.get('armed') is not False
                or vehicle.get('platform_state') not in (0, 1)
                or vehicle.get('connected') is not True
                or vehicle.get('position_valid') is not True
                or not finite(vehicle.get('speed')) or vehicle['speed'] > .05):
            raise ValueError('Fresh connected, disarmed, stationary ground evidence and idle runtime required')
        self.journal.reconcile_runtime()
        self.recovery_required=False;self.aborted=False;self.safety.abort_reason=''
        self.owner=None;self.target=None;self.swarm_id='';self.last_segment=''
        self.last_release=None;self.incident_track=None;self.handled.clear();self.results.clear()
        self.handled_labels.clear();self.incident_label=None;self.home=None;self.leg=None;self.last_goal=None;self.vision.search_area=None
        self.vision.handled_points=[];self.vision.served_points=[];self.vision.delivered=None;self.vision.watch=None;self.mode_change=None;self.external=False;self.offboard_seen=False
        self.vision.release_claim();self.vision.yielded=None

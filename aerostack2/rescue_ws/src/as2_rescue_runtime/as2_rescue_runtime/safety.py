"""Deterministic safety decisions. Missing or invalid evidence never grants authority."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Limits:
    max_age_s: float = 0.5
    # AS2's platform pauses all of its topics for ~0.5 s while PX4 handles an arm request;
    # vehicle evidence therefore tolerates 1 s before readiness is withdrawn.
    vehicle_max_age_s: float = 1.0
    # On the ground (last state disarmed/landed) the arm pause can exceed 1 s on a loaded
    # computer; a gap there must not fail the flight before take-off. Airborne stays 1 s.
    ground_vehicle_max_age_s: float = 3.0
    hold_speed_mps: float = 0.2
    # Sustained (unstable_persist_s) tilt; accelerating to cruise speed legitimately tilts 15-20 deg.
    max_tilt_deg: float = 30.0
    # Roll/pitch rate only: yaw turns are commanded manoeuvres, not instability.
    max_body_rate_dps: float = 45.0
    max_gimbal_rate_dps: float = 8.0
    # Std-dev of raw accelerometer samples; includes thrust changes, so it is a coarse
    # guard against a failing airframe, not a fine vibration metric.
    max_vibration_rms: float = 6.0
    # Instability must persist this long before it latches an abort.
    unstable_persist_s: float = 0.5
    max_fix_sigma_m: float = 1.5
    battery_reserve: float = 0.25
    stable_s: float = 2.0
    min_person_clearance_m: float = 5.0
    # Overhead drop (release directly above the recipient). This mode trades
    # recipient clearance for release accuracy and can injure the person below
    # if the package is heavy, the release is late, or the shadow/wind offset is
    # larger than expected. It is therefore opt-in per release, and the gates
    # below are enforced in addition to every normal stability gate, not instead
    # of them. Confirm the package mass and altitude envelope on the bench with
    # inert payloads before relying on it.
    min_overhead_agl_m: float = 8.0
    max_overhead_horizontal_error_m: float = 0.75
    max_overhead_agl_m: float = 20.0
    max_overhead_tilt_deg: float = 5.0
    overhead_envelope_verified: bool = False
    allow_operator_load: bool = False


def finite(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


class Evidence:
    def __init__(self,clock):self.clock=clock;self.samples={}
    def put(self,key,data,stamp=None):
        now=self.clock();stamp=now if stamp is None else stamp
        if not finite(stamp) or stamp>now+.05:return False
        old=self.samples.get(key)
        if old and stamp<=old[1]:return False
        self.samples[key]=(data,stamp,now);return True
    def get(self,key,max_age=.5):
        item=self.samples.get(key);now=self.clock()
        if not item or not 0<=now-item[1]<=max_age or not 0<=now-item[2]<=max_age:return None
        return item[0]


class Safety:
    def __init__(self,evidence,limits=Limits()):
        self.evidence=evidence;self.limits=limits;self.stable_since=None;self.abort_reason='';self.unstable_since=None
    def evaluate(self):
        e,l=self.evidence,self.limits
        v=e.get('vehicle',l.vehicle_max_age_s);g=e.get('gimbal',l.max_age_s);t=e.get('track',l.max_age_s)
        if v is None:
            last=e.get('vehicle',l.ground_vehicle_max_age_s)
            if last and last.get('platform_state') in (0,1):v=last
        fix=e.get('fix',l.max_age_s);zone=e.get('zone',l.max_age_s);payload=e.get('payload',l.max_age_s)
        flight=bool(v and v.get('position_valid') is True and v.get('connected') is True and
            all(finite(v.get(k)) for k in ('speed','tilt_deg','body_rate_dps','vibration_rms','battery')))
        rate=v.get('tilt_rate_dps',v.get('body_rate_dps')) if v else None
        # Touchdown contact is not airframe vibration: while AS2 lands or the aircraft is down
        # (LANDING 4 / LANDED 1 / DISARMED 0) vibration does not count; tilt and rates still do.
        # Aborting a landing for it made the aircraft hover instead of land (SITL, 2026-10-25), and
        # landing is what an unstable aircraft should do anyway.
        ground_contact=bool(v) and v.get('platform_state') in (0,1,4)
        vibration_ok=ground_contact or bool(flight and v['vibration_rms']<=l.max_vibration_rms)
        stable=bool(flight and abs(v['tilt_deg'])<=l.max_tilt_deg and finite(rate) and abs(rate)<=l.max_body_rate_dps and vibration_ok)
        if flight and not stable:
            # Unstable evidence withdraws readiness at once; it latches an abort only if it persists.
            if self.unstable_since is None:self.unstable_since=e.clock()
            if e.clock()-self.unstable_since>=l.unstable_persist_s:self.abort_reason='AIRCRAFT_UNSTABLE'
        else:self.unstable_since=None
        if flight and v['battery']<l.battery_reserve:self.abort_reason='BATTERY_RESERVE'
        if v and v.get('position_valid') is False:self.abort_reason='POSITION_INVALID'
        gimbal=bool(g and g.get('verified') is True and all(finite(g.get(k)) for k in ('yaw','pitch','yaw_rate','pitch_rate')) and
                    max(abs(g['yaw_rate']),abs(g['pitch_rate']))<=l.max_gimbal_rate_dps and not g.get('oscillating'))
        track=bool(t and t.get('confirmed') is True and t.get('predicted') is False and
                   finite(t.get('confidence')) and t['confidence']>=.5 and t.get('track_id') is not None)
        localized=bool(fix and fix.get('valid') is True and finite(fix.get('sigma_m')) and
                       0<=fix['sigma_m']<=l.max_fix_sigma_m and track and fix.get('track_id')==t.get('track_id') and
                       fix.get('frame_id')==v.get('frame_id') if v else False)
        held=bool(stable and v.get('hold_ack') is True and v['speed']<=l.hold_speed_mps and v.get('motion_idle') is True)
        visual_required=not (zone and zone.get('kind')=='planned_waypoint' and zone.get('requires_visual_tracking') is False)
        if held and (gimbal or not visual_required):
            if self.stable_since is None:self.stable_since=e.clock()
        else:self.stable_since=None
        settled=self.stable_since is not None and e.clock()-self.stable_since>=l.stable_s

        # Two release geometries exist and they are not interchangeable.
        #  'beside'   - release at an approved clear point near the recipient,
        #               which is what the person-clearance limit protects.
        #  'overhead' - release directly above the recipient, as the operator
        #               requested. There is no recipient clearance in this mode
        #               by definition, so it is refused unless the separate
        #               overhead gates below hold. An unknown mode fails closed.
        mode=(zone or {}).get('drop_mode','beside')
        if mode not in ('beside','overhead'):
            self.abort_reason='INVALID_DROP_MODE'
            mode='beside'
        base_zone=bool(zone and zone.get('clear') is True and zone.get('approved') is True and
                       zone.get('frame_id')==(v or {}).get('frame_id') and zone.get('target_id'))
        overhead_ok=False
        if mode=='overhead':
            # The release is only over the person if the measured geometry says
            # so: the camera is looking straight down, the target is centred,
            # the aircraft is high enough above the surface, and the estimated
            # recipient position is directly beneath the aircraft.
            agl=None
            if localized and isinstance(fix.get('position'),list) and len(fix['position'])==3 and all(finite(x) for x in fix['position']):
                agl=v['position'][2]-fix['position'][2]
            nadir=bool(g and finite(g.get('pitch')) and
                       abs(g['pitch']+90.0)<=l.max_overhead_tilt_deg)
            error=(g or {}).get('center_error')
            centered=bool(gimbal and track and g.get('tracking') is True and str(g.get('track_id'))==str(t['track_id'])
                and isinstance(error,list) and len(error)==2 and all(finite(x) and abs(x)<.04 for x in error))
            height_ok=bool(finite(agl) and l.min_overhead_agl_m<=agl<=l.max_overhead_agl_m)
            offset=None
            if localized and isinstance(fix.get('position'),list) and len(fix['position'])==3:
                fx,fy=fix['position'][0],fix['position'][1]
                if finite(fx) and finite(fy):
                    offset=math.hypot(fx-v['position'][0],fy-v['position'][1])
            aligned=bool(offset is not None and offset<=l.max_overhead_horizontal_error_m)
            overhead_ok=bool(l.overhead_envelope_verified and zone.get('overhead_authorized') is True and base_zone and nadir and centered and height_ok and aligned and localized)
            zone_ok=overhead_ok
            # Misalignment inhibits release. It must not latch an aircraft abort
            # while a healthy aircraft is still approaching the target.
        else:
            zone_ok=bool(base_zone and
                         finite(zone.get('person_clearance_m')) and
                         zone['person_clearance_m']>=l.min_person_clearance_m)
        loaded=bool(payload and payload.get('present') is True and payload.get('latched') is True and payload.get('verified') is True)
        operator_loaded=bool(l.allow_operator_load and payload and payload.get('operator_confirmed') is True and payload.get('consumed') is False)
        # Brief transients (mode switches, gusts) do not withdraw readiness; persistent
        # instability latches AIRCRAFT_UNSTABLE above and withdraws it for good.
        settling=bool(flight and not stable and self.unstable_since is not None and e.clock()-self.unstable_since<l.unstable_persist_s)
        healthy=flight and (stable or settling) and not self.abort_reason
        if healthy:health=''
        elif self.abort_reason:health='abort latched: '+self.abort_reason
        elif not v:health='no fresh flight-controller data (pose/twist/IMU/platform older than %.1f s)'%l.vehicle_max_age_s
        elif not flight:health='flight-controller data incomplete (connected=%s, position_valid=%s%s)'%(v.get('connected'),v.get('position_valid'),
            ''.join(', %s unknown'%k for k in ('speed','tilt_deg','body_rate_dps','vibration_rms','battery') if not finite(v.get(k))))
        else:health='unstable: tilt %.1f deg (max %.0f), roll/pitch rate %.0f deg/s (max %.0f), vibration %.2f (max %.1f)'%(
            v['tilt_deg'],l.max_tilt_deg,rate if finite(rate) else float('nan'),l.max_body_rate_dps,v['vibration_rms'],l.max_vibration_rms)
        return dict(VehicleHealthy=healthy,CameraFresh=bool(e.get('camera',l.max_age_s)),
            GimbalStable=gimbal,TrackConfirmed=track,GeolocationValid=localized,
            HoldConfirmed=held,DeliveryZoneClear=bool(base_zone and l.overhead_envelope_verified and zone.get('overhead_authorized') is True) if mode=='overhead' else zone_ok,PayloadLoaded=loaded,
            DropMode=mode,OverheadAligned=overhead_ok,CenteredOnTarget=overhead_ok,
            PayloadOperatorConfirmed=operator_loaded,
            SafeToRelease=bool(healthy and settled and zone_ok and (loaded or operator_loaded)),
            AbortRequested=bool(self.abort_reason),reason=self.abort_reason or ('GIMBAL_UNSTABLE' if not gimbal else ''),
            health=health)

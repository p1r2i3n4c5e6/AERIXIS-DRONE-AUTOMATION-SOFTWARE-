"""Decision core of the flight-controller guardian (no ROS or framework imports; see fc_guardian.py).

The guardian is the last onboard layer before PX4's own failsafe. It watches whether the AERIXIS core
"brain" is alive and, only in an emergency, commands PX4 directly:

  * AERIXIS core lost   AERIXIS core was seen alive, then platform/info or rescue/state stopped for longer than
               ``as2_timeout_s`` while the aircraft is armed, airborne and still in an AERIXIS core mode
               (OFFBOARD, or PX4 HOLD after an offboard-loss failsafe) -> RTL or LAND.
  * Operator   the GCS asked for RTL/LAND over the telemetry radio (through the MAVLink
               gateway), even while AERIXIS core is alive or dead -> that action.

Rules that keep it safe:
  * never acts unless PX4 reports the aircraft armed and airborne;
  * never acts while a pilot mode (MANUAL/ALTCTL/POSCTL/ACRO/STAB) is active: the pilot wins,
    also during a running emergency (state STOOD_DOWN);
  * never fights a mode PX4 or AERIXIS core already chose for landing/returning (RTL, LAND, DESCEND,
    PRECLAND, TERMINATION): AERIXIS-loss with PX4 already in such a mode is reported as
    PX4_FAILSAFE and nothing is sent;
  * one decision per event: the command is repeated every ``retry_period_s`` only until PX4
    shows the new mode, then never again (no double action). A refused RTL (e.g. no home
    position) falls back to LAND after ``rtl_fallback_after`` attempts;
  * if AERIXIS core was LANDING when it died, LAND is used instead of RTL (an RTL would climb first);
  * landed + disarmed clears everything for the next flight.
PX4 v1.14 does not bridge vehicle_command_ack or vehicle_land_detected over uXRCE-DDS, so
confirmation is the nav_state change and "airborne" falls back to vehicle_status.takeoff_time
plus height above the ground level seen while disarmed. Both topics are used when present.
"""

ACTIONS = ('RTL', 'LAND')
VEHICLE_CMD = {'RTL': 20, 'LAND': 21}           # VEHICLE_CMD_NAV_RETURN_TO_LAUNCH / NAV_LAND
TARGET_NAV = {'RTL': 5, 'LAND': 18}             # NAVIGATION_STATE_AUTO_RTL / AUTO_LAND
NAV_NAMES = {0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'MISSION', 4: 'HOLD', 5: 'RTL', 10: 'ACRO',
             12: 'DESCEND', 13: 'TERMINATION', 14: 'OFFBOARD', 15: 'STAB', 17: 'TAKEOFF', 18: 'LAND',
             19: 'FOLLOW', 20: 'PRECLAND', 21: 'ORBIT', 22: 'VTOL_TAKEOFF'}
PILOT_MODES = frozenset((0, 1, 2, 10, 15))
EMERGENCY_MODES = frozenset((5, 12, 13, 18, 20))
AS2_MODES = frozenset((14, 4))                  # OFFBOARD, HOLD (PX4's offboard-loss hold)
ARMED = 2                                       # VehicleStatus.ARMING_STATE_ARMED
ACK_ACCEPTED, ACK_IN_PROGRESS = 0, 5
AS2_LANDING = 4                                 # as2_msgs/PlatformStatus.LANDING

# States (published in fc_guardian/state and logged on every change)
NO_FC = 'NO_FC'                     # no fresh PX4 vehicle_status: guardian cannot act
ON_GROUND = 'ON_GROUND'             # disarmed or landed: never acts
WAITING_FOR_AS2 = 'WAITING_FOR_AS2' # airborne, AS2 not seen since the guardian started
MONITORING = 'MONITORING'           # airborne, AS2 alive
PILOT_CONTROL = 'PILOT_CONTROL'     # a pilot mode is active: AS2 loss is not acted on
OTHER_MODE = 'OTHER_MODE'           # AS2 lost in a PX4 mode AS2 does not own (MISSION, ...)
PX4_FAILSAFE = 'PX4_FAILSAFE'       # AS2 lost, PX4 is already returning/landing by itself
COMMANDING = 'COMMANDING'           # emergency decided, command sent, waiting for PX4's mode
EMERGENCY_ACTIVE = 'EMERGENCY_ACTIVE'  # PX4 confirmed the emergency mode
STOOD_DOWN = 'STOOD_DOWN'           # pilot/GCS/AS2 changed the mode after/while we acted


class Config:
    def __init__(self, action='RTL', as2_timeout_s=3.0, px4_timeout_s=1.5, retry_period_s=1.0,
                 rtl_fallback_after=5, min_airborne_height_m=1.0, require_rescue_state=True):
        self.action = normalise_action(action)
        self.as2_timeout_s = float(as2_timeout_s)
        self.px4_timeout_s = float(px4_timeout_s)
        self.retry_period_s = float(retry_period_s)
        self.rtl_fallback_after = int(rtl_fallback_after)
        self.min_airborne_height_m = float(min_airborne_height_m)
        self.require_rescue_state = bool(require_rescue_state)


def normalise_action(value):
    text = str(value).strip().upper()
    text = {'RETURN': 'RTL', 'RETURN_TO_LAUNCH': 'RTL', 'LAND_NOW': 'LAND'}.get(text, text)
    if text not in ACTIONS:
        raise ValueError('emergency action must be RTL or LAND, not {!r}'.format(value))
    return text


class _Source:
    """Liveness of one AERIXIS core signal (receipt time on the guardian's monotonic clock)."""
    def __init__(self):
        self.at = None
        self.seen = False

    def mark(self, now):
        self.at, self.seen = now, True

    def age(self, now):
        return None if self.at is None else now - self.at


class GuardianCore:
    def __init__(self, config=None):
        self.config = config or Config()
        self.platform = _Source()
        self.rescue = _Source()
        self.as2_state = None           # last as2_msgs/PlatformStatus.state
        self.status = None              # dict(armed, nav_state, takeoff_time) from vehicle_status
        self.status_at = None
        self.landed = None              # vehicle_land_detected.landed (only if PX4 bridges it)
        self.landed_at = None
        self.height = None              # metres above the ground level recorded while disarmed
        self.height_at = None
        self.ground_z = None
        self.state = NO_FC
        self.emergency = None           # dict while an emergency is latched
        self.events = []                # (time, text) for the state topic, newest last
        self.outbox = []                # (time, text, level) for the node to log
        self.last_detail = ''

    # ------------------------------------------------------------------ inputs
    def on_platform(self, now, state=None):
        self.platform.mark(now)
        if state is not None:
            self.as2_state = int(state)

    def on_rescue(self, now):
        self.rescue.mark(now)

    def on_status(self, now, armed, nav_state, takeoff_time=0):
        self.status = dict(armed=bool(armed), nav_state=int(nav_state), takeoff_time=int(takeoff_time))
        self.status_at = now

    def on_land_detected(self, now, landed):
        self.landed, self.landed_at = bool(landed), now

    def on_local_position(self, now, z, z_valid=True):
        """NED z of PX4's local position; the ground level is the last value while disarmed."""
        if not z_valid or z != z:   # NaN
            return
        if self.status is not None and not self.status['armed']:
            self.ground_z = z
        reference = self.ground_z if self.ground_z is not None else 0.0
        self.height, self.height_at = reference - z, now

    def on_ack(self, now, command, result):
        e = self.emergency
        if e is None or e['state'] != COMMANDING or int(command) != VEHICLE_CMD[e['action']]:
            return
        e['ack'] = int(result)
        if int(result) not in (ACK_ACCEPTED, ACK_IN_PROGRESS):
            e['refused'] += 1
            self._event(now, 'PX4 refused {} (result {})'.format(e['action'], int(result)), 'warn')

    def set_action(self, action):
        action = normalise_action(action)
        if action != self.config.action:
            self.config.action = action
            return True
        return False

    # ---------------------------------------------------------------- queries
    def fc_fresh(self, now):
        return self.status_at is not None and now - self.status_at <= self.config.px4_timeout_s

    def armed(self, now):
        return self.fc_fresh(now) and self.status['armed']

    def airborne(self, now):
        if not self.armed(now):
            return False
        if self.landed_at is not None and now - self.landed_at <= self.config.px4_timeout_s:
            return not self.landed
        if not self.status['takeoff_time']:
            return False            # PX4 has not detected a takeoff since arming
        if self.height_at is not None and now - self.height_at <= self.config.px4_timeout_s:
            return self.height >= self.config.min_airborne_height_m
        return True                 # PX4 says it took off; height unknown

    def as2_seen(self):
        return self.platform.seen and (self.rescue.seen or not self.config.require_rescue_state)

    def as2_missing(self, now):
        """Names of AERIXIS core signals that stopped (only signals seen before can be 'stopped')."""
        missing = []
        limit = self.config.as2_timeout_s
        if self.platform.seen and self.platform.age(now) > limit:
            missing.append('platform/info')
        if self.rescue.seen and self.rescue.age(now) > limit:
            missing.append('rescue/state')
        return missing

    # ------------------------------------------------------------------ requests
    def request(self, now, action, source='gcs'):
        """Operator emergency (GCS over the radio). Returns (accepted, reason)."""
        try:
            action = normalise_action(action)
        except ValueError as exc:
            return False, str(exc)
        if not self.fc_fresh(now):
            return False, 'no PX4 link (vehicle_status stale)'
        if not self.airborne(now):
            return False, 'aircraft is not airborne; nothing to do on the ground'
        e = self.emergency
        if e is not None and e['action'] == action and e['state'] in (COMMANDING, EMERGENCY_ACTIVE):
            return True, '{} already {}'.format(action, 'active' if e['state'] == EMERGENCY_ACTIVE else 'commanded')
        if self.status['nav_state'] == TARGET_NAV[action] and e is None:
            self._latch(now, action, source, 'operator request; PX4 already in ' + action)
            self.emergency['state'] = EMERGENCY_ACTIVE
            self.state = EMERGENCY_ACTIVE
            return True, 'PX4 already in ' + action
        self._latch(now, action, source, 'operator request from ' + source)
        return True, '{} commanded'.format(action)

    # ------------------------------------------------------------------ tick
    def step(self, now):
        """Advance the state machine. Returns a list of commands: dict(action, command, attempt)."""
        commands = []
        if not self.fc_fresh(now):
            self._set(now, NO_FC, 'no PX4 vehicle_status for {:.1f} s'.format(
                (now - self.status_at) if self.status_at is not None else float('inf')))
            return commands
        nav = self.status['nav_state']
        if not self.armed(now) or not self.airborne(now):
            if self.emergency is not None and not self.armed(now):
                self._event(now, 'Disarmed on the ground: emergency {} finished'.format(self.emergency['action']), 'info')
                self.emergency = None
            if self.emergency is None:
                self._set(now, ON_GROUND, 'armed, not airborne' if self.armed(now) else 'disarmed')
                return commands
            # Armed and touched down during an emergency: keep the latch until disarm, send nothing.
            self.emergency['state'] = EMERGENCY_ACTIVE if nav in EMERGENCY_MODES else self.emergency['state']
            self._set(now, self.emergency['state'], 'touched down, waiting for disarm')
            return commands

        e = self.emergency
        if e is not None:
            return self._run_emergency(now, e, nav)

        missing = self.as2_missing(now)
        if not self.as2_seen() and not missing:
            self._set(now, WAITING_FOR_AS2, 'airborne; AERIXIS core has not been seen by the guardian yet')
            return commands
        if not missing:
            self._set(now, MONITORING, 'AERIXIS core alive')
            return commands
        lost = 'AERIXIS core lost ({} silent > {:.1f} s)'.format(', '.join(missing), self.config.as2_timeout_s)
        if nav in PILOT_MODES:
            self._set(now, PILOT_CONTROL, lost + '; pilot has control in ' + NAV_NAMES.get(nav, str(nav)))
        elif nav in EMERGENCY_MODES:
            self._set(now, PX4_FAILSAFE, lost + '; PX4 already in ' + NAV_NAMES.get(nav, str(nav)))
        elif nav not in AS2_MODES:
            self._set(now, OTHER_MODE, lost + '; PX4 mode {} is not AERIXIS-controlled'.format(NAV_NAMES.get(nav, nav)))
        else:
            action = self.config.action
            reason = lost
            if action == 'RTL' and self.as2_state == AS2_LANDING:
                action, reason = 'LAND', lost + '; AERIXIS core was landing, finishing with LAND instead of RTL'
            self._latch(now, action, 'as2_loss', reason)
            return self._run_emergency(now, self.emergency, nav)
        return commands

    def _run_emergency(self, now, e, nav):
        commands = []
        if e['state'] == STOOD_DOWN:
            # Stay out of the way while AS2 is still silent (acting again would fight whoever
            # changed the mode); once AS2 is alive again, a new loss is a new emergency.
            if not self.as2_missing(now) and self.as2_seen():
                self._event(now, 'AERIXIS core alive after stand-down; monitoring again', 'info')
                self.emergency = None
                self._set(now, MONITORING, 'AERIXIS core alive')
            else:
                self._set(now, STOOD_DOWN, e['note'])
            return commands
        target = TARGET_NAV[e['action']]
        e['nav_changed'] = e['nav_changed'] or nav != e['nav_at_latch']
        # A pilot mode entered after the decision means the pilot took over. (An operator request
        # made while a pilot mode was already active is honoured until the mode changes.)
        if nav in PILOT_MODES and e['nav_changed']:
            e['state'], e['note'] = STOOD_DOWN, 'pilot took control in {} during {}'.format(NAV_NAMES.get(nav, nav), e['action'])
            self._event(now, e['note'] + '; guardian stands down', 'warn')
            self._set(now, STOOD_DOWN, e['note'])
            return commands
        if e['state'] == COMMANDING:
            if nav == target or (nav in EMERGENCY_MODES and nav != e['nav_at_latch']):
                e['state'] = EMERGENCY_ACTIVE
                e['confirmed_at'] = now
                self._event(now, 'PX4 confirmed {} (nav {}) after {} command(s)'.format(
                    e['action'], NAV_NAMES.get(nav, nav), e['attempts']), 'warn')
                self._set(now, EMERGENCY_ACTIVE, '{} active ({})'.format(e['action'], e['reason']))
                return commands
            if e['last_sent'] is None or now - e['last_sent'] >= self.config.retry_period_s - 1e-6:
                if e['action'] == 'RTL' and e['attempts'] >= self.config.rtl_fallback_after:
                    self._event(now, 'RTL not taken by PX4 after {} attempts (refused {}): falling back to LAND'.format(
                        e['attempts'], e['refused']), 'error')
                    e.update(action='LAND', attempts=0, refused=0, ack=None,
                             reason=e['reason'] + '; RTL refused -> LAND')
                e['attempts'] += 1
                e['last_sent'] = now
                commands.append(dict(action=e['action'], command=VEHICLE_CMD[e['action']], attempt=e['attempts']))
                if e['attempts'] == 1 or e['attempts'] % 5 == 0:
                    self._event(now, 'Commanding PX4 {} (attempt {}): {}'.format(e['action'], e['attempts'], e['reason']),
                                'error' if e['attempts'] > 1 else 'warn')
            self._set(now, COMMANDING, 'sent {} x{}, PX4 in {}'.format(e['action'], e['attempts'], NAV_NAMES.get(nav, nav)))
            return commands
        # EMERGENCY_ACTIVE: watch only; never re-command.
        if nav not in EMERGENCY_MODES:
            e['state'], e['note'] = STOOD_DOWN, 'mode changed to {} after {}; not overriding'.format(NAV_NAMES.get(nav, nav), e['action'])
            self._event(now, e['note'], 'warn')
            self._set(now, STOOD_DOWN, e['note'])
            return commands
        self._set(now, EMERGENCY_ACTIVE, '{} active, PX4 in {}'.format(e['action'], NAV_NAMES.get(nav, nav)))
        return commands

    # ---------------------------------------------------------------- helpers
    def _latch(self, now, action, source, reason):
        self.emergency = dict(action=action, source=source, reason=reason, state=COMMANDING, since=now,
                              attempts=0, refused=0, ack=None, last_sent=None, confirmed_at=None, note='', nav_changed=False,
                              nav_at_latch=self.status['nav_state'] if self.status else None)
        self._event(now, 'EMERGENCY {} ({}): {}'.format(action, source, reason), 'error')

    def _set(self, now, state, detail):
        if state != self.state:
            self._event(now, 'state {} -> {}: {}'.format(self.state, state, detail),
                        'warn' if state in (PILOT_CONTROL, OTHER_MODE, PX4_FAILSAFE, STOOD_DOWN, NO_FC) else 'info')
        self.state, self.last_detail = state, detail

    def _event(self, now, text, level='info'):
        self.events.append((now, text))
        del self.events[:-10]
        self.outbox.append((now, text, level))

    def take_logs(self):
        logs, self.outbox = self.outbox, []
        return logs

    def snapshot(self, now):
        e = self.emergency
        status = self.status or {}
        return dict(
            state=self.state, detail=self.last_detail, action=self.config.action,
            as2=dict(alive=self.as2_seen() and not self.as2_missing(now), seen=self.as2_seen(),
                     platform_age_s=_round(self.platform.age(now)), rescue_age_s=_round(self.rescue.age(now)),
                     timeout_s=self.config.as2_timeout_s),
            px4=dict(fresh=self.fc_fresh(now), armed=self.armed(now), airborne=self.airborne(now),
                     nav=NAV_NAMES.get(status.get('nav_state'), status.get('nav_state')),
                     height_m=_round(self.height)),
            emergency=None if e is None else dict(
                action=e['action'], source=e['source'], reason=e['reason'], state=e['state'],
                attempts=e['attempts'], refused=e['refused'], ack=e['ack'], age_s=_round(now - e['since']),
                note=e['note']),
            events=[dict(age_s=_round(now - t), text=text) for t, text in self.events[-5:]])


def _round(value):
    return None if value is None else round(value, 2)

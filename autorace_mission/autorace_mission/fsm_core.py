'''
Mission state machine core (no ROS, unit-testable).

Who drives:
  - lane following missions: the lane node keeps driving; the FSM only sets its inputs
    (/lane_state, /lane/follow_line, /intersection_state).
  - own-motion missions (parking, tunnel, ...; not written yet): the FSM stops the lane node
    and drives through /cmd_vel_mission. The mux forwards only the source named by `drive`.

Implemented now: start + intersection.

  LANE ── INTERSECTION sign confirmed ──▶ ARMED ── /left or /right confirmed ──▶ BRANCH
   ▲  (left/right ignored here)            │ arm_timeout_m driven, no direction       │ follow one line, slow
   └───────────────────────────────────────┘                                          │
   └──── branch_hold_m driven since the LAST sighting of the chosen sign ◀────────────┘

Why ARMED: on this map the intersection island is ~1.2 m from the start straight, and a
left/right sign may be seen before the robot is at the fork (other layouts, other props).
Without the gate the first stray /left would trigger the branch and, with intersection_once,
make the robot ignore the real intersection.

Why the hold counts from the last sighting: the direction signs stand on the island at the
fork, so the last sighting is close to the fork whatever the detection range was.
'''
import math

DEFAULTS = {
    'start_mode': 'go',          # 'go' = start lane following immediately (traffic light: later)
    'confirm_count': 1,          # True messages needed within confirm_window (detector already confirms)
    'confirm_window': 1.0,       # s
    'require_arm': True,         # left/right count only after the INTERSECTION sign was confirmed
    'arm_timeout_m': 4.0,        # m driven while armed without a direction sign -> disarm
    'slow_when_armed': False,    # use v_max_intersection already while armed
    'left_line': 'yellow',       # line to follow for the left branch  (checked on the course texture)
    'right_line': 'white',       # line to follow for the right branch (checked on the course texture)
    'branch_hold_m': 1.0,        # m driven after the last sighting of the chosen sign (initial value, tune)
    'branch_timeout_s': 30.0,    # s since the decision, safety exit
    'intersection_once': True,   # ignore intersection/left/right after the intersection is done
}

SIGNS = ('intersection', 'left', 'right')


class Outputs(object):
    '''What the FSM wants right now. The ROS node publishes these.'''

    def __init__(self, lane_on=False, follow_line='both', slow=False, drive='stop'):
        self.lane_on = lane_on            # /lane_state
        self.follow_line = follow_line    # /lane/follow_line: both | yellow | white
        self.slow = slow                  # /intersection_state (lane node uses v_max_intersection)
        self.drive = drive                # /mission/drive: lane | mission | stop

    def as_dict(self):
        return dict(lane_on=self.lane_on, follow_line=self.follow_line, slow=self.slow, drive=self.drive)


class MissionFSM(object):
    def __init__(self, **params):
        self.p = dict(DEFAULTS)
        for k, v in params.items():
            if k not in DEFAULTS:
                raise KeyError('unknown parameter %s' % k)
            self.p[k] = v
        for side in ('left_line', 'right_line'):
            if self.p[side] not in ('yellow', 'white'):
                raise ValueError('%s must be yellow or white' % side)
        self.state = 'WAIT_START'
        self.sign_times = dict((s, []) for s in SIGNS)
        self.intersection_done = False
        self.travel = 0.0                # m driven since start (odometry)
        self.last_xy = None
        self.arm_travel = None           # travel when armed
        self.branch = None               # 'left' | 'right' while in BRANCH
        self.branch_t0 = None
        self.last_seen_travel = None     # travel at the last True of the chosen sign
        self.ignored_log_t = -1e9
        self.events = []                 # (t, text) log for the node to print

    # ---- inputs ----------------------------------------------------------------------

    def on_sign(self, sign, value, t):
        '''sign: intersection | left | right, value: Bool data. Only True counts.'''
        if sign not in SIGNS:
            raise ValueError('unknown sign %r' % sign)
        if not value:
            return
        self.sign_times[sign].append(t)
        if self.state == 'BRANCH' and sign == self.branch:
            self.last_seen_travel = self.travel

    def on_odom(self, x, y):
        if self.last_xy is not None:
            self.travel += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)

    # ---- helpers ---------------------------------------------------------------------

    def _log(self, t, text):
        self.events.append((t, text))

    def _prune(self, t):
        w = self.p['confirm_window']
        for s in SIGNS:
            self.sign_times[s] = [x for x in self.sign_times[s] if t - x <= w]

    def _clear(self, *signs):
        for s in signs or SIGNS:
            self.sign_times[s] = []

    def _has(self, sign):
        return len(self.sign_times[sign]) >= self.p['confirm_count']

    def _direction(self, t):
        '''Confirmed left/right, None if neither or both (conflict).'''
        l, r = self._has('left'), self._has('right')
        if l and r:
            self._log(t, 'left and right both seen within %.1f s -> ignored' % self.p['confirm_window'])
            self._clear('left', 'right')
            return None
        return 'left' if l else ('right' if r else None)

    def _lane(self):
        return Outputs(lane_on=True, follow_line='both', slow=False, drive='lane')

    def _start_branch(self, side, t):
        self.state, self.branch, self.branch_t0 = 'BRANCH', side, t
        self.last_seen_travel = self.travel
        self._clear()
        self._log(t, 'sign %s -> BRANCH, follow %s' % (side, self.p[side + '_line']))

    # ---- main ------------------------------------------------------------------------

    def step(self, t):
        '''Advance the state machine and return Outputs.'''
        self._prune(t)

        if self.state == 'WAIT_START':
            if self.p['start_mode'] != 'go':
                return Outputs(lane_on=False, drive='stop')
            self.state = 'LANE'
            self._clear()
            self._log(t, 'start -> LANE')

        if self.state == 'LANE':
            if self.intersection_done and self.p['intersection_once']:
                self._clear()
                return self._lane()
            if not self.p['require_arm']:
                side = self._direction(t)
                if side is None:
                    return self._lane()
                self._start_branch(side, t)
            elif self._has('intersection'):
                self.state, self.arm_travel = 'ARMED', self.travel
                self._clear()
                self._log(t, 'INTERSECTION sign -> ARMED (waiting for left/right, %.1f m max)'
                          % self.p['arm_timeout_m'])
            else:
                if (self._has('left') or self._has('right')) and t - self.ignored_log_t > 2.0:
                    self.ignored_log_t = t
                    self._log(t, 'left/right seen before the INTERSECTION sign -> ignored')
                self._clear('left', 'right')
                return self._lane()

        if self.state == 'ARMED':
            side = self._direction(t)
            if side is not None:
                self._start_branch(side, t)
            elif self.travel - self.arm_travel >= self.p['arm_timeout_m']:
                self._log(t, 'no left/right within %.1f m of the INTERSECTION sign -> LANE (not done)'
                          % self.p['arm_timeout_m'])
                self.state, self.arm_travel = 'LANE', None
                self._clear()
                return self._lane()
            else:
                return Outputs(lane_on=True, follow_line='both', slow=self.p['slow_when_armed'], drive='lane')

        if self.state == 'BRANCH':
            since_seen = self.travel - self.last_seen_travel
            timed_out = t - self.branch_t0 >= self.p['branch_timeout_s']
            if since_seen >= self.p['branch_hold_m'] or timed_out:
                self._log(t, 'branch %s done (%.2f m after last sighting, %.1f s%s) -> LANE'
                          % (self.branch, since_seen, t - self.branch_t0, ', TIMEOUT' if timed_out else ''))
                self.state, self.branch, self.intersection_done = 'LANE', None, True
                self._clear()
                return self._lane()
            return Outputs(lane_on=True, follow_line=self.p[self.branch + '_line'], slow=True, drive='lane')

        raise RuntimeError('unknown state %s' % self.state)

    def status(self, t):
        s = 'state=%s' % self.state
        if self.state == 'ARMED':
            s += ' armed %.2f/%.2fm' % (self.travel - self.arm_travel, self.p['arm_timeout_m'])
        if self.state == 'BRANCH':
            s += ' branch=%s since_seen=%.2f/%.2fm t=%.1fs' % (
                self.branch, self.travel - self.last_seen_travel, self.p['branch_hold_m'], t - self.branch_t0)
        s += ' intersection_done=%s' % self.intersection_done
        return s

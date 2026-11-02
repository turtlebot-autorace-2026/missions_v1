'''
Mission state machine core (no ROS, unit-testable).

Who drives:
  - lane following missions: the lane node keeps driving; the FSM only sets its inputs
    (/lane_state, /lane/follow_line, /intersection_state).
  - own-motion missions (parking, tunnel, ...; not written yet): the FSM stops the lane node
    and drives through /cmd_vel_mission. The mux forwards only the source named by `drive`.

Implemented now: start + intersection.
  LANE ── /left or /right confirmed ──▶ INTERSECTION (follow one line, slow)
       ◀── branch_hold_m driven (odometry) or branch_timeout_s ──┘
'''
import math

DEFAULTS = {
    'start_mode': 'go',          # 'go' = start lane following immediately (traffic light: later)
    'confirm_count': 1,          # True messages needed within confirm_window (detector already confirms)
    'confirm_window': 1.0,       # s
    'left_line': 'yellow',       # line to follow for the left branch  (verify in Gazebo)
    'right_line': 'white',       # line to follow for the right branch (verify in Gazebo)
    'branch_hold_m': 0.9,        # m driven on one line after the decision (initial value, tune in Gazebo)
    'branch_timeout_s': 20.0,    # s, safety exit of the branch state
    'intersection_once': True,   # ignore /left /right after the intersection is done
}


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
        self.sign_times = {'left': [], 'right': []}
        self.intersection_done = False
        self.branch = None               # 'left' | 'right' while in INTERSECTION
        self.branch_t0 = None
        self.travel = 0.0                # m driven since the branch decision
        self.last_xy = None
        self.events = []                 # (t, text) log for the node to print

    # ---- inputs ----------------------------------------------------------------------

    def on_sign(self, side, value, t):
        '''side: left | right, value: Bool data. Only True counts.'''
        if value:
            self.sign_times[side].append(t)

    def on_odom(self, x, y):
        if self.last_xy is not None and self.state == 'INTERSECTION':
            self.travel += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)

    # ---- main ------------------------------------------------------------------------

    def _log(self, t, text):
        self.events.append((t, text))

    def _confirmed(self, t):
        w = self.p['confirm_window']
        for side in ('left', 'right'):
            self.sign_times[side] = [s for s in self.sign_times[side] if t - s <= w]
        n_l, n_r = len(self.sign_times['left']), len(self.sign_times['right'])
        need = self.p['confirm_count']
        if n_l >= need and n_r >= need:
            self._log(t, 'left and right both seen within %.1f s -> ignored' % w)
            self.sign_times = {'left': [], 'right': []}
            return None
        if n_l >= need:
            return 'left'
        if n_r >= need:
            return 'right'
        return None

    def step(self, t):
        '''Advance the state machine and return Outputs.'''
        if self.state == 'WAIT_START':
            if self.p['start_mode'] == 'go':
                self.state = 'LANE'
                self._log(t, 'start -> LANE')
            else:
                return Outputs(lane_on=False, drive='stop')

        if self.state == 'LANE':
            side = self._confirmed(t)
            if side is not None and not (self.intersection_done and self.p['intersection_once']):
                self.state, self.branch, self.branch_t0, self.travel = 'INTERSECTION', side, t, 0.0
                self.sign_times = {'left': [], 'right': []}
                self._log(t, 'sign %s -> INTERSECTION, follow %s' % (side, self.p[side + '_line']))
            else:
                return Outputs(lane_on=True, follow_line='both', slow=False, drive='lane')

        if self.state == 'INTERSECTION':
            line = self.p[self.branch + '_line']
            timed_out = t - self.branch_t0 >= self.p['branch_timeout_s']
            if self.travel >= self.p['branch_hold_m'] or timed_out:
                self._log(t, 'branch %s done (%.2f m, %.1f s%s) -> LANE'
                          % (self.branch, self.travel, t - self.branch_t0, ', TIMEOUT' if timed_out else ''))
                self.state, self.branch, self.intersection_done = 'LANE', None, True
                self.sign_times = {'left': [], 'right': []}
                return Outputs(lane_on=True, follow_line='both', slow=False, drive='lane')
            return Outputs(lane_on=True, follow_line=line, slow=True, drive='lane')

        raise RuntimeError('unknown state %s' % self.state)

    def status(self, t):
        s = 'state=%s' % self.state
        if self.state == 'INTERSECTION':
            s += ' branch=%s travel=%.2f/%.2fm t=%.1fs' % (self.branch, self.travel, self.p['branch_hold_m'],
                                                         t - self.branch_t0)
        s += ' intersection_done=%s' % self.intersection_done
        return s

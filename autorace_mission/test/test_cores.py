'''Offline checks of the mission FSM and cmd_vel mux cores (no ROS).'''
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from autorace_mission.fsm_core import MissionFSM  # noqa: E402
from autorace_mission.mux_core import ZERO, CmdVelMux  # noqa: E402


class Robot(object):
    '''Drives along x and feeds odometry; each step is 0.1 s.'''

    def __init__(self, fsm):
        self.fsm, self.x, self.t = fsm, 0.0, 0.0
        fsm.on_odom(0.0, 0.0)

    def drive(self, dist, signs=(), steps=10):
        '''Move dist [m]; each step also sends the given signs (True). Returns the last Outputs.'''
        out = None
        for _ in range(steps):
            self.x += dist / steps
            self.t += 0.1
            self.fsm.on_odom(self.x, 0.0)
            for s in signs:
                self.fsm.on_sign(s, True, self.t)
            out = self.fsm.step(self.t)
        return out


def started(**kw):
    fsm = MissionFSM(**kw)
    fsm.step(0.0)
    return fsm, Robot(fsm)


def test_start_go_then_lane():
    fsm = MissionFSM()
    out = fsm.step(0.0)
    assert fsm.state == 'LANE'
    assert out.as_dict() == dict(lane_on=True, follow_line='both', slow=False, drive='lane')


def test_start_wait():
    fsm = MissionFSM(start_mode='traffic_light')
    out = fsm.step(0.0)
    assert fsm.state == 'WAIT_START' and not out.lane_on and out.drive == 'stop'


def test_direction_sign_before_arming_is_ignored():
    '''The user's case: a left/right sign seen near the start must not trigger or use up the intersection.'''
    fsm, r = started()
    out = r.drive(0.5, signs=('left',))
    assert fsm.state == 'LANE' and out.follow_line == 'both' and not fsm.intersection_done
    out = r.drive(0.5, signs=('right',))
    assert fsm.state == 'LANE' and out.follow_line == 'both'
    r.drive(0.3, signs=('intersection',))                # the real intersection still works
    assert fsm.state == 'ARMED'
    out = r.drive(0.2, signs=('left',))
    assert fsm.state == 'BRANCH' and out.follow_line == 'yellow' and out.slow


@pytest.mark.parametrize('side,line', [('left', 'yellow'), ('right', 'white')])
def test_branch_hold_counts_from_last_sighting(side, line):
    fsm, r = started(branch_hold_m=1.0)
    r.drive(0.2, signs=('intersection',))
    out = r.drive(1.5, signs=(side,))                  # sign visible for 1.5 m (detected early)
    assert fsm.state == 'BRANCH' and out.follow_line == line
    out = r.drive(0.9)                                 # 0.9 m after the last sighting: still holding
    assert fsm.state == 'BRANCH' and out.follow_line == line
    out = r.drive(0.2)                                 # 1.1 m after the last sighting: done
    assert fsm.state == 'LANE' and out.follow_line == 'both' and not out.slow
    assert fsm.intersection_done


def test_other_side_after_decision_is_ignored():
    fsm, r = started()
    r.drive(0.1, signs=('intersection',))
    r.drive(0.1, signs=('left',))
    out = r.drive(0.3, signs=('right',))               # misdetection at the fork does not flip the line
    assert fsm.branch == 'left' and out.follow_line == 'yellow'


def test_arm_timeout_disarms_without_using_up():
    fsm, r = started(arm_timeout_m=4.0)
    r.drive(0.1, signs=('intersection',))
    r.drive(3.0)
    assert fsm.state == 'ARMED'
    r.drive(1.2)
    assert fsm.state == 'LANE' and not fsm.intersection_done
    r.drive(0.1, signs=('left',))                      # not armed any more
    assert fsm.state == 'LANE'
    r.drive(0.1, signs=('intersection',))              # can arm again
    assert fsm.state == 'ARMED'


def test_once():
    fsm, r = started(branch_hold_m=0.5)
    r.drive(0.1, signs=('intersection',))
    r.drive(0.1, signs=('right',))
    r.drive(0.6)
    assert fsm.state == 'LANE' and fsm.intersection_done
    r.drive(0.2, signs=('intersection', 'left'))       # e.g. the parking-area left-arrow sign
    assert fsm.state == 'LANE'


def test_branch_timeout():
    fsm, r = started(branch_timeout_s=30.0)
    r.drive(0.1, signs=('intersection',))
    r.drive(0.1, signs=('left',))
    for k in range(31):                                # standing still (no odometry change)
        fsm.on_sign('left', True, r.t + k)
        fsm.step(r.t + k)
    assert fsm.state == 'LANE' and fsm.intersection_done


def test_without_arming():
    fsm, r = started(require_arm=False)
    out = r.drive(0.1, signs=('right',))
    assert fsm.state == 'BRANCH' and out.follow_line == 'white'


def test_false_is_ignored():
    fsm, r = started()
    fsm.on_sign('intersection', False, 0.5)
    fsm.step(0.5)
    assert fsm.state == 'LANE'


def test_conflicting_signs_ignored():
    fsm, r = started()
    r.drive(0.1, signs=('intersection',))
    r.drive(0.1, signs=('left', 'right'))
    assert fsm.state == 'ARMED'


def test_confirm_count_window():
    fsm = MissionFSM(confirm_count=3, confirm_window=1.0, require_arm=False)
    fsm.step(0.0)
    for t in (1.0, 1.5, 2.6):                          # third one is 1.6 s after the first
        fsm.on_sign('right', True, t)
        fsm.step(t)
    assert fsm.state == 'LANE'
    fsm.on_sign('right', True, 2.8)                    # window now holds 2.6, 2.8 -> 2 of 3
    fsm.step(2.8)
    assert fsm.state == 'LANE'
    fsm.on_sign('right', True, 2.9)                    # 2.6, 2.8, 2.9 -> confirmed
    fsm.step(2.9)
    assert fsm.state == 'BRANCH' and fsm.branch == 'right'


def test_bad_params():
    with pytest.raises(KeyError):
        MissionFSM(branch_hold=1.0)
    with pytest.raises(ValueError):
        MissionFSM(left_line='left')
    with pytest.raises(ValueError):
        MissionFSM().on_sign('stop', True, 0.0)


def test_mux_selects_one_source():
    mux = CmdVelMux(source_timeout=0.5, mode_timeout=1.0)
    assert mux.tick(0.0) == (ZERO, 'no drive mode from the FSM')
    mux.set_mode('lane', 0.0)
    assert mux.on_cmd('lane', 0.1, 0.2, 0.05) == (0.1, 0.2)
    assert mux.on_cmd('mission', 0.3, 0.0, 0.05) is None        # not selected
    assert mux.tick(0.1) == (None, 'ok')
    assert mux.tick(0.7) == (ZERO, 'lane silent')               # lane node stopped publishing
    mux.set_mode('mission', 0.7)
    assert mux.on_cmd('mission', 0.3, 0.0, 0.75) == (0.3, 0.0)
    mux.set_mode('stop', 0.8)
    assert mux.on_cmd('mission', 0.3, 0.0, 0.85) is None
    assert mux.tick(0.85) == (ZERO, 'mode stop')


def test_mux_fsm_dead():
    mux = CmdVelMux(source_timeout=0.5, mode_timeout=1.0)
    mux.set_mode('lane', 0.0)
    assert mux.on_cmd('lane', 0.1, 0.0, 1.5) is None            # mode older than 1 s
    assert mux.tick(1.5) == (ZERO, 'no drive mode from the FSM')
    with pytest.raises(ValueError):
        mux.set_mode('teleop', 2.0)

'''Offline checks of the mission FSM and cmd_vel mux cores (no ROS).'''
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from autorace_mission.fsm_core import MissionFSM  # noqa: E402
from autorace_mission.mux_core import ZERO, CmdVelMux  # noqa: E402


def drive(fsm, t, x0, dist, steps=10):
    '''Move along x by dist over `steps` odometry updates.'''
    for k in range(1, steps + 1):
        fsm.on_odom(x0 + dist * k / steps, 0.0)
    return fsm.step(t)


def test_start_go_then_lane():
    fsm = MissionFSM()
    out = fsm.step(0.0)
    assert fsm.state == 'LANE'
    assert out.as_dict() == dict(lane_on=True, follow_line='both', slow=False, drive='lane')


def test_start_wait():
    fsm = MissionFSM(start_mode='traffic_light')
    out = fsm.step(0.0)
    assert fsm.state == 'WAIT_START' and not out.lane_on and out.drive == 'stop'


@pytest.mark.parametrize('side,line', [('left', 'yellow'), ('right', 'white')])
def test_intersection_branch_and_release(side, line):
    fsm = MissionFSM(branch_hold_m=0.9)
    fsm.step(0.0)
    fsm.on_odom(0.0, 0.0)
    fsm.on_sign(side, True, 1.0)
    out = fsm.step(1.0)
    assert fsm.state == 'INTERSECTION'
    assert out.follow_line == line and out.slow and out.lane_on and out.drive == 'lane'
    out = drive(fsm, 2.0, 0.0, 0.5)                    # 0.5 m: still holding the line
    assert fsm.state == 'INTERSECTION' and out.follow_line == line
    out = drive(fsm, 3.0, 0.5, 0.5)                    # 1.0 m >= 0.9 m: back to normal
    assert fsm.state == 'LANE' and out.follow_line == 'both' and not out.slow
    assert fsm.intersection_done


def test_false_is_ignored_and_once():
    fsm = MissionFSM()
    fsm.step(0.0)
    fsm.on_sign('left', False, 0.5)
    fsm.step(0.5)
    assert fsm.state == 'LANE'
    fsm.on_sign('left', True, 1.0)
    fsm.step(1.0)
    fsm.step(30.0)                                     # timeout ends the branch
    assert fsm.state == 'LANE' and fsm.intersection_done
    fsm.on_sign('right', True, 31.0)                   # second intersection sign: ignored
    fsm.step(31.0)
    assert fsm.state == 'LANE'


def test_conflicting_signs_ignored():
    fsm = MissionFSM()
    fsm.step(0.0)
    fsm.on_sign('left', True, 1.0)
    fsm.on_sign('right', True, 1.1)
    fsm.step(1.1)
    assert fsm.state == 'LANE'


def test_confirm_count_window():
    fsm = MissionFSM(confirm_count=3, confirm_window=1.0)
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
    assert fsm.state == 'INTERSECTION' and fsm.branch == 'right'


def test_bad_params():
    with pytest.raises(KeyError):
        MissionFSM(branch_hold=1.0)
    with pytest.raises(ValueError):
        MissionFSM(left_line='left')


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

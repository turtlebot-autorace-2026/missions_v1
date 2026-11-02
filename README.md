# missions_v1 — 미션 FSM + cmd_vel mux

TurtleBot3 AutoRace 2026. 지금 들어 있는 것은 **출발 + 교차로**뿐이다. 나머지 미션(신호등, 차단바, 터널, 공사, 주차)은 같은 구조에 하나씩 추가한다.

## 구조

```text
표지판 인식 노드 ── /left /right (Bool) ──▶ mission_fsm ── /lane_state, /lane/follow_line, /intersection_state ──▶ lane_detection_ad
                                              │
                                              └── /mission/drive (lane|mission|stop) ──▶ cmd_vel_mux
lane_detection_ad ── /cmd_vel_lane ──────────────────────────────────────────────────▶ cmd_vel_mux ──▶ cmd_vel
mission_fsm 직접 기동(주차·터널, 아직 없음) ── /cmd_vel_mission ──────────────────────▶ cmd_vel_mux
```

- `cmd_vel`을 내는 노드는 **mux 하나뿐**이다. 차선 노드와 미션 기동이 동시에 바퀴를 잡는 일이 구조적으로 없다.
- 차선을 따라가며 하는 미션은 차선 노드의 입력만 바꾼다. 따로 움직여야 하는 미션은 차선 노드를 끄고 `/cmd_vel_mission`으로 운전한다.
- mux는 선택된 쪽이 0.5초 조용하거나 FSM이 1초간 아무것도 안 보내면 정지 명령을 낸다.
- **mux가 떠 있는 동안 teleop을 같이 쓰지 말 것** (정지 중에도 0 명령을 계속 보낸다).

## 교차로 동작

1. `/left` 또는 `/right`에 `True`가 오면 (`confirm_count`개, `confirm_window`초 안) 분기 결정
2. 왼쪽 → `left_line`(기본 노란선)만, 오른쪽 → `right_line`(기본 흰선)만 따라간다. 속도는 `v_max_intersection`
3. 결정 지점부터 오도메트리로 `branch_hold_m`(기본 0.9 m)를 달리면 다시 양쪽 선으로. 안전용 시간 제한 `branch_timeout_s`
4. 교차로는 한 번만 (`intersection_once`). `False` 메시지는 무시. 왼쪽·오른쪽이 같은 창 안에 둘 다 오면 무시

**Gazebo에서 확인할 것** (지금 값은 추정):

- 왼쪽 분기에서 노란선, 오른쪽 분기에서 흰선을 따라가는 게 맞는지. 반대면 `left_line`/`right_line`만 바꾼다
- `branch_hold_m`: 분기를 다 돌아 양쪽 선이 다시 정상으로 보이는 지점까지의 거리
- 표지판 인식 노드가 몇 m 앞에서 `True`를 내는지 (멀리서 내면 `branch_hold_m`를 그만큼 늘려야 함)

## 필요한 것

- `lane_detection_ad` **이 커밋 이후 버전** (`/lane/follow_line`, 반복 `/lane_state` 무시)
- 표지판 인식 노드: `/left`, `/right` `std_msgs/Bool`, 여러 프레임 확인 후 `True`. 토픽 이름이 바뀌면 launch 인자로 맞춘다

## 빌드

```bash
cd ~/autorace_ad_ws/src
git clone https://github.com/turtlebot-autorace-2026/missions_v1.git
cd lane_detection_ad && git pull && cd ..
cd ~/autorace_ad_ws
colcon build --symlink-install --packages-select lane_detection_ad autorace_mission
source ~/autorace_ad_ws/install/setup.bash
```

## Gazebo 실행

```bash
# 터미널 1
ros2 launch autorace_gazebo autorace.launch.py referee:=false
# 터미널 2 (바로 차선 주행 시작)
ros2 launch autorace_mission mission_sim.launch.py
# 표지판 노드가 없을 때: 교차로 섬 앞에 왔을 때 손으로
ros2 topic pub --once /left std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /right std_msgs/msg/Bool "{data: true}"
# 상태 보기
ros2 topic echo /mission/state
```

인자: `left_topic:=`, `right_topic:=`, `v_max:=`, `show:=false`, `mission_params:=<yaml>`, `lane_params:=<yaml>`.

로그에서 볼 것: `sign left -> INTERSECTION, follow yellow`, 차선 로그의 `[auto follow=yellow]`, `branch left done (0.90 m, ...) -> LANE`.

## 테스트 (ROS 없이)

```bash
cd ~/autorace_ad_ws/src/missions_v1/autorace_mission && python3 -m pytest -q test
```

## 구성

```text
autorace_mission/
  autorace_mission/fsm_core.py      상태 기계 (ROS 없음)
  autorace_mission/mux_core.py      mux 판단 (ROS 없음)
  autorace_mission/mission_fsm.py   ROS 노드
  autorace_mission/cmd_vel_mux.py   ROS 노드
  config/mission_params.yaml        파라미터
  launch/mission_sim.launch.py      Gazebo (차선 노드 포함, cmd_vel_topic=/cmd_vel_lane)
  test/test_cores.py                pytest
```

실차 launch는 아직 없다 (Gazebo 확인 후 추가).

# missions_v1 — 미션 FSM + cmd_vel mux

TurtleBot3 AutoRace 2026. 지금 들어 있는 것은 **출발 + 교차로**뿐이다. 나머지 미션(신호등, 차단바, 터널, 공사, 주차)은 같은 구조에 하나씩 추가한다.

## 구조

```text
표지판 인식 노드 ── /intersection /left /right (Bool) ──▶ mission_fsm ── /lane_state, /lane/follow_line, /intersection_state ──▶ lane_detection_ad
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

```text
LANE ── /intersection(세모 표지판) ──▶ ARMED ── /left 또는 /right ──▶ BRANCH (한쪽 선만, 감속)
 ▲   (이때 left/right는 무시)          │ arm_timeout_m(4 m) 동안 방향 없음      │
 └─────────────────────────────────────┘                                        │
 └──── 마지막으로 그 표지판을 본 지점부터 branch_hold_m(1.0 m) ◀────────────────┘
```

1. **좌/우 표지판은 INTERSECTION(세모) 표지판을 본 뒤에만 받는다** (`require_arm`). 이 맵은 교차로 섬이 출발 직선에서 약 1.2 m 옆이라, 출발 근처에서 좌/우가 잡히면 엉뚱한 곳에서 분기하고 진짜 교차로를 무시하게 된다. 그걸 막는 장치다.
2. 방향은 처음 확인된 것으로 고정 (분기 중 반대쪽 오검출로 뒤집히지 않음). 같은 창 안에 좌·우가 둘 다 오면 무시
3. 왼쪽 → `left_line`(노란선)만, 오른쪽 → `right_line`(흰선)만. 맵 텍스처로 확인: 왼쪽 분기의 바깥선은 노란 호, 오른쪽 분기의 바깥선은 흰 호 (심판 영역 `branch_left`/`branch_right`와도 일치)
4. 양쪽 선으로 돌아오는 시점: **그 표지판을 마지막으로 본 지점부터** `branch_hold_m`. 좌/우 표지판은 섬(갈림길)에 서 있으므로, 멀리서 일찍 봤든 아니든 마지막 목격 ≈ 갈림길이다. 그래서 인식 노드는 **확인된 동안 매 프레임 `True`**를 보내야 한다 (한 번만 보내면 첫 목격 지점부터 세게 됨)
5. 교차로는 한 번만 (`intersection_once`). 세모 표지판 뒤 4 m 안에 방향 표지판이 없으면 해제하고 다시 기다린다 (교차로를 쓴 걸로 치지 않음)

**Gazebo에서 확인할 것** (지금 값은 추정):

- 섬을 따라 돌 때 한쪽 선만 따라가다 섬의 같은 색 선(섬 왼쪽 아래는 노란색, 위는 흰색)으로 옮겨 타지 않는지
- `branch_hold_m`: 갈림길부터 두 분기가 다시 합쳐지는 지점까지 (텍스처 기준 약 1.0 m)
- 출발 지점에서 세모 표지판이 비스듬히(약 1.7 m, 29° 왼쪽) 잡힐 수 있다. 그러면 출발부터 ARMED가 되는데, 갈림길까지 약 2.9 m라 `arm_timeout_m` 4 m 안이다

## 필요한 것

- `lane_detection_ad` **이 커밋 이후 버전** (`/lane/follow_line`, 반복 `/lane_state` 무시)
- 표지판 인식 노드: `/intersection`(세모), `/left`, `/right` `std_msgs/Bool`. 여러 프레임 확인 후, 보이는 동안 매 프레임 `True`. 토픽 이름이 바뀌면 launch 인자(`arm_topic:=`, `left_topic:=`, `right_topic:=`)로 맞춘다

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
# 표지판 노드가 없을 때 손으로: 세모 표지판 지날 때 arm, 섬 앞에서 방향 (갈림길에서 Ctrl+C = 마지막 목격)
ros2 topic pub --once /intersection std_msgs/msg/Bool "{data: true}"
ros2 topic pub -r 5 /left std_msgs/msg/Bool "{data: true}"
# 상태 보기
ros2 topic echo /mission/state
```

인자: `arm_topic:=`, `left_topic:=`, `right_topic:=`, `v_max:=`, `show:=false`, `mission_params:=<yaml>`, `lane_params:=<yaml>`.

로그에서 볼 것: `INTERSECTION sign -> ARMED`, `sign left -> BRANCH, follow yellow`, 차선 로그의 `[auto follow=yellow]`, `branch left done (1.00 m after last sighting, ...) -> LANE`. 세모 전에 좌/우가 오면 `left/right seen before the INTERSECTION sign -> ignored`.

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

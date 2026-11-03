# missions_v1 — 미션 FSM + cmd_vel mux

TurtleBot3 AutoRace 2026. 신호등 · 교차로 · 공사(장애물) · 주차 · 차단바 · 터널 6개 미션을 **선배 `mission_model`(lane_detection_dl repo)의 표지판 토픽**으로 시작한다. 선배 repo는 수정하지 않는다.

> 오프라인 테스트(28개)만 통과한 상태. **ROS/Gazebo/실차에서는 아직 한 번도 안 돌렸다.** 기본값은 Gazebo world 파일과 코스 텍스처에서 잰 값이다.

## 구조

```text
mission_model (선배) ── /parking /left /right /stop /construction /intersection /tunnel (Bool) ─┐
/odom  /scan  /camera2(앞 카메라)  /lane/status  /lane/centre ─────────────────────────────────┤
                                                                                               ▼
                                                                                         mission_fsm
   /lane_state /lane/follow_line /lane/offset /intersection_state ◀────────────────────────────┤
   /mission/drive (lane|mission|stop) ─────────────────────▶ cmd_vel_mux ──▶ cmd_vel           │
lane_detection_ad ── /cmd_vel_lane ───────────────────────▶      ▲                             │
mission_fsm 직접 기동 ── /cmd_vel_mission ─────────────────────────┘ ◀─────────────────────────┘
```

- `cmd_vel`을 내는 건 **mux 하나**. 선택된 쪽이 0.5초 조용하거나 FSM이 1초 끊기면 정지.
- 차선 노드는 계속 켜 둔다. 정지(`stop`)나 직접 기동(`mission`) 뒤 차선으로 돌아올 때 FSM이 `/lane_state` False→True를 한 번 보내 추정기를 새로 만든다 (안 그러면 막혀 있던 동안의 속도로 바로 튀어 나감).
- **mux가 떠 있는 동안 teleop 금지** (0 명령을 계속 보냄).

## 표지판 입력 (mission_model)

`mission_model`은 Bool을 **상태가 바뀔 때만** 보낸다 (latched). 박스가 이미지 높이의 10 % 이상(가까움)인 프레임이 3번 연속이면 True, 5번 연속 없으면 False. 그래서 여기서는 True~False 사이를 "보이는 중"으로 다룬다 (`signs.py`).

| 표지판 | 시작하는 미션 |
| --- | --- |
| `/intersection` (세모) | 교차로 |
| `/construction` | 공사 |
| `/parking` | 주차 |
| `/stop` | 차단바 |
| `/tunnel` | 터널 |
| `/left` `/right` | **혼자서는 아무것도 안 함**. 교차로 미션 안에서만 방향으로 씀 (출발 근처나 주차장 앞 왼쪽 화살표 무시) |

- 순서는 고정 안 함 (당일 배치 변경 대비). 다른 미션 중에 표지판이 뜨면 대기열에 넣고, 그 뒤 1 m 안에 시작.
- 미션은 각각 한 번. 중단(abort)된 건 다시 시작 가능.
- 토픽 이름은 `sign_topics` 파라미터 (mission_model 순서 그대로).

## 미션

| 미션 | 누가 운전 | 동작 | 맵 근거 |
| --- | --- | --- | --- |
| 신호등 | stop → lane | 앞 카메라 HSV. 빨강을 본 뒤 초록 2프레임 → 출발 | 기본 ROI는 오른쪽 위 |
| 교차로 | lane | 세모 뒤 `/left`·`/right` → 그쪽 선만 따라감(왼쪽=노랑, 오른쪽=흰색), 표지판을 **마지막으로 본 지점**부터 1.0 m | 텍스처: 왼쪽 분기 바깥선 노랑, 오른쪽 흰색. 심판 영역과 일치 |
| 공사 | lane + 직접 | 2차선 폭 구간. 지금 칸 옆 선을 따라가다(안쪽=노랑, 바깥=흰색) LiDAR로 앞이 막히면 오도메트리로 옆 칸으로 이동 | 선 간격 0.574 m, 박스 3개 바깥/안/바깥. 1번 박스가 안쪽 모서리 바로 뒤 → **흰선으로 진입** |
| 주차 | lane + 직접 | 노란선 따라 왼쪽 막다른 길로 → 직진 → LiDAR 좌우로 빈 칸 → 90° → 들어감 → 1.5초 → 후진 → 90° 더 → 나와서 노란선 → 도로 | 칸 폭 0.285 m(3열), 칸 줄 y 0.864, 꺾은 뒤 0.94 m |
| 차단바 | lane → stop → lane | `/stop` 뒤 감속. 바가 내려간 게 보이면(앞 카메라 빨강 가로 폭, 또는 LiDAR 앞) 정지, 올라가면 출발. 25초 넘으면 그냥 출발(30초 규정) | S1→S2 0.229 m, 바 닫힘 1초 |
| 터널 | lane → 직접 → lane | 선이 사라지면(또는 양쪽 벽) 오른쪽 벽 따라가기(0.20 m) + 앞 장애물 회피 → 차선이 다시 보이면 복귀 | 벽 4개·원기둥 3개 world 값 그대로 |

**왜 공사는 "선 바꾸기 + 직접 이동"인가**: 차선 카메라는 옆으로 약 ±0.2 m만 본다. 2차선 폭 구간에서 반대쪽 선은 0.43 m 떨어져 안 보인다. 그래서 각 칸에서는 가까운 선을 따라가고, 칸 사이는 선 없이 짧게 이동한다. 마지막 박스 뒤에는 흰선의 넓은 곡선이 그대로 다음 도로의 차선 중앙(y 2.004)으로 이어진다.

## 오프라인 테스트 (ROS 없이)

```bash
cd ~/autorace_ad_ws/src/missions_v1/autorace_mission && python3 -m pytest -q test
```

| 테스트 | 확인한 것 |
| --- | --- |
| `test_cores.py` (15) | 표지판 레벨 처리, 미션 시작/대기열/1회, 좌우 단독 무시, 신호등, 교차로, 차단바, mux |
| `test_sim.py` (7) | world 파일 기하 + LiDAR 레이캐스트 + 운동학 로봇. 터널 3경우(맵 + 왼벽/아랫벽 경로에 장애물 추가) **충돌 0, 출구 도착**. 주차 3경우(빈칸 → 왼쪽, 왼쪽에 로봇 → 오른쪽, 오른쪽에 로봇 → 왼쪽) **선 안에 완전히 들어감, 충돌 0, 도로 복귀**. 공사: 칸 이동 2번, **박스 충돌 0, 선 침범 0**, 다음 도로 중앙 복귀 |
| `test_vision_lidar.py` (6) | 신호등 색(바닥 노란선 무시), 바 내려감/올라감(STOP 표지판 무시), 벽 직선 맞춤, 기동 함수 |

시뮬레이션에서 차선 노드는 **선 위 경로를 따라가는 대역**으로 바꿨다. 실제 YOLO 마스크, 지연, 바퀴 미끄러짐은 반영 안 됨.

## 빌드

```bash
cd ~/autorace_ad_ws/src
git clone https://github.com/turtlebot-autorace-2026/missions_v1.git        # 처음만
(cd missions_v1 && git pull) && (cd lane_detection_ad && git pull)       # lane_detection_ad도 최신 필요
cd ~/autorace_ad_ws
colcon build --symlink-install --packages-select camera_ros lane_detection_ad autorace_mission
source ~/autorace_ad_ws/install/setup.bash
```

선배 `mission_model`은 선배 repo(lane_detection_dl)를 같은 워크스페이스에 받아 그대로 빌드 (`--packages-select mission_model`).

## Gazebo

```bash
ros2 launch autorace_gazebo autorace.launch.py                       # 터미널 1
ros2 launch mission_model mission_model.launch.py sim:=true          # 터미널 2 (선배 표지판 인식)
ros2 launch autorace_mission mission_sim.launch.py                   # 터미널 3
ros2 topic echo /mission/state                                       # 상태 한 줄
ros2 run rqt_image_view rqt_image_view /mission/debug/compressed     # 신호등/바 판정 화면
```

- 신호등은 앞 카메라(`/camera2`)가 필요하다. Gazebo 로봇에 앞 카메라가 없으면 `start_mode:=go`.
- 선배 camera_ros에 `sim_camera`가 없으면 `front_bridge:=true` (`/front_camera/image_raw` → `/camera2`를 여기서 만듦. 둘 다 켜지 말 것).
- 표지판 노드 없이 손으로: `ros2 topic pub --once /intersection std_msgs/msg/Bool "{data: true}"` (끌 때 `false`).

**Gazebo에서 볼 것** (기본값은 추정):

1. 신호등: 디버그 화면에서 켜진 렌즈에 노란 박스가 뜨는지 → 안 뜨면 `light.roi`, `light.v_min`
2. 교차로: 섬 앞에서 맞는 쪽으로 가는지, 섬 같은 색 선으로 갈아타지 않는지
3. 공사: 흰선으로 진입해 1번 박스 바깥으로, 2번 앞에서 안쪽, 3번 앞에서 바깥 → 곡선 후 차선 중앙. 박스를 긁으면 `beside_back`, `sweep_ahead`, `change_lookahead`
4. 주차: 칸 안에 완전히 들어가는지 → 짧거나 길면 `spur_lane_m` + `lot_drive_m`(합 0.94 m)
5. 차단바: S1 지나 바가 내려올 때 S2 전에 서는지 (`/autorace/race/state`의 `barrier`). 늦으면 `bar.min_width` 낮추기, LiDAR가 바를 보는지 확인
6. 터널: 벽에 안 닿고 출구로 나와 차선으로 돌아오는지

## 실차

```bash
ros2 launch turtlebot3_bringup robot.launch.py                 # 로봇
ros2 launch mission_model mission_model.launch.py              # 선배 표지판 인식 (/camera2)
ros2 launch autorace_mission mission_robot.launch.py           # 카메라 2개 + 차선 + mux + FSM
```

`mission_robot.launch.py`가 camera_ros `publisher`(/camera, 차선)와 `publisher2`(/camera2, 앞, /dev/video2)를 띄운다. 이미 떠 있으면 `cameras:=false`. 첫 주행은 `v_max` 0.08.

Gazebo에 없는 것: 터널 어둠, 주차장 로봇. 이 둘은 실차에서만 확인된다.

## 구성

```text
autorace_mission/
  signs.py        mission_model 표지판 상태 (레벨, 마지막 목격 거리)
  vision.py       신호등 색, 차단바 (HSV 규칙)
  lidar.py        LaserScan → 점, 구역 최소거리, 벽 직선
  motion.py       제자리 회전, 직진, 직선 추종, 벽 따라가기
  missions.py     6개 미션 (ROS 없음)
  fsm_core.py     미션 시작/대기열/순서 (ROS 없음)
  mux_core.py     mux 판단 (ROS 없음)
  mission_fsm.py  ROS 노드
  cmd_vel_mux.py  ROS 노드
config/mission_params.yaml        모든 파라미터 (코드 기본값과 같음)
launch/mission_sim.launch.py      Gazebo
launch/mission_robot.launch.py    실차
test/                             pytest 28개 (simlib.py: 2D 시뮬레이터)
```

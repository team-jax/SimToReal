# SimToReal — 시뮬 관절각을 실물 JAX 팔로 (ROS 2 Jazzy)

시뮬(team-jax/IK의 `motion2/arm/arm.xml`, 동작 1·2가 같이 쓰는 모델)의 관절각을 실물 모터 명령으로 바꾼다. ROS 2 패키지 이름은 `sim2real`이다.

```
/sim2real/joint_command  (sensor_msgs/JointState, 시뮬 관절 이름 elbow·wrist_pitch…, rad)
   ▼
sim2real_node ── calibration.json으로 변환, 모터 명령 속도 제한(기본 30°/s, 50 Hz)
   ├─ AK45 4대 → /ak45/command (ak45_1~4, rad) → ak45_node(team-jax/ak45_ros2)가 10 Hz 재송신 → can0
   └─ 서보 10개 → can0에 CAN_STM 프레임(표준 ID 5) 직접 송신
```

## Ubuntu 24.04 + ROS 2 Jazzy에 올리기

ak45_ros2는 Humble 기준으로 쓰였지만, Jazzy(Ubuntu 24.04, GCC 13)에서도 코드 수정 없이 경고 0으로 빌드된다(2026-11-05, ros:jazzy 이미지에서 확인).

```bash
# 1) 필요한 패키지 (Jazzy는 이미 설치되어 있다고 가정)
sudo apt install -y git can-utils python3-can python3-colcon-common-extensions

# 2) 세 저장소를 형제로 클론 (ak45_ros2가 ../ak45_can을 찾는다)
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/team-jax/ak45_can.git
git clone https://github.com/team-jax/ak45_ros2.git
git clone https://github.com/team-jax/SimToReal.git

# 3) 빌드. --symlink-install이면 calibrate가 저장하는 calibration.json이 저장소 파일 그대로다
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws && colcon build --symlink-install
source install/setup.bash

# 4) CAN 올리기 (재부팅마다 1회) 그리고 피드백 확인
sudo ip link set can0 up type can bitrate 1000000
candump can0      # 00002901~00002904 확장 프레임이 흘러야 한다 (AK45 1~4)
```

`candump`에 피드백이 없으면 ak45_ros2 README §3을 본다(CubeMarsTool의 "Send status over CAN").

## 1단계: 관절 보정 (ak45_node + calibrate)

```bash
# 터미널 1
ros2 launch ak45_ros2 ak45_node.launch.py
# 터미널 2
ros2 run sim2real calibrate --ros
```

`sim2real_node`는 이때 띄우지 않는다. 같은 모터를 두 곳에서 움직이게 된다.

관절 하나를 보정하는 순서:
1. `j elbow`로 관절을 고른다.
2. `m +2`, `m -2`로 움직여 시뮬 0° 자세(팔이 아래로 곧게 늘어진 URDF 영점)에 맞추고 `z`를 누른다.
3. `s 10`으로 보내서 실물이 시뮬과 같은 쪽으로 도는지 본다. 반대면 `f`를 누른다.
4. `v`로 확인 표시를 하고 `w`로 저장한다.

명령 목록은 `sim2real/calibrate.py` 맨 위에 있다. Mac처럼 ROS가 없는 곳에서는 `python calibrate.py`가 dry-run으로 보낼 내용만 출력한다.

## 2단계: 시뮬 각도로 움직이기

```bash
# 터미널 1: ak45_node + sim2real_node
ros2 launch sim2real sim2real.launch.py            # servo_channel:=can0 max_speed_deg_s:=30.0
# 터미널 2: 예) 팔꿈치를 시뮬 0°, 손목 pitch를 20°로
ros2 topic pub -t 3 -r 10 /sim2real/joint_command sensor_msgs/msg/JointState \
  "{name: ['elbow', 'wrist_pitch'], position: [0.0, 0.349]}"
```

- 첫 명령이 오기 전에는 아무것도 보내지 않는다.
- 메시지에 없는 관절은 이전 목표를 유지한다.
- 이름을 모르거나, 길이가 틀리거나, NaN인 메시지는 통째로 버린다(WARN).
- 범위 밖 값은 자르고 WARN을 낸다.
- AK45는 `/joint_states` 피드백 위치에서 출발하고, 피드백이 없으면 움직이지 않는다.
- 서보의 첫 명령은 바로 그 각도로 간다. CAN_STM은 첫 명령 전에 펄스를 내지 않아서 위치를 모른다.

## 3단계: 실물 좌표의 큐브를 집어 옮기기 (회전각 포함)

카메라가 준 큐브 위치·회전각과 놓을 곳으로 시뮬(team-jax/IK 동작 1)을 먼저 돌리고, 그 관절 궤적을 실물로 재생한다.

```bash
# IK 저장소에서: 실물 좌표(어깨 롤 모터 축 원점, x 왼쪽 +, y 뒤 +, m; 회전각은 위에서 볼 때 반시계, °)
JAX_SETUP=real venv/bin/python motion1/scripts/run_real.py --cube 0.10 -0.30 20 --target 0.05 -0.27 --out traj.json
#   닿지 않으면 종료 코드 3, 시뮬에서 실패하면 1, 성공하면 0. traj.json = 관절 13개(rad)의 50 Hz 궤적

# 로봇 컴퓨터에서 (sim2real.launch.py가 떠 있는 상태)
ros2 run sim2real play_trajectory traj.json --dry-run   # 걸리는 시간, 보정 표에 막히는 관절 확인
ros2 run sim2real play_trajectory traj.json             # 첫 자세에서 5 s 기다린 뒤 재생
```

- 시뮬은 실물이 허용하는 속도보다 빠르다(예: `hand_roll` 162°/s). 그래서 가장 빠른 관절이 `--max-speed`(기본 30°/s, `sim2real_node`의 `max_speed_deg_s`와 맞출 것)에 맞도록 전체를 느리게 재생한다. 14 s짜리 배달이 약 80 s가 된다.
- `--dry-run`은 `calibration.json`의 범위에 막혀 잘릴 관절을 알려 준다. **지금 AK45는 ±15°(`limit_deg`)라 실물 높이 배치의 궤적(팔꿈치 76~105°, 상완 회전 11~81° 등)은 대부분 잘린다.** 보정을 마치고 실제 기구 한계를 확인한 뒤 `limit_deg`를 올려야 이 단계가 의미 있다.

## 구성

| 파일 | 내용 |
|---|---|
| `calibration.json` | 관절 → 모터(종류, id), `sign`, `scale`, `zero_deg`, AK45의 `limit_deg`, 시뮬 범위. `verified: false`는 아직 가정이라는 뜻 |
| `sim2real/protocol.py` | CAN 프레임 인코딩과 AK45 피드백 해석. 표준 라이브러리만 쓴다 |
| `sim2real/calibration.py` | `motor_deg = zero_deg + sign × scale × sim_deg`. 시뮬 범위로 자른 다음 모터 범위로 한 번 더 자른다(서보 0~180°, AK45 `zero_deg ± limit_deg`) |
| `sim2real/motion.py` | 속도 제한(`ramp`, `Mover`) |
| `sim2real/ros_io.py` | `/ak45/command`·`/joint_states`, 서보 CAN 송신 |
| `sim2real/node.py` | `sim2real_node` |
| `sim2real/calibrate.py` | 보정 CLI(`ros2 run sim2real calibrate --ros`, 기본은 dry-run) |
| `sim2real/play.py` | `play_trajectory`: IK `run_real.py`의 궤적을 속도 제한에 맞게 느리게 `/sim2real/joint_command`로 보냄 |
| `launch/sim2real.launch.py` | ak45_node(그 패키지의 yaml 그대로)와 sim2real_node |
| `tests/check_protocol.py` | 장치 없이 바이트를 확인한다(Mac에서도 됨). 기대값은 아래 근거 문서에서 가져왔다 |
| `tests/check_node.py`, `tests/docker_check.sh` | ros:jazzy 이미지에서 세 저장소를 빌드하고, 가상 CAN 버스로 sim2real_node를 끝까지 확인한다 |

## 근거

- **서보:** team-jax/CAN_STM(`Stm32servosepc.md`, `src/main.c`, 2026-11-05 `2c9c45f`). 표준 ID 5, `[motor_id][angle×10 H][L]`, `motor_id` 0~9, 0~1800.
- **AK45:** team-jax/ak45_ros2의 `/ak45/command`(rad, `ak45_N` 이름 매칭, 10 Hz 재송신, `docs/ros2.md` §13). 프레임 형식은 CubeMars AK Series Driver Manual V1.0.18의 서보 모드 위치(§5.1.5)와 피드백 0x29(§5.2.1)를 따른다. dry-run과 검사에서 쓴다.
- **어댑터:** CANable(candleLight, gs_usb, 1 Mbps, team-jax/ak45_can `docs/FINDINGS.md`). Linux에서는 `can0`로 바로 잡힌다.

## 가정 (보정하면서 확인하고 `v`로 표시)

| 시뮬 관절 | 모터 |
|---|---|
| `shoulder_roll`, `shoulder_yaw`, `upper_arm_roll`, `elbow` | AK45 id 1, 2, 3, 4 (위에서부터) |
| `wrist_pitch`, `wrist_yaw`, `hand_roll` | 서보 9 W_PITCH, 8 W_YAW, 7 W_X |
| `finger`*k*`_joint1`, `_joint2` | 서보 F*k*_1, F*k*_2 (`_1`이 뿌리 쪽인지는 CAN_STM에서도 미확정) |
| 손가락 회전(시뮬에서 −π 고정) | 서보 6 F_ROT, 시뮬 0 = 그 고정 자세 |

- AK45는 우리 쪽 `limit_deg` 15°(영점 기준 ±15°)로 시작한다.
- ak45_node 자체 상한(`config/ak45_node.yaml`의 `max_command_deg`)은 360이다. yaml 주석은 "2026-08-27 출력축 기준으로 판별 완료"라고 하지만, 같은 저장소의 `docs/ros2.md` §13.9와 README §9는 "V3 미실시"라고 한다. 팀 확인 전까지 `scale`은 1, 상한은 15°로 둔다.

## 아직 안 된 것

- **실물 실행:** Docker Desktop for Mac에는 CAN 소켓이 없어서 ak45_node가 시작하지 못한다. 실물 확인은 Ubuntu 컴퓨터에서 위 순서로 한다.
- **종료:** ak45_node를 끄면 AK45가 힘을 놓는다(ak45_ros2 README §6). 먼저 팔을 받친다. sim2real_node나 calibrate를 끄는 것만으로는 AK45가 마지막 목표를 유지한다(ak45_node가 계속 재송신).
- **궤적은 열린 루프다.** 시뮬에서 계산한 관절각을 그대로 재생하고, 실물 큐브가 미끄러졌는지는 보지 않는다. 실물 높이 배치는 지금 손으로 시뮬에서도 배달 2/100이다(IK `motion1/STATUS.md`).

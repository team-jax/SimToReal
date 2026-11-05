# SimToReal — 시뮬 관절각을 실물 JAX 팔로

시뮬(team-jax/IK의 `motion2/arm/arm.xml`, 동작 1·2가 같이 쓰는 모델)의 관절각을 실물 모터 명령으로 바꿔 CAN으로 보낸다. 지금은 **1단계: 관절 하나씩 보정**이다. 관절마다 시뮬 0°에 해당하는 모터 각도, 방향, 배율을 찾아 `calibration.json`에 적는다.

## 구성

| 파일 | 내용 |
|---|---|
| `calibration.json` | 관절 → 모터(종류, id), `sign`, `scale`, `zero_deg`, AK45의 `limit_deg`, 시뮬 범위. `verified: false`는 아직 가정이라는 뜻 |
| `sim2real/protocol.py` | CAN 프레임 인코딩. 표준 라이브러리만 쓴다 |
| `sim2real/calibration.py` | `motor_deg = zero_deg + sign × scale × sim_deg`. 시뮬 범위로 자른 다음 모터 범위로 한 번 더 자른다(서보 0~180°, AK45 `zero_deg ± limit_deg`) |
| `sim2real/bus.py` | dry-run(프레임을 출력만 함, 기본값)과 python-can 버스, 50 Hz 램프 |
| `calibrate.py` | 보정 CLI. 명령은 파일 docstring에 있다 |
| `tests/check_protocol.py` | 장치 없이 바이트를 확인한다. 기대값은 아래 근거 문서에서 가져왔다 |

## 근거

- **서보:** team-jax/CAN_STM(`Stm32servosepc.md`, `src/main.c`, 2026-11-05 `2c9c45f`)
  - 표준 ID 5, 데이터 `[motor_id][angle×10 H][L]`, `motor_id` 0~9, 0~1800.
  - 펌웨어는 첫 명령이 오기 전에 펄스를 내지 않는다. 그래서 서보에 보내는 첫 명령은 램프 없이 바로 그 각도로 간다.
- **AK45-36:** CubeMars AK Series Driver Manual V1.0.18
  - 서보 모드이고 확장 ID는 `(모드 << 8) | id`다.
  - 위치 모드 4: int32 big-endian, 각도×10000(§5.1.5).
  - 피드백 0x29(§5.2.1).
  - 매뉴얼에 "위치 모드는 최대 속도로 이동한다"고 되어 있다. 그래서 50 Hz로 조금씩 보내는 램프(`--rate`, 기본 30°/s)를 쓴다. team-jax/ak45_can의 `demo_ramp`와 같은 방식이다.
- **어댑터:** CANable(candleLight, gs_usb, 1 Mbps, team-jax/ak45_can `docs/FINDINGS.md`).

## 가정 (보정하면서 확인하고 `v`로 표시)

| 시뮬 관절 | 모터 |
|---|---|
| `shoulder_roll`, `shoulder_yaw`, `upper_arm_roll`, `elbow` | AK45 id 1, 2, 3, 4 (위에서부터) |
| `wrist_pitch`, `wrist_yaw`, `hand_roll` | 서보 9 W_PITCH, 8 W_YAW, 7 W_X |
| `finger`*k*`_joint1`, `_joint2` | 서보 F*k*_1, F*k*_2 (`_1`이 뿌리 쪽인지는 CAN_STM에서도 미확정) |
| 손가락 회전(시뮬에서 −π 고정) | 서보 6 F_ROT, 시뮬 0 = 그 고정 자세 |

- AK45는 `limit_deg` 15°(영점 기준 ±15°)로 시작한다. ak45_can ROS 노드의 기본 상한과 같다. 축 판별 뒤에 올린다.
- **AK45 위치가 출력축 기준인지 로터 기준(36:1)인지는 매뉴얼에 없다**(ak45_can V3, 미해결). `scale` 1로 시작하면, 로터 기준이라도 출력축은 명령의 1/36만 돌아서 안전하다. 처음에 `m +5`로 출력축이 5° 도는지 0.14° 도는지 보고, 0.14°면 `k 36`을 준다.

## 사용

```
python calibrate.py                                      # dry-run: 보낼 프레임만 출력
python tests/check_protocol.py                           # 바이트 확인 (../IK가 있으면 시뮬 범위도 대조)
python calibrate.py --send --interface gs_usb --channel 0   # 실물 (아래 준비 후)
```

python은 IK 저장소의 `venv/bin/python`을 쓰면 된다. 프로토콜과 dry-run은 표준 라이브러리만 쓴다.

보정 순서(관절 하나):
1. `j elbow`로 관절을 고른다.
2. `m +2`, `m -2`로 움직여 시뮬 0° 자세(팔이 아래로 곧게 늘어진 URDF 영점)에 맞추고 `z`를 누른다.
3. `s 10`으로 보내 실물이 시뮬과 같은 쪽으로 도는지 본다. 반대면 `f`를 누른다.
4. `v`로 확인 표시를 하고 `w`로 저장한다.

## 아직 안 된 것

- **실물 전송(`--send`)은 시험하지 않았다.** venv에 `python-can`이 없다. Mac에서 gs_usb는 `pyusb`와 libusb가 필요하다(설치 때 python-can 공식 문서로 확인).
- AK45가 명령을 받지 못하면 어떻게 하는지(타임아웃)는 확인하지 않았다. `q`나 Ctrl-C로 끝내면 모터는 마지막 명령을 유지한다. 힘을 빼는 명령(전류 브레이크)은 일부러 넣지 않았다. 어깨가 중력으로 떨어질 수 있기 때문이다.
- AK45에서 `z`를 누르면 ±15° 창이 새 영점으로 옮겨 간다. 영점을 찾는 동안에는 창이 움직인다.
- 2단계(시뮬 궤적 재생)는 보정이 끝난 뒤에 한다.

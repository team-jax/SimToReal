"""CAN frames for the real arm: the STM32 servo node (CAN_STM) and the AK45-36 actuators.

Pure functions, standard library only, so the byte layout is checked without a bus
(tests/check_protocol.py).

Servo node, team-jax/CAN_STM (Stm32servosepc.md section 3, src/main.c):
  standard 11-bit id 5, data [motor_id][angle_x10 H][angle_x10 L], motor_id 0..9,
  angle_x10 0..1800 (0.0..180.0 deg). The firmware clips out-of-range angles and drops
  motor_id > 9; we clip and reject before sending so the caller knows.

AK45-36, servo mode (CubeMars AK Series Driver Manual V1.0.18, section 5.1/5.2):
  extended 29-bit id = (control mode << 8) | controller id.
  Position mode 4: int32 big-endian, deg * 10000, range +-36000 deg (5.1.5).
  Feedback 0x29: int16 pos * 0.1 deg, int16 speed * 10 ERPM, int16 current * 0.01 A,
  int8 temperature (C), uint8 error code (5.2.1).
  The manual does not say whether the position is on the output shaft or on the rotor
  (36:1); ak45_can calls this V3 and it is still open. calibration.json's `scale` holds it.
"""
from dataclasses import dataclass
import struct

SERVO_STDID = 5
SERVO_COUNT = 10
SERVO_X10_MAX = 1800

AK45_MODE_POS = 4
AK45_FEEDBACK = 0x29
AK45_POS_MAX_DEG = 36000.0

AK45_ERRORS = {0: "none", 1: "motor over-temperature", 2: "over-current", 3: "over-voltage",
               4: "under-voltage", 5: "encoder fault", 6: "MOSFET over-temperature", 7: "motor stall"}


@dataclass(frozen=True)
class Frame:
    arb_id: int
    extended: bool
    data: bytes

    def __str__(self):
        # cansend/candump notation, so a printed frame can be compared with `candump can0`
        ident = f"{self.arb_id:08X}" if self.extended else f"{self.arb_id:03X}"
        return f"{ident}#{self.data.hex().upper()}"


def servo_frame(motor_id: int, angle_deg: float) -> Frame:
    """One servo command. The angle is clipped to 0..180 deg like the firmware does."""
    if not 0 <= motor_id < SERVO_COUNT:
        raise ValueError(f"servo motor_id {motor_id} is not 0..{SERVO_COUNT - 1}")
    x10 = min(max(round(angle_deg * 10), 0), SERVO_X10_MAX)
    return Frame(SERVO_STDID, False, bytes([motor_id, x10 >> 8, x10 & 0xFF]))


def ak45_pos_frame(controller_id: int, deg: float) -> Frame:
    """Position mode (4). The manual's routine truncates pos * 10000 to int32; so do we."""
    if not 0 < controller_id <= 0xFF:
        raise ValueError(f"AK45 controller id {controller_id} is not 1..255")
    if abs(deg) > AK45_POS_MAX_DEG:
        raise ValueError(f"AK45 position {deg} deg is outside +-{AK45_POS_MAX_DEG:g}")
    return Frame((AK45_MODE_POS << 8) | controller_id, True, struct.pack(">i", int(deg * 10000.0)))


@dataclass(frozen=True)
class AK45State:
    controller_id: int
    position_deg: float
    speed_erpm: float
    current_a: float
    temperature_c: int
    error: int


def parse_ak45_feedback(arb_id: int, extended: bool, data: bytes):
    """AK45State for a 0x29 feedback frame, None for anything else."""
    if not extended or len(data) < 8 or (arb_id >> 8) != AK45_FEEDBACK:
        return None
    pos, spd, cur, temp, err = struct.unpack(">hhhbB", bytes(data[:8]))
    return AK45State(arb_id & 0xFF, pos * 0.1, spd * 10.0, cur * 0.01, temp, err)

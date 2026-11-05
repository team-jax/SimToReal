"""calibration.json: which motor drives each sim joint, and how a sim angle becomes a motor angle.

motor_deg = zero_deg + sign * scale * sim_deg, after clipping sim_deg to the joint's sim range;
then the motor's own bounds: servo 0..180 deg, AK45 zero_deg +- limit_deg.
"""
from dataclasses import dataclass, asdict, field
import json
from pathlib import Path

from .protocol import Frame, servo_frame, ak45_pos_frame

DEFAULT_FILE = Path(__file__).resolve().parent.parent / "calibration.json"
SERVO_RANGE = (0.0, 180.0)


@dataclass
class Joint:
    joint: str
    motor: str            # "ak45" or "servo"
    id: int
    name: str
    sign: int
    scale: float
    zero_deg: float
    sim_range_deg: list
    verified: bool = False
    limit_deg: float = None   # AK45 only
    note: str = None

    def motor_bounds(self):
        if self.motor == "servo":
            return SERVO_RANGE
        return (self.zero_deg - self.limit_deg, self.zero_deg + self.limit_deg)

    def to_motor(self, sim_deg):
        """(motor_deg, clipped) for a sim angle in degrees."""
        lo, hi = self.sim_range_deg
        s = min(max(sim_deg, lo), hi)
        m = self.zero_deg + self.sign * self.scale * s
        mlo, mhi = self.motor_bounds()
        mc = min(max(m, mlo), mhi)
        return mc, (s != sim_deg or mc != m)

    def to_sim(self, motor_deg):
        return (motor_deg - self.zero_deg) / (self.sign * self.scale)

    def frame(self, motor_deg) -> Frame:
        if self.motor == "servo":
            return servo_frame(self.id, motor_deg)
        return ak45_pos_frame(self.id, motor_deg)


@dataclass
class Calibration:
    about: str
    joints: list = field(default_factory=list)
    path: Path = None

    def __getitem__(self, name) -> Joint:
        for j in self.joints:
            if name in (j.joint, j.name):
                return j
        raise KeyError(f"no joint or motor named {name!r}")

    def save(self, path=None):
        path = Path(path or self.path)
        rows = [{k: v for k, v in asdict(j).items() if v is not None} for j in self.joints]
        lines = ",\n".join("    " + json.dumps(r, ensure_ascii=False) for r in rows)
        path.write_text(f'{{\n  "about": {json.dumps(self.about, ensure_ascii=False)},\n  "joints": [\n{lines}\n  ]\n}}\n')


def default_file():
    """The repository's calibration.json (also under colcon build --symlink-install), else the
    copy installed in the package share directory."""
    if DEFAULT_FILE.exists():
        return DEFAULT_FILE
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("sim2real")) / "calibration.json"


def load(path=None) -> Calibration:
    path = Path(path) if path else default_file()
    raw = json.loads(path.read_text())
    joints = [Joint(**j) for j in raw["joints"]]
    for j in joints:
        if j.motor not in ("ak45", "servo"):
            raise ValueError(f"{j.joint}: motor must be ak45 or servo, not {j.motor}")
        if j.sign not in (1, -1) or j.scale <= 0:
            raise ValueError(f"{j.joint}: sign must be +-1 and scale > 0")
        if j.motor == "ak45" and not j.limit_deg:
            raise ValueError(f"{j.joint}: AK45 joints need limit_deg")
    ids = [(j.motor, j.id) for j in joints]
    if len(set(ids)) != len(ids):
        raise ValueError("two joints share a motor id")
    return Calibration(raw["about"], joints, path)

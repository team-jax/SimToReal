"""The sim planner for pick_place: team-jax/IK motion1, run on this computer (needs MuJoCo).

humanoid_grasp_rl.real_task.plan_delivery runs the delivery in the sim from real coordinates and
returns the joint trajectory with its verdict ("status": playable, retried, not delivered,
unreachable). This module only finds the IK repository and imports it in its real setup.

IK is looked for in --ik, then $JAX_IK_DIR, then ~/IK, then next to this repository. It needs the
branch that has real_task.py (sim-table-friction) and mujoco, gymnasium and scipy for this Python.
"""
import os
import sys
from pathlib import Path

IK_ENV = "JAX_IK_DIR"
MARKER = Path("motion1/src/humanoid_grasp_rl/real_task.py")


def find_ik(path=None) -> Path:
    candidates = [path, os.environ.get(IK_ENV), Path.home() / "IK", Path(__file__).resolve().parents[2] / "IK"]
    for c in candidates:
        if c and (Path(c) / MARKER).is_file():
            return Path(c).resolve()
    raise FileNotFoundError(
        "team-jax/IK with motion1 real_task.py not found. Clone it (git clone -b sim-table-friction "
        "https://github.com/team-jax/IK.git ~/IK), or pass --ik / set JAX_IK_DIR")


def plan_delivery(cube, target, d=None, ik=None) -> dict:
    """cube = (x, y, yaw), target = (x, y): real frame, metres and degrees (see the IK docs)."""
    root = find_ik(ik)
    if os.environ.get("JAX_SETUP", "real") != "real":
        raise RuntimeError(f"JAX_SETUP={os.environ['JAX_SETUP']}: real coordinates need the real setup")
    os.environ["JAX_SETUP"] = "real"     # read once, when humanoid_grasp_rl.env is first imported
    sys.path.insert(0, str(root / "motion1" / "src"))
    from humanoid_grasp_rl.real_task import plan_delivery as run
    return run(cube, target, d)

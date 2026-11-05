"""pick_place: a cube on the real table, picked and set down, from its position and yaw alone.

    ros2 run sim2real pick_place --cube 0.10 -0.30 20 --target 0.05 -0.27 --dry-run
    ros2 run sim2real pick_place --cube 0.10 -0.30 20 --target 0.05 -0.27      # sim2real_node running

--cube x y yaw, --target x y: the real frame of team-jax/IK realcoords.py - origin on the shoulder
roll motor's axis, x to the robot's left, y backwards (the front is -y), metres; yaw in degrees,
counter-clockwise seen from above, of one face's normal from +x (a cube repeats every 90 degrees).

1. plan:  the delivery runs in the sim here (planner.py: IK motion1 real_task.plan_delivery)
2. check: a clean delivery in the sim, no joint cut short by the calibration, the calibration
          verified (play.check_trajectory); anything wrong stops it before the arm moves (--force)
3. play:  slowed to the speed limit, the AK45s watched (play.run_on_robot)

Exit code: 0 done (or dry run passed), 1 refused, 3 the arm cannot serve it, 5 stopped during the
playback.
"""
import argparse
import json

from .play import add_run_args, checked_run
from .planner import plan_delivery


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cube", type=float, nargs=3, required=True, metavar=("X", "Y", "YAW"))
    ap.add_argument("--target", type=float, nargs=2, required=True, metavar=("X", "Y"))
    ap.add_argument("--d", type=float, default=None, help="shoulder axis to table top, m (default: IK's real setup)")
    ap.add_argument("--ik", default=None, help="team-jax/IK checkout (default: $JAX_IK_DIR, ~/IK, next to this repo)")
    ap.add_argument("--save", default=None, help="also write the planned trajectory here (JSON)")
    add_run_args(ap)
    a, _ = ap.parse_known_args(argv)

    print("planning in the sim ...", flush=True)
    r = plan_delivery(a.cube, a.target, a.d, a.ik)
    s = r["sim"]
    print(f"cube -> sim {[round(v, 4) for v in s['cube']]}, yaw {s['cube_yaw_deg']:+.1f} deg; target -> sim "
          f"{[round(v, 4) for v in s['target']]}")
    if r["status"] == "unreachable":
        print(f"refused: {r['reason']}")
        raise SystemExit(3)
    print(f"sim: {r['status']} ({r['reason']}), hand turned {s['hand_yaw_deg']:+.1f} deg, "
          f"{r['final_error_m'] * 1000:.1f} mm from the target"
          + (", retries " + ", ".join(f"{x['phase']}@{x['step']}" for x in r["retries"]) if r["retries"] else ""))
    if a.save:
        with open(a.save, "w") as f:
            json.dump(r, f, indent=1)
        print(f"wrote {a.save}")
    raise SystemExit(checked_run(r, a))


if __name__ == "__main__":
    main()

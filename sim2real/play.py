"""play_trajectory: send a joint trajectory to /sim2real/joint_command, slow enough for the robot.

The file comes from team-jax/IK `motion1/scripts/run_real.py` (a delivery given in real coordinates,
run in the sim): {"joint_names": [...], "dt": s, "positions": [[rad, ...], ...]}, sim joint names.

    ros2 run sim2real play_trajectory trajectory.json                  # with sim2real_node running
    ros2 run sim2real play_trajectory trajectory.json --dry-run        # print the schedule only

The sim moves faster than the robot may (sim2real_node holds every motor to max_speed_deg_s), so the
frames are played slower by peak joint speed / max_speed: then no joint is cut short and the path
keeps its shape. The first frame is held for --hold seconds first, for the arm to get there.
"""
import argparse
import json
import threading
import time
from math import degrees

COMMAND_TOPIC = "/sim2real/joint_command"


def schedule(traj, max_speed_deg_s):
    """(seconds between frames, slow-down factor, peak joint speed of the sim in deg/s)."""
    q, dt = traj["positions"], traj["dt"]
    peak = max((abs(degrees(b - a)) / dt for prev, cur in zip(q, q[1:]) for a, b in zip(prev, cur)),
               default=0.0)
    slow = max(1.0, peak / max_speed_deg_s)
    return dt * slow, slow, peak


def clipped_joints(traj, cal):
    """[(joint, sim min deg, sim max deg, what the calibration allows)] for every joint of the
    trajectory that sim2real_node would cut short (calibration.Joint.to_motor)."""
    out = []
    for k, name in enumerate(traj["joint_names"]):
        j = cal[name]
        sim = [degrees(f[k]) for f in traj["positions"]]
        # 0.05 deg of slack: a joint the sim holds on its stop reads a hair past it in floats
        if any(abs(j.to_motor(v)[0] - (j.zero_deg + j.sign * j.scale * v)) > 0.05 for v in sim):
            lo, hi = j.motor_bounds()
            allowed = sorted((j.to_sim(lo), j.to_sim(hi)))
            allowed = [max(allowed[0], j.sim_range_deg[0]), min(allowed[1], j.sim_range_deg[1])]
            out.append((name, min(sim), max(sim), allowed))
    return out


def play(publish, traj, max_speed_deg_s, hold, sleep=time.sleep, stop=None):
    """Calls publish(names, positions) for every frame; the first one over `hold` seconds."""
    period, _, _ = schedule(traj, max_speed_deg_s)
    names, frames = traj["joint_names"], traj["positions"]
    for _ in range(max(1, int(hold * 10))):
        publish(names, frames[0])
        sleep(0.1)
    for frame in frames[1:]:
        if stop is not None and stop.is_set():
            return
        publish(names, frame)
        sleep(period)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("--max-speed", type=float, default=30.0,
                    help="deg/s; keep it at sim2real_node's max_speed_deg_s or below")
    ap.add_argument("--hold", type=float, default=5.0, help="seconds on the first frame before playing")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--calibration", default=None, help="calibration.json (default: the repository's)")
    a, _ = ap.parse_known_args(argv)
    traj = json.loads(open(a.file).read())
    period, slow, peak = schedule(traj, a.max_speed)
    n = len(traj["positions"])
    print(f"{n} frames, sim {n * traj['dt']:.1f} s, peak joint speed {peak:.0f} deg/s -> played {slow:.1f}x "
          f"slower: {period * 1000:.0f} ms a frame, {a.hold + (n - 1) * period:.1f} s with the hold")
    if traj.get("delivered_in_sim") is False:
        print("warning: this delivery failed in the sim")
    from . import calibration
    cal = calibration.load(a.calibration)
    for name, lo, hi, allowed in clipped_joints(traj, cal):
        print(f"warning: {name} goes {lo:.1f}..{hi:.1f} deg but {cal.path.name} allows "
              f"{allowed[0]:.1f}..{allowed[1]:.1f}: sim2real_node will cut it short")
    if a.dry_run:
        return
    import rclpy
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = rclpy.create_node("sim2real_play")
    pub = node.create_publisher(JointState, COMMAND_TOPIC, 10)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def publish(names, positions):
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name, msg.position = list(names), [float(p) for p in positions]
        pub.publish(msg)

    try:
        play(publish, traj, a.max_speed, a.hold)
        print("done; the robot holds the last frame")
    except KeyboardInterrupt:
        print("\nstopped; the robot holds the last frame it was sent")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

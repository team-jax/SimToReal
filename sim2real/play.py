"""Playing a joint trajectory on the robot, and play_trajectory, which plays one from a file.

A trajectory is what team-jax/IK motion1 real_task.plan_delivery returns (or run_real.py writes):
{"joint_names": [...], "dt": s, "positions": [[rad, ...], ...], "status": "playable", ...}, sim names.

    ros2 run sim2real play_trajectory trajectory.json --dry-run     # the checks and the schedule only
    ros2 run sim2real play_trajectory trajectory.json               # with sim2real_node running

Before anything moves, check_trajectory lists what is wrong with it: not a clean delivery in the sim,
joints the calibration would cut short, joints whose calibration is not verified. Any of them stops
the run unless --force (a run that was not delivered or not reachable never plays).

The sim moves faster than the robot may (sim2real_node holds every motor to max_speed_deg_s), so the
frames are played slower by peak joint speed / max speed: no joint is cut short and the path keeps its
shape. The first frame is held for `hold` seconds, for the arm to get there. During the playback the
AK45s are watched (watch.TrackingWatch); one that stops following stops the playback, and the arm is
told to stay where it is.
"""
import argparse
import json
import threading
import time
from math import degrees, radians

COMMAND_TOPIC = "/sim2real/joint_command"
NEVER = ("not delivered", "unreachable")


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


def check_trajectory(traj, cal):
    """(fatal, problems): fatal is a reason it must never play, problems need --force."""
    status = traj.get("status") or ("playable" if traj.get("playable", True) else "not delivered")
    if traj.get("delivered_in_sim") is False:
        status = "not delivered"
    if status in NEVER:
        return f"{status}: {traj.get('reason', 'failed in the sim')}", []
    problems = []
    if status != "playable" or traj.get("playable") is False:
        problems.append(f"{status}: {traj.get('reason', 'not a clean delivery')} (a sim retry would replay on the robot)")
    for name, lo, hi, allowed in clipped_joints(traj, cal):
        problems.append(f"{name} goes {lo:.1f}..{hi:.1f} deg but the calibration allows "
                        f"{allowed[0]:.1f}..{allowed[1]:.1f}: it would be cut short")
    unverified = [n for n in traj["joint_names"] if not cal[n].verified]
    if unverified:
        problems.append("calibration not verified (zero and direction are assumptions): " + ", ".join(unverified))
    return None, problems


def play(publish, traj, max_speed_deg_s, hold, sleep=time.sleep, stop=None, watch=None):
    """publish(names, positions) for every frame, the first over `hold` seconds. watch(frame), when
    given, is asked after each frame and a reason it returns ends the playback: returned here."""
    period, _, _ = schedule(traj, max_speed_deg_s)
    names, frames = traj["joint_names"], traj["positions"]
    for _ in range(max(1, int(hold * 10))):
        publish(names, frames[0])
        sleep(0.1)
    for frame in frames[1:]:
        if stop is not None and stop.is_set():
            return "stopped"
        publish(names, frame)
        sleep(period)
        reason = watch(frame) if watch else None
        if reason:
            return reason
    return None


def report(traj, cal, max_speed):
    """Prints the schedule and the checks; returns (fatal, problems)."""
    period, slow, peak = schedule(traj, max_speed)
    n = len(traj["positions"])
    print(f"{n} frames, sim {n * traj['dt']:.1f} s, peak joint speed {peak:.0f} deg/s -> played {slow:.1f}x "
          f"slower at {max_speed:g} deg/s: {period * 1000:.0f} ms a frame, {(n - 1) * period:.1f} s")
    fatal, problems = check_trajectory(traj, cal)
    if fatal:
        print(f"refused: {fatal}")
    for p in problems:
        print(f"check: {p}")
    if not fatal and not problems:
        print("checks: none")
    return fatal, problems


def run_on_robot(traj, cal, max_speed, hold, max_error, node_name="sim2real_play"):
    """Plays a checked trajectory through sim2real_node with the AK45s watched. Returns the reason it
    stopped early, or None when it played to the end."""
    import rclpy
    from sensor_msgs.msg import JointState
    from .ros_io import Ak45Topics, ak45_topic_name
    from .watch import TrackingWatch

    rclpy.init()
    node = rclpy.create_node(node_name)
    pub = node.create_publisher(JointState, COMMAND_TOPIC, 10)
    ak45 = Ak45Topics(node, command=False)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    names = traj["joint_names"]
    motors = [(k, cal[n]) for k, n in enumerate(names) if cal[n].motor == "ak45"]
    tracker = TrackingWatch(max_error)

    def publish(names_, positions):
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name, msg.position = list(names_), [float(p) for p in positions]
        pub.publish(msg)

    def watch(frame):
        expected = {ak45_topic_name(j): j.to_motor(degrees(frame[k]))[0] for k, j in motors}
        feedback = {ak45_topic_name(j): ak45.feedback_deg(j) for _, j in motors}
        return tracker.check(expected, feedback, time.monotonic())

    try:
        reason = play(publish, traj, max_speed, hold, watch=watch)
        if reason:
            # Stay where the arm is: each AK45 told its present angle, the servos keep their last one.
            here = [(j.joint, ak45.feedback_deg(j)) for _, j in motors]
            here = [(n, radians(cal[n].to_sim(fb))) for n, fb in here if fb is not None]
            if here:
                publish([n for n, _ in here], [p for _, p in here])
                time.sleep(0.3)
        return reason
    except KeyboardInterrupt:
        return "interrupted"
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def add_run_args(ap):
    ap.add_argument("--max-speed", type=float, default=30.0,
                    help="deg/s; keep it at sim2real_node's max_speed_deg_s or below")
    ap.add_argument("--hold", type=float, default=5.0, help="seconds on the first frame before playing")
    ap.add_argument("--max-error", type=float, default=15.0,
                    help="deg an AK45 may lag its command (for 0.5 s) before the playback stops")
    ap.add_argument("--dry-run", action="store_true", help="checks and schedule only")
    ap.add_argument("--force", action="store_true", help="play despite the checks (never a failed run)")
    ap.add_argument("--calibration", default=None, help="calibration.json (default: the repository's)")


def checked_run(traj, a):
    """report -> refuse or play. Exit code: 0 played to the end, 1 refused, 5 stopped early."""
    from . import calibration
    cal = calibration.load(a.calibration)
    fatal, problems = report(traj, cal, a.max_speed)
    if fatal or (problems and not a.force):
        if problems and not fatal:
            print("refused: fix the above or pass --force")
        return 1
    if a.dry_run:
        return 0
    reason = run_on_robot(traj, cal, a.max_speed, a.hold, a.max_error)
    if reason:
        print(f"stopped: {reason}; the arm was told to stay where it is")
        return 5
    print("done; the robot holds the last frame")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    add_run_args(ap)
    a, _ = ap.parse_known_args(argv)   # ros2 run adds --ros-args
    raise SystemExit(checked_run(json.loads(open(a.file).read()), a))


if __name__ == "__main__":
    main()

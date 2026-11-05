"""Calibrate one joint at a time: find the motor angle that matches sim 0, the direction, the scale.

Dry run by default: prints what would be sent. --ros sends for real: AK45s through ak45_node
(start it first, and do not run sim2real_node at the same time), servos onto can0.

    python calibrate.py                     # dry run, anywhere (standard library only)
    ros2 run sim2real calibrate --ros       # on the robot computer

Commands (angles in degrees):
    j NAME      select a joint (sim name like elbow, or motor name like W_PITCH)
    m DEG       move the motor to DEG;  m +N / m -N  move it by N
    s DEG       move to the sim angle DEG through the current calibration
    z           this motor angle is sim 0  (zero_deg)
    f           flip the direction  (sign)
    k X         scale
    v           mark the joint verified
    p           print the joint and, for an AK45, its feedback
    w           write calibration.json
    q           quit. Motors keep their last command: ak45_node keeps resending AK45 targets
                until it is stopped (stopping it releases them, ak45_ros2 README section 6)

Every move ramps at --rate deg/s from the last known position. AK45s start from their feedback and
do not move without it; a servo's first command jumps (CAN_STM sends no pulse until then).
"""
import argparse
import threading
import time
from math import radians

from . import calibration
from .motion import ramp
from .protocol import servo_frame

HZ = 50


class DryOutput:
    """Prints what --ros would send. No feedback."""
    live = False

    def __init__(self, out=print):
        self.out = out
        self.sent = []

    def send(self, joint, deg):
        if joint.motor == "ak45":
            s = f"/ak45/command ak45_{joint.id} {radians(deg):.5f} rad ({deg:g} deg)"
        else:
            s = f"can0 {servo_frame(joint.id, deg)}"
        self.sent.append(s)
        self.out(f"  dry-run {s}")

    def feedback(self, joint):
        return None

    def close(self):
        pass


class RosOutput:
    live = True

    def __init__(self, node, servo):
        from .ros_io import Ak45Topics
        self.ak45 = Ak45Topics(node)
        self.servo = servo

    def send(self, joint, deg):
        if joint.motor == "ak45":
            self.ak45.publish([(joint, deg)])
        else:
            self.servo.send(servo_frame(joint.id, deg))

    def feedback(self, joint):
        return self.ak45.feedback_deg(joint) if joint.motor == "ak45" else None

    def close(self):
        self.servo.close()


class Session:
    def __init__(self, cal, output, rate, out=print, sleep=time.sleep):
        self.cal, self.output, self.rate, self.out, self.sleep = cal, output, rate, out, sleep
        self.joint = None
        self.pos = {}    # last commanded motor angle per joint

    def current(self, j):
        if j.motor == "ak45":
            fb = self.output.feedback(j)
            if fb is not None:
                return fb
        return self.pos.get(j.joint)

    def move_motor(self, target):
        j = self.joint
        lo, hi = j.motor_bounds()
        if not lo <= target <= hi:
            self.out(f"  {target:g} is outside {j.name}'s bounds {lo:g}..{hi:g}; not moving")
            return
        start = self.current(j)
        if start is None:
            if j.motor == "ak45" and self.output.live:
                self.out(f"  no /joint_states for ak45_{j.id}; is ak45_node running? not moving")
                return
            path = [target]
        else:
            path = ramp(start, target, self.rate, HZ)
        for p in path:
            self.output.send(j, p)
            self.sleep(1 / HZ)
        self.pos[j.joint] = target
        self.out(f"  {j.name} at {target:g} deg (sim {j.to_sim(target):g})")

    def handle(self, line):
        parts = line.split()
        if not parts:
            return True
        cmd, arg = parts[0], (parts[1] if len(parts) > 1 else None)
        if cmd == "q":
            return False
        if cmd == "w":
            self.cal.save()
            self.out(f"  wrote {self.cal.path}")
            return True
        if cmd == "j":
            self.joint = self.cal[arg]
            self.out(f"  {self.joint}")
            return True
        if self.joint is None:
            self.out("  select a joint first: j NAME")
            return True
        j = self.joint
        if cmd == "m":
            here = self.current(j)
            here = j.zero_deg if here is None else here
            self.move_motor(here + float(arg) if arg[0] in "+-" else float(arg))
        elif cmd == "s":
            m, clipped = j.to_motor(float(arg))
            if clipped:
                self.out(f"  sim {arg} clipped to motor {m:g}")
            self.move_motor(m)
        elif cmd == "z":
            here = self.current(j)
            if here is None:
                self.out("  move the motor first")
                return True
            j.zero_deg = round(here, 2)
            self.out(f"  {j.joint}: zero_deg = {j.zero_deg:g}")
        elif cmd == "f":
            j.sign = -j.sign
            self.out(f"  {j.joint}: sign = {j.sign}")
        elif cmd == "k":
            j.scale = float(arg)
            self.out(f"  {j.joint}: scale = {j.scale:g}")
        elif cmd == "v":
            j.verified = True
            self.out(f"  {j.joint}: verified")
        elif cmd == "p":
            self.out(f"  {j}\n  last command {self.pos.get(j.joint)}")
            if j.motor == "ak45":
                self.out(f"  feedback {self.output.feedback(j)}")
        else:
            self.out(f"  unknown command {cmd!r}")
        return True


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ros", action="store_true", help="send for real (default: dry run)")
    ap.add_argument("--servo-interface", default="socketcan", help="python-can interface for the servos")
    ap.add_argument("--servo-channel", default="can0")
    ap.add_argument("--rate", type=float, default=30.0, help="ramp speed, deg/s of the motor command")
    ap.add_argument("--file", default=None, help="calibration.json (default: the repository's)")
    a, _ = ap.parse_known_args(argv)   # ros2 run adds --ros-args
    cal = calibration.load(a.file)
    if a.ros:
        import rclpy
        from .ros_io import ServoCan
        rclpy.init()
        node = rclpy.create_node("sim2real_calibrate")
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        output = RosOutput(node, ServoCan(a.servo_interface, a.servo_channel))
        print(f"sending: AK45 via /ak45/command, servos on {a.servo_interface} {a.servo_channel}")
    else:
        output = DryOutput()
        print("dry run: nothing is sent")
    print(f"calibration: {cal.path}")
    s = Session(cal, output, a.rate)
    try:
        while True:
            try:
                line = input(f"[{s.joint.joint if s.joint else '-'}] > ")
            except EOFError:
                break
            try:
                if not s.handle(line):
                    break
            except (KeyError, ValueError, TypeError) as e:
                print(f"  {e}")
    except KeyboardInterrupt:
        print("\n  stopped; motors keep their last command")
    finally:
        output.close()
        if a.ros:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Calibrate one joint at a time: find the motor angle that matches sim 0, the direction, the scale.

Dry run by default (frames are printed, nothing is sent). --send puts them on the CAN bus.

    python calibrate.py                                  # dry run
    python calibrate.py --send --interface gs_usb --channel 0

Commands (angles in degrees):
    j NAME      select a joint (sim name like elbow, or motor name like W_PITCH)
    m DEG       move the motor to DEG;  m +N / m -N  move it by N
    s DEG       move to the sim angle DEG through the current calibration
    z           this motor angle is sim 0  (zero_deg)
    f           flip the direction  (sign)
    k X         scale (AK45: 36 if the position turns out to be on the rotor)
    v           mark the joint verified
    p           print the joint and, for an AK45, its feedback
    w           write calibration.json
    q           quit (motors keep their last command)

Every move ramps at --rate deg/s from the last known position. AK45s start from their feedback;
a servo's first command jumps (CAN_STM sends no pulse until then, so its position is unknown).
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim2real import calibration
from sim2real.bus import DryBus, CanBus, ramp, SEND_HZ


class Session:
    def __init__(self, cal, bus, rate, out=print, sleep=time.sleep):
        self.cal, self.bus, self.rate, self.out, self.sleep = cal, bus, rate, out, sleep
        self.joint = None
        self.pos = {}    # last commanded motor angle per joint

    def current(self, j):
        if j.motor == "ak45":
            fb = self.bus.feedback(j.id)
            if fb is not None:
                if fb.error:
                    raise RuntimeError(f"{j.name} reports error {fb.error}; not moving it")
                return fb.position_deg
        return self.pos.get(j.joint)

    def move_motor(self, target):
        j = self.joint
        lo, hi = j.motor_bounds()
        if not lo <= target <= hi:
            self.out(f"  {target:g} is outside {j.name}'s bounds {lo:g}..{hi:g}; not moving")
            return
        start = self.current(j)
        if start is None:
            if j.motor == "ak45" and isinstance(self.bus, CanBus):
                self.out(f"  no feedback from {j.name}; not moving")
                return
            path = [target]
        else:
            path = ramp(start, target, self.rate)
        for p in path:
            self.bus.send(j.frame(p))
            self.sleep(1 / SEND_HZ)
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
                self.out(f"  feedback {self.bus.feedback(j.id)}")
        else:
            self.out(f"  unknown command {cmd!r}")
        return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--send", action="store_true", help="send on the CAN bus (default: dry run)")
    ap.add_argument("--interface", default="gs_usb", help="python-can interface")
    ap.add_argument("--channel", default="0", help="python-can channel")
    ap.add_argument("--rate", type=float, default=30.0, help="ramp speed, deg/s of the motor command")
    ap.add_argument("--file", default=calibration.DEFAULT_FILE)
    a = ap.parse_args()
    cal = calibration.load(a.file)
    bus = CanBus(a.interface, a.channel) if a.send else DryBus()
    s = Session(cal, bus, a.rate)
    print("dry run: nothing is sent" if not a.send else f"sending on {a.interface} {a.channel}")
    try:
        while True:
            try:
                line = input(f"[{s.joint.joint if s.joint else '-'}] > ")
            except EOFError:
                break
            try:
                if not s.handle(line):
                    break
            except (KeyError, ValueError, TypeError, RuntimeError) as e:
                print(f"  {e}")
    except KeyboardInterrupt:
        print("\n  stopped; motors keep their last command")
    finally:
        bus.close()


if __name__ == "__main__":
    main()

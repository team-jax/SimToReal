"""Byte-level checks with no device. Exit code 0 when everything passes.

    python tests/check_protocol.py [--ik ../IK]

Expected bytes come from the sources, not from this code:
  servo frames   - team-jax/CAN_STM Stm32servosepc.md section 8 (cansend examples)
  AK45 position  - CubeMars manual V1.0.18 5.1.5: int32 big-endian, deg * 10000
  AK45 feedback  - manual 5.2.1, and ak45_can docs/FINDINGS.md (payload FF 9E -> -9.8 deg)
"""
import argparse
import sys
import xml.etree.ElementTree as ET
from math import degrees, radians
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim2real.protocol import servo_frame, ak45_pos_frame, parse_ak45_feedback
from sim2real import calibration
from sim2real.motion import ramp, Mover
from sim2real.calibrate import Session, DryOutput

failures = []


def check(name, got, want):
    ok = got == want
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {got}" + ("" if ok else f" (want {want})"))
    if not ok:
        failures.append(name)


def raises(name, fn):
    try:
        fn()
    except ValueError:
        print(f"PASS  {name}: rejected")
        return
    print(f"FAIL  {name}: accepted")
    failures.append(name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ik", default=str(ROOT.parent / "IK"), help="IK repository, for the sim joint ranges")
    a = ap.parse_args()

    # servo node (CAN_STM spec section 8)
    check("servo 0 -> 0 deg", str(servo_frame(0, 0)), "005#000000")
    check("servo 3 -> 90 deg", str(servo_frame(3, 90)), "005#030384")
    check("servo 9 -> 180 deg", str(servo_frame(9, 180)), "005#090708")
    check("servo 3 -> 200 deg is clipped to 180", str(servo_frame(3, 200)), "005#030708")
    check("servo 3 -> -5 deg is clipped to 0", str(servo_frame(3, -5)), "005#030000")
    check("servo 4 -> 22.84 deg rounds to 228", str(servo_frame(4, 22.84)), "005#0400E4")
    raises("servo motor_id 10", lambda: servo_frame(10, 90))

    # AK45 position mode 4: id (4 << 8) | controller, int32 deg * 10000
    check("AK45 2 -> 10 deg", str(ak45_pos_frame(2, 10)), "00000402#000186A0")
    check("AK45 1 -> -5 deg", str(ak45_pos_frame(1, -5)), "00000401#FFFF3CB0")
    check("AK45 4 -> 360 deg", str(ak45_pos_frame(4, 360)), "00000404#0036EE80")
    check("AK45 frame is extended", ak45_pos_frame(3, 1).extended, True)
    raises("AK45 beyond +-36000 deg", lambda: ak45_pos_frame(1, 36001))
    raises("AK45 controller id 0", lambda: ak45_pos_frame(0, 1))

    # AK45 feedback 0x29
    fb = parse_ak45_feedback(0x2902, True, bytes.fromhex("FF9E000A00141A00"))
    check("feedback id", fb.controller_id, 2)
    check("feedback position", round(fb.position_deg, 3), -9.8)
    check("feedback speed", fb.speed_erpm, 100.0)
    check("feedback current", round(fb.current_a, 3), 0.2)
    check("feedback temperature", fb.temperature_c, 26)
    check("feedback, other function id", parse_ak45_feedback(0x0402, True, bytes(8)), None)
    check("feedback, standard frame", parse_ak45_feedback(0x2902, False, bytes(8)), None)

    # calibration.json: every motor once, CAN_STM ids 0..9, AK45 1..4
    cal = calibration.load()
    check("servo ids", sorted(j.id for j in cal.joints if j.motor == "servo"), list(range(10)))
    check("AK45 ids", sorted(j.id for j in cal.joints if j.motor == "ak45"), [1, 2, 3, 4])
    xml = Path(a.ik) / "motion2/arm/arm.xml"
    if xml.exists():
        sim = {}
        for jt in ET.parse(xml).getroot().iter("joint"):
            if jt.get("range"):
                sim[jt.get("name")] = [round(degrees(float(v)), 1) for v in jt.get("range").split()]
        mine = {j.joint: j.sim_range_deg for j in cal.joints if j.joint in sim}
        check("sim ranges match arm.xml", mine, {k: sim[k] for k in mine})
        check("every sim joint has a motor", sorted(sim), sorted(mine))
    else:
        print(f"SKIP  sim ranges: {xml} not found")

    # mapping
    j = cal["wrist_pitch"]
    check("servo sim 0 -> zero_deg", j.to_motor(0), (90.0, False))
    check("servo sim 120 clipped to the sim range", j.to_motor(120), (180.0, True))
    e = cal["elbow"]
    check("AK45 sim 30 clipped to zero +- limit", e.to_motor(30), (15.0, True))
    check("AK45 sim -10", e.to_motor(-10), (-10.0, False))
    check("ramp ends on target", ramp(0, 10, 30, 50)[-1], 10)
    check("ramp step <= rate / 50 Hz", max(abs(b - a) for a, b in zip([0] + ramp(0, 10, 30, 50), ramp(0, 10, 30, 50))) <= 0.6 + 1e-9, True)

    # Mover (sim2real_node): steps of rate / hz, only after start()
    mv = Mover(30, 50)
    mv.set_target("elbow", 1.0)
    check("mover waits for a start position", mv.step(), {})
    mv.start("elbow", 0.0)
    check("mover first step", round(mv.step()["elbow"], 6), 0.6)
    check("mover ends on target", mv.step(), {"elbow": 1.0})
    check("mover idle on target", mv.step(), {})

    # dry-run session: AK45 to /ak45/command in rad, servos as CAN_STM frames; bounds hold
    out = DryOutput(out=lambda s: None)
    s = Session(cal, out, 30, out=lambda s: None, sleep=lambda t: None)
    s.handle("j W_PITCH"); s.handle("m 100"); s.handle("m +10")
    check("session servo first move jumps", out.sent[0], "can0 005#0903E8")
    check("session servo ends at 110", out.sent[-1], "can0 005#09044C")
    n = len(out.sent)
    s.handle("m 181")
    check("session refuses a servo angle over 180", len(out.sent), n)
    s.handle("j elbow"); s.handle("m 20")
    check("session refuses an AK45 angle past the limit", len(out.sent), n)
    s.handle("m -10")
    check("session AK45 goes to /ak45/command in rad", out.sent[-1], "/ak45/command ak45_4 -0.17453 rad (-10 deg)")

    # trajectory playback: slowed so the fastest joint keeps to the speed limit
    from sim2real.play import schedule, play
    traj = {"joint_names": ["elbow", "wrist_pitch"], "dt": 0.02,
            "positions": [[0.0, 0.0], [radians(3.0), 0.0], [radians(3.0), radians(-1.0)]]}
    period, slow, peak = schedule(traj, 30.0)
    check("play: peak joint speed 3 deg in 20 ms", round(peak, 6), 150.0)
    check("play: 5x slower at 30 deg/s", (round(slow, 6), round(period, 6)), (5.0, 0.1))
    sent = []
    play(lambda n, q: sent.append(list(q)), traj, 30.0, hold=0.3, sleep=lambda t: None)
    check("play: first frame held 3 times, then the rest", len(sent), 3 + 2)
    check("play: ends on the last frame", sent[-1], traj["positions"][-1])
    from sim2real.play import clipped_joints
    far = {"joint_names": ["elbow", "wrist_pitch"], "dt": 0.02,
           "positions": [[radians(80.0), radians(10.0)], [radians(-5.0), radians(10.0)]]}
    check("play: joints past the calibration are named", [(n, round(lo), round(hi), al) for n, lo, hi, al in
          clipped_joints(far, cal)], [("elbow", -5, 80, [-15.0, 15.0])])

    # w writes a file that loads back to the same joints
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        cal.save(Path(d) / "c.json")
        check("save and load round trip", calibration.load(Path(d) / "c.json").joints, cal.joints)

    print(f"\n{'all passed' if not failures else f'{len(failures)} failed: ' + ', '.join(failures)}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()

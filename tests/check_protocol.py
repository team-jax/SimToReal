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
from math import degrees
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim2real.protocol import servo_frame, ak45_pos_frame, parse_ak45_feedback
from sim2real import calibration
from sim2real.bus import DryBus, ramp
from calibrate import Session

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
    check("ramp ends on target", ramp(0, 10, 30)[-1], 10)
    check("ramp step <= rate / 50 Hz", max(abs(b - a) for a, b in zip([0] + ramp(0, 10, 30), ramp(0, 10, 30))) <= 0.6 + 1e-9, True)

    # dry-run session: moves never leave the bounds
    bus = DryBus(out=lambda s: None)
    s = Session(cal, bus, 30, out=lambda s: None, sleep=lambda t: None)
    s.handle("j W_PITCH"); s.handle("m 100"); s.handle("m +10")
    check("session servo first move jumps, then ramps", [str(f) for f in bus.sent[:1]], ["005#0903E8"])
    check("session servo ends at 110", str(bus.sent[-1]), "005#09044C")
    n = len(bus.sent)
    s.handle("m 181")
    check("session refuses a servo angle over 180", len(bus.sent), n)
    s.handle("j elbow"); s.handle("m 20")
    check("session refuses an AK45 angle past the limit", len(bus.sent), n)

    # w writes a file that loads back to the same joints
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        cal.save(Path(d) / "c.json")
        check("save and load round trip", calibration.load(Path(d) / "c.json").joints, cal.joints)

    print(f"\n{'all passed' if not failures else f'{len(failures)} failed: ' + ', '.join(failures)}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()

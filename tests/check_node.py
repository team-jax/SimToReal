"""sim2real_node end to end, with no hardware: a python-can virtual bus stands in for can0 and this
script plays ak45_node (publishes /joint_states, listens to /ak45/command). Needs ROS 2 (Jazzy) and
python3-can; tests/docker_check.sh runs it in the ros:jazzy image.

    python3 tests/check_node.py
"""
import sys
import time
from math import radians

import can
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState

from sim2real.node import Sim2RealNode

failures = []


def check(name, got, want):
    ok = got == want
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {got}" + ("" if ok else f" (want {want})"))
    if not ok:
        failures.append(name)


def main():
    rclpy.init(args=["--ros-args", "-p", "servo_interface:=virtual", "-p", "servo_channel:=t",
                     "-p", "max_speed_deg_s:=300.0"])
    node = Sim2RealNode()
    fake = rclpy.create_node("fake_ak45_node")
    states = fake.create_publisher(JointState, "/joint_states", 10)
    commands = fake.create_publisher(JointState, "/sim2real/joint_command", 10)
    got = []
    fake.create_subscription(JointState, "/ak45/command", lambda m: got.append(m), 10)
    servo_rx = can.Bus(interface="virtual", channel="t")
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(fake)

    feedback_deg = {"ak45_1": 0.0, "ak45_2": 0.0, "ak45_3": 0.0, "ak45_4": 2.0}

    def run(seconds):
        frames = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            st = JointState()
            st.name = list(feedback_deg)
            st.position = [radians(v) for v in feedback_deg.values()]
            states.publish(st)
            ex.spin_once(timeout_sec=0.01)
            while (m := servo_rx.recv(timeout=0)) is not None:
                frames.append(f"{m.arbitration_id:03X}#{bytes(m.data).hex().upper()}")
        return frames

    def send(names, degs):
        msg = JointState()
        msg.name = names
        msg.position = [radians(d) for d in degs]
        commands.publish(msg)

    frames = run(1.0)
    check("nothing on /ak45/command before the first command", len(got), 0)
    check("no servo frame before the first command", frames, [])

    send(["elbow", "nonexistent"], [-10.0, 0.0])
    frames = run(0.5)
    check("a message with an unknown name is dropped whole", (len(got), frames), (0, []))

    send(["elbow", "wrist_pitch"], [-10.0, 20.0])
    frames = run(1.5)
    elbow = [m.position[list(m.name).index("ak45_4")] for m in got if "ak45_4" in m.name]
    check("AK45 commands use ak45_N names", sorted({n for m in got for n in m.name}), ["ak45_4"])
    check("elbow starts from its feedback (2 deg) and steps 6 deg at 300 deg/s, 50 Hz",
          [round(v, 4) for v in elbow[:2]], [round(radians(-4.0), 4), round(radians(-10.0), 4)])
    check("elbow ends at -10 deg in rad", round(elbow[-1], 6), round(radians(-10.0), 6))
    check("servo: first frame jumps to 90 + 20 deg", frames[:1], ["005#09044C"])
    check("servo: no more frames once there", len(frames), 1)

    send(["elbow"], [30.0])
    run(1.0)
    check("elbow 30 deg is clipped to zero + limit 15", round(got[-1].position[0], 6), round(radians(15.0), 6))

    node.destroy_node()
    fake.destroy_node()
    servo_rx.shutdown()
    rclpy.shutdown()
    print(f"\n{'all passed' if not failures else f'{len(failures)} failed: ' + ', '.join(failures)}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()

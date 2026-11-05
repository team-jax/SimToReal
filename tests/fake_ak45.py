"""Stands in for ak45_node in tests: /ak45/command echoed as /joint_states feedback at 50 Hz.

    python3 tests/fake_ak45.py [--freeze ak45_4]

Every motor starts at 0 deg (power-on) and then reads whatever it was last told, at once; a frozen
one stays at 0 whatever it is told, like a motor that hit something.
"""
import argparse

import rclpy
from sensor_msgs.msg import JointState


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--freeze", default=None)
    a, _ = ap.parse_known_args()
    rclpy.init()
    node = rclpy.create_node("fake_ak45_node")
    position = {f"ak45_{i}": 0.0 for i in range(1, 5)}

    def on_command(msg):
        for n, p in zip(msg.name, msg.position):
            if n in position and n != a.freeze:
                position[n] = p

    node.create_subscription(JointState, "/ak45/command", on_command, 10)
    pub = node.create_publisher(JointState, "/joint_states", 10)

    def tick():
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name, msg.position = list(position), list(position.values())
        pub.publish(msg)

    node.create_timer(0.02, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

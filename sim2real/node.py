"""sim2real_node: sim joint angles in, real motor commands out.

    /sim2real/joint_command  sensor_msgs/JointState, sim joint names (elbow, wrist_pitch, ...), rad
      -> calibration.json -> per-joint speed limit (max_speed_deg_s of the motor command, rate_hz steps)
      -> AK45 joints: /ak45/command (ak45_node sends and resends them)
      -> servo joints: CAN_STM frames on servo_channel

Rules, like ak45_node: nothing is sent before the first command; a message with a length mismatch, an
unknown name or a non-finite value is dropped whole; joints not in a message keep their target.
An AK45 starts from its /joint_states feedback and does not move without it. A servo's first command
jumps (CAN_STM sends no pulse before its first command, so its position is unknown).
"""
from math import degrees, isfinite

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from . import calibration
from .motion import Mover
from .protocol import servo_frame
from .ros_io import Ak45Topics, ServoCan

COMMAND_TOPIC = "/sim2real/joint_command"


class Sim2RealNode(Node):
    def __init__(self):
        super().__init__("sim2real_node")
        cal_file = self.declare_parameter("calibration_file", "").value
        hz = self.declare_parameter("rate_hz", 50.0).value
        rate = self.declare_parameter("max_speed_deg_s", 30.0).value
        interface = self.declare_parameter("servo_interface", "socketcan").value
        channel = self.declare_parameter("servo_channel", "can0").value
        self.cal = calibration.load(cal_file or None)
        self.joints = {j.joint: j for j in self.cal.joints}
        self.servo = ServoCan(interface, channel)
        self.ak45 = Ak45Topics(self)
        self.mover = Mover(rate, hz)
        self.create_subscription(JointState, COMMAND_TOPIC, self.on_command, 10)
        self.create_timer(1.0 / hz, self.on_timer)
        unverified = [j.joint for j in self.cal.joints if not j.verified]
        self.get_logger().info(
            f"{self.cal.path}: {len(self.joints)} joints, {rate:g} deg/s at {hz:g} Hz, servos on "
            f"{interface} {channel}. Unverified calibration: {', '.join(unverified) or 'none'}")

    def on_command(self, msg):
        log = self.get_logger()
        if len(msg.name) != len(msg.position):
            log.warning(f"name and position lengths differ ({len(msg.name)}, {len(msg.position)}); "
                        "message ignored", throttle_duration_sec=5.0)
            return
        for n, p in zip(msg.name, msg.position):
            if n not in self.joints:
                log.warning(f"unknown joint {n}; message ignored", throttle_duration_sec=5.0)
                return
            if not isfinite(p):
                log.warning(f"non-finite target for {n}; message ignored", throttle_duration_sec=5.0)
                return
        for n, p in zip(msg.name, msg.position):
            m, clipped = self.joints[n].to_motor(degrees(p))
            if clipped:
                log.warning(f"{n}: {degrees(p):.1f} deg clipped to motor {m:.1f}",
                            throttle_duration_sec=5.0)
            self.mover.set_target(n, m)

    def on_timer(self):
        for n in self.mover.needs_start():
            j = self.joints[n]
            if j.motor == "servo":
                t = self.mover.target[n]
                self.mover.start(n, t)
                self.servo.send(servo_frame(j.id, t))
                continue
            fb = self.ak45.feedback_deg(j)
            if fb is None:
                self.get_logger().warning(f"no /joint_states for ak45_{j.id}; {n} waits",
                                          throttle_duration_sec=5.0)
                continue
            self.mover.start(n, fb)
        ak45 = []
        for n, deg in self.mover.step().items():
            j = self.joints[n]
            if j.motor == "ak45":
                ak45.append((j, deg))
            else:
                self.servo.send(servo_frame(j.id, deg))
        if ak45:
            self.ak45.publish(ak45)

    def destroy_node(self):
        self.servo.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Sim2RealNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

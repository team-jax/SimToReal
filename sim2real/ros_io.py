"""ROS 2 side. AK45s go through team-jax/ak45_ros2's ak45_node; servos go straight onto can0.

ak45_node (ak45_ros2 README, docs/ros2.md section 13):
  /ak45/command  sensor_msgs/JointState, names ak45_1..ak45_6 (CAN id 1..6), position in rad,
                 RELIABLE KeepLast(10). The node resends the latest target at 10 Hz, rejects a joint
                 whose |target| exceeds max_command_deg, and sends nothing before the first command.
  /joint_states  feedback of the motors that answer, rad.
The rad here is the AK45 position command in radians (motor_deg from calibration.json), not the sim
joint angle.

Servos (team-jax/CAN_STM) have no ROS node: we write their frames on the same SocketCAN interface.
SocketCAN lets several sockets share can0; the one-process lock (/tmp/ak45_ctrl.lock) belongs to the
AK45 C++ library only.
"""
import time
from math import degrees, radians

from sensor_msgs.msg import JointState

AK45_COMMAND = "/ak45/command"
AK45_STATES = "/joint_states"


def ak45_topic_name(joint):
    return f"ak45_{joint.id}"


class Ak45Topics:
    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(JointState, AK45_COMMAND, 10)
        self.state = {}
        node.create_subscription(JointState, AK45_STATES, self._on_state, 10)

    def _on_state(self, msg):
        now = time.monotonic()
        for n, p in zip(msg.name, msg.position):
            self.state[n] = (degrees(p), now)

    def feedback_deg(self, joint, max_age=0.5):
        got = self.state.get(ak45_topic_name(joint))
        if got is None or time.monotonic() - got[1] > max_age:
            return None
        return got[0]

    def publish(self, targets):
        """targets: [(Joint, motor_deg), ...]."""
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [ak45_topic_name(j) for j, _ in targets]
        msg.position = [radians(d) for _, d in targets]
        self.pub.publish(msg)


class ServoCan:
    """python-can bus for the servo frames. On SocketCAN the bitrate is set by `ip link`, not here."""

    def __init__(self, interface="socketcan", channel="can0"):
        import can
        self._can = can
        self.bus = can.Bus(interface=interface, channel=channel)

    def send(self, frame):
        self.bus.send(self._can.Message(arbitration_id=frame.arb_id, is_extended_id=frame.extended,
                                        data=frame.data))

    def close(self):
        self.bus.shutdown()

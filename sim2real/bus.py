"""Where frames go: printed (dry run, the default) or onto a CAN bus through python-can.

One bus carries both the AK45s and the STM32 servo node (CAN_STM), 1 Mbps.
"""
import time

from .protocol import parse_ak45_feedback

SEND_HZ = 50   # ramp step rate; the servo firmware updates every 20 ms anyway


class DryBus:
    """Prints each frame in cansend notation. No feedback."""

    def __init__(self, out=print):
        self.out = out
        self.sent = []

    def send(self, frame):
        self.sent.append(frame)
        self.out(f"  dry-run {frame}")

    def feedback(self, controller_id):
        return None

    def close(self):
        pass


class CanBus:
    """python-can bus. The interface name depends on the adapter (CANable candleLight: gs_usb)."""

    def __init__(self, interface, channel, bitrate=1_000_000):
        import can   # only needed when sending for real
        self._can = can
        self.bus = can.Bus(interface=interface, channel=channel, bitrate=bitrate)
        self.states = {}

    def send(self, frame):
        self.bus.send(self._can.Message(arbitration_id=frame.arb_id, is_extended_id=frame.extended,
                                        data=frame.data))
        self.poll()

    def poll(self):
        while (msg := self.bus.recv(timeout=0)) is not None:
            s = parse_ak45_feedback(msg.arbitration_id, msg.is_extended_id, msg.data)
            if s is not None:
                self.states[s.controller_id] = (s, time.monotonic())

    def feedback(self, controller_id, max_age=0.2):
        """Latest AK45 state if it is newer than max_age seconds (the motor uploads it periodically)."""
        self.poll()
        got = self.states.get(controller_id)
        if got is None or time.monotonic() - got[1] > max_age:
            return None
        return got[0]

    def close(self):
        self.bus.shutdown()


def ramp(start, target, max_rate, hz=SEND_HZ):
    """Intermediate positions from start to target at most max_rate deg/s, ending exactly on target."""
    step = max_rate / hz
    n = max(1, int(abs(target - start) / step + 0.999999))
    return [start + (target - start) * (i + 1) / n for i in range(n)]

"""What the robot can tell during a playback, with no hand or cube sensor yet: whether the AK45s follow.

An AK45 that stops following its command has hit something, or is not answering. The playback then
stops and the arm is told to stay where it is. A finger or cube sensor (FSR, camera) would add its
own check here.
"""


class TrackingWatch:
    """check() names the first AK45 that has been more than max_error_deg from what it was told, or
    silent, for longer than grace_s; None while every one follows."""

    def __init__(self, max_error_deg=15.0, grace_s=0.5):
        self.max_error, self.grace = max_error_deg, grace_s
        self._since = {}

    def check(self, expected, feedback, now):
        """expected, feedback: {motor name: deg} (feedback None when nothing came)."""
        for name, want in expected.items():
            got = feedback.get(name)
            if got is not None and abs(want - got) <= self.max_error:
                self._since.pop(name, None)
                continue
            start = self._since.setdefault(name, now)
            if now - start > self.grace:
                what = "no feedback" if got is None else f"{abs(want - got):.1f} deg from its command"
                return f"{name}: {what} for {now - start:.1f} s"
        return None

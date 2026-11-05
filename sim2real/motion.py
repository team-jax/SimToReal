"""Speed limits for motor commands (degrees of the motor command)."""


def ramp(start, target, max_rate, hz):
    """Intermediate positions from start to target at most max_rate deg/s, ending exactly on target."""
    step = max_rate / hz
    n = max(1, int(abs(target - start) / step + 0.999999))
    return [start + (target - start) * (i + 1) / n for i in range(n)]


class Mover:
    """Per-joint command that walks toward its target by at most max_rate / hz each step.

    A joint moves only after start() gave its present position (feedback, or the first command).
    """

    def __init__(self, max_rate, hz):
        self.step_deg = max_rate / hz
        self.current = {}
        self.target = {}

    def set_target(self, name, deg):
        self.target[name] = deg

    def start(self, name, deg):
        self.current[name] = deg

    def needs_start(self):
        return [n for n in self.target if n not in self.current]

    def step(self):
        """{name: new command} for the joints that moved this step."""
        moved = {}
        for n, t in self.target.items():
            c = self.current.get(n)
            if c is None or c == t:
                continue
            d = t - c
            c = t if abs(d) <= self.step_deg else c + (self.step_deg if d > 0 else -self.step_deg)
            self.current[n] = moved[n] = c
        return moved

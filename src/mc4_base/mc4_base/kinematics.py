"""Differential-drive maths, kept free of ROS so it can be unit-tested."""
import math


def wheel_speeds_from_twist(v, w, wheel_radius, wheel_separation):
    """Body velocity (m/s, rad/s) -> wheel angular speeds (rad/s) as (left, right)."""
    left = (v - w * wheel_separation / 2.0) / wheel_radius
    right = (v + w * wheel_separation / 2.0) / wheel_radius
    return left, right


def scale_to_limit(left, right, max_speed):
    """Scale both wheels equally so neither exceeds max_speed; keeps the turn radius."""
    peak = max(abs(left), abs(right))
    if max_speed > 0.0 and peak > max_speed:
        s = max_speed / peak
        return left * s, right * s
    return left, right


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class DiffDriveOdometry:
    """Integrates cumulative encoder ticks into a 2D pose."""

    def __init__(self, wheel_radius, wheel_separation, ticks_per_rev):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.ticks_per_rev = ticks_per_rev
        self.reset()

    @property
    def metres_per_tick(self):
        return 2.0 * math.pi * self.wheel_radius / self.ticks_per_rev

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_angle = 0.0   # wheel joint angles (rad) for /joint_states
        self.right_angle = 0.0
        self._prev = None

    def update(self, left_ticks, right_ticks, dt):
        """Feed cumulative tick counts. Returns (v, w) body velocity."""
        if self._prev is None:
            self._prev = (left_ticks, right_ticks)
            return 0.0, 0.0

        d_lt = left_ticks - self._prev[0]
        d_rt = right_ticks - self._prev[1]
        self._prev = (left_ticks, right_ticks)

        dl = d_lt * self.metres_per_tick
        dr = d_rt * self.metres_per_tick
        ds = (dl + dr) / 2.0
        dth = (dr - dl) / self.wheel_separation

        # Midpoint integration: more accurate than Euler on curves.
        self.x += ds * math.cos(self.theta + dth / 2.0)
        self.y += ds * math.sin(self.theta + dth / 2.0)
        self.theta = normalize_angle(self.theta + dth)

        rad_per_tick = 2.0 * math.pi / self.ticks_per_rev
        self.left_angle = normalize_angle(self.left_angle + d_lt * rad_per_tick)
        self.right_angle = normalize_angle(self.right_angle + d_rt * rad_per_tick)

        if dt <= 0.0:
            return 0.0, 0.0
        return ds / dt, dth / dt

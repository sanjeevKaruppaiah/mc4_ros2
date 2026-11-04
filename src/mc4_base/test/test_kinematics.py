import math

import numpy as np

from mc4_base.kinematics import DiffDriveOdometry, scale_to_limit, wheel_speeds_from_twist
from mc4_base.world import box, raycast

R, SEP, TPR = 0.033, 0.16, 1320.0


def test_straight_line_wheel_speeds():
    left, right = wheel_speeds_from_twist(0.2, 0.0, R, SEP)
    assert math.isclose(left, right)
    assert math.isclose(left, 0.2 / R)


def test_spin_in_place_is_antisymmetric():
    left, right = wheel_speeds_from_twist(0.0, 1.0, R, SEP)
    assert math.isclose(left, -right)
    assert right > 0.0


def test_scale_keeps_ratio():
    left, right = scale_to_limit(20.0, 10.0, 10.0)
    assert math.isclose(left, 10.0)
    assert math.isclose(right, 5.0)


def test_odometry_one_wheel_revolution_forward():
    odom = DiffDriveOdometry(R, SEP, TPR)
    odom.update(0, 0, 0.1)
    v, w = odom.update(TPR, TPR, 1.0)
    assert math.isclose(odom.x, 2 * math.pi * R, rel_tol=1e-9)
    assert abs(odom.y) < 1e-12
    assert math.isclose(v, 2 * math.pi * R)
    assert abs(w) < 1e-12


def test_odometry_quarter_turn_in_place():
    odom = DiffDriveOdometry(R, SEP, TPR)
    odom.update(0, 0, 0.1)
    # each wheel travels (pi/2) * SEP/2 in opposite directions
    arc = (math.pi / 2) * SEP / 2
    ticks = arc / odom.metres_per_tick
    odom.update(-ticks, ticks, 1.0)
    assert math.isclose(odom.theta, math.pi / 2, rel_tol=1e-9)
    assert abs(odom.x) < 1e-9 and abs(odom.y) < 1e-9


def test_raycast_inside_square():
    walls = np.array(box(0.0, 0.0, 4.0, 4.0))
    r = raycast(walls, 0.0, 0.0, np.array([0.0, math.pi / 2, math.pi, math.pi / 4]), 10.0)
    assert np.allclose(r[:3], 2.0)
    assert math.isclose(r[3], 2.0 * math.sqrt(2.0), rel_tol=1e-9)


def test_raycast_out_of_range_is_inf():
    walls = np.array(box(0.0, 0.0, 4.0, 4.0))
    r = raycast(walls, 0.0, 0.0, np.array([0.0]), 1.0)
    assert np.isinf(r[0])

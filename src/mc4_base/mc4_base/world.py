"""A tiny 2D world (line segments) and a vectorised lidar ray-caster.

Used by fake_lidar_node so students can run SLAM + Nav2 with no robot.
Coordinates are in the odom frame; the robot starts at (0, 0) facing +x.
"""
import numpy as np


def box(cx, cy, sx, sy):
    """Four segments of an axis-aligned rectangle centred at (cx, cy)."""
    x0, x1 = cx - sx / 2.0, cx + sx / 2.0
    y0, y1 = cy - sy / 2.0, cy + sy / 2.0
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def default_world():
    """A 6 m x 4 m room with an inner wall and a few boxes (like a competition mat arena)."""
    segs = []
    segs += box(1.5, 0.0, 6.0, 4.0)                 # outer walls: x -1.5..4.5, y -2..2
    segs += [(3.0, -2.0, 3.0, 0.6)]                 # inner wall, leaves a gap at the top
    segs += box(1.6, 1.1, 0.5, 0.5)                 # obstacle boxes
    segs += box(1.0, -1.2, 0.6, 0.3)
    segs += box(3.9, -1.0, 0.4, 0.8)
    segs += box(4.0, 1.3, 0.3, 0.3)
    return np.array(segs, dtype=float)


def raycast(segments, ox, oy, angles, max_range):
    """Distance from (ox, oy) along each world-frame angle to the nearest segment.

    segments: (M, 4) array of x1, y1, x2, y2.  angles: (N,) radians.
    Returns (N,) ranges; inf where nothing is hit within max_range.
    """
    dx = np.cos(angles)[:, None]              # (N, 1) ray direction
    dy = np.sin(angles)[:, None]
    px, py = segments[:, 0][None, :], segments[:, 1][None, :]   # (1, M)
    sx = (segments[:, 2] - segments[:, 0])[None, :]
    sy = (segments[:, 3] - segments[:, 1])[None, :]

    denom = dx * sy - dy * sx                  # cross(d, s)
    qx, qy = px - ox, py - oy                  # p - o
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (qx * sy - qy * sx) / denom        # distance along ray
        u = (qx * dy - qy * dx) / denom        # position along segment [0, 1]
    hit = (np.abs(denom) > 1e-12) & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
    t = np.where(hit, t, np.inf)
    r = t.min(axis=1)
    r[r > max_range] = np.inf
    return r

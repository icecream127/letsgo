"""Quaternion and velocity helpers, ROS xyzw convention."""
import math
import numpy as np


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return np.array([w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                     w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z])


def rotate(q, v):
    return multiply(multiply(q, [*v, 0.]), [-q[0], -q[1], -q[2], q[3]])[:3]


def yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def yaw_q(a):
    return np.array([0., 0., math.sin(a/2), math.cos(a/2)])


def planar_velocity(samples):
    """Least squares over measured positions; unwrap heading, return body-frame twist."""
    data = np.asarray(samples, dtype=float)
    t = data[:, 0] - data[0, 0]
    if len(t) < 3 or t[-1] < .08:
        return (0., 0., 0.)
    t -= t.mean()
    values = np.column_stack([data[:, 1:3], np.unwrap(data[:, 3])])
    vx, vy, wz = (t[:, None] * values).sum(axis=0) / (t*t).sum()
    heading = data[-1, 3]
    return (math.cos(heading)*vx+math.sin(heading)*vy,
            -math.sin(heading)*vx+math.cos(heading)*vy, wz)

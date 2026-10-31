import math
import unittest
import numpy as np
from ros2_to_stm32.nav_geometry import planar_velocity, rotate, yaw_q


class GeometryTest(unittest.TestCase):
    def test_velocity_in_body_axes(self):
        # Moving north while facing north is forward, not lateral velocity.
        samples = [(t, 0., .2*t, math.pi/2) for t in np.linspace(0., .2, 6)]
        np.testing.assert_allclose(planar_velocity(samples), [.2, 0., 0.], atol=1e-10)

    def test_heading_wrap(self):
        samples = [(t, 0., 0., (3.13+.4*t+math.pi)%(2*math.pi)-math.pi)
                   for t in np.linspace(0., .2, 6)]
        np.testing.assert_allclose(planar_velocity(samples), [0., 0., .4], atol=1e-10)

    def test_mount_offset_rotation(self):
        np.testing.assert_allclose(rotate(yaw_q(math.pi/2), [1., 0., 0.]), [0., 1., 0.], atol=1e-10)


if __name__ == '__main__': unittest.main()

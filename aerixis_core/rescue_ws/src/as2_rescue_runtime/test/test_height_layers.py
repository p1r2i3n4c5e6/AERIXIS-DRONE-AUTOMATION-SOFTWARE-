"""Height layers (2026-11-01): drones whose heights differ clearly are separated even when they pass
close horizontally (search drones at 15 m, delivery drones at 20 m)."""
import unittest

from as2_rescue_runtime.swarm import VERTICAL_SEPARATION_M, separation_safe


def frame(x, z, vz=0.0, owner='drone0', **extra):
    out = dict(owner=owner, position=[x, 0.0, z], frame_id='earth', stamp=100.0, sigma_m=1.5, speed=2.0,
               velocity=[2.0, 0.0], vz=vz)
    out.update(extra)
    return out


class HeightLayerTests(unittest.TestCase):
    def test_same_height_close_is_not_safe(self):
        self.assertFalse(separation_safe(frame(0, 15), [frame(3, 15, owner='drone3')], 100.5, 3.0, 1.0))

    def test_five_metres_above_is_safe_in_level_flight(self):
        self.assertTrue(separation_safe(frame(0, 15), [frame(3, 20, owner='drone3')], 100.5, 3.0, 1.0))

    def test_a_small_gap_or_climbing_or_unknown_vertical_speed_is_not_enough(self):
        self.assertFalse(separation_safe(frame(0, 15), [frame(3, 15 + VERTICAL_SEPARATION_M - 1, owner='d')], 100.5, 3.0, 1.0))
        self.assertFalse(separation_safe(frame(0, 15), [frame(3, 20, vz=-2.0, owner='d')], 100.5, 3.0, 1.0))
        peer = frame(3, 20, owner='d'); del peer['vz']                    # older software: no vz
        self.assertFalse(separation_safe(frame(0, 15), [peer], 100.5, 3.0, 1.0))

    def test_far_apart_is_safe_as_before(self):
        self.assertTrue(separation_safe(frame(0, 15), [frame(40, 15, owner='d')], 100.5, 3.0, 1.0))


if __name__ == '__main__':
    unittest.main()

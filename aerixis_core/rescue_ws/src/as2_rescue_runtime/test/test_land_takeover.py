"""Recovery landing after a take-off that failed on the ground (SITL 2026-11-01): the cancelled
take-off did not hand motion back within MOTION_WAIT_S on a loaded computer, ManagedLand was
refused ("motion ownership unavailable") and the drone stayed ARMED on the ground. Landing now
takes over after the wait; other operations still refuse."""
import unittest

from as2_rescue_runtime.engine import MOTION_WAIT_S
import test_mode_change as base   # module import: pytest must not collect its tests twice


class LandTakeoverTests(unittest.TestCase):
    setUp, tearDown, step, command, moves = (base.ModeChangeTests.setUp, base.ModeChangeTests.tearDown,
                                             base.ModeChangeTests.step, base.ModeChangeTests.command,
                                             base.ModeChangeTests.moves)

    def stuck_motion(self):
        """The driver still holds the cancelled take-off goal (never idle)."""
        self.d.idle = False
        self.d.abandoned = 0
        def abandon():
            self.d.abandoned += 1
            self.d.idle = True
        self.d.abandon = abandon

    def test_landing_takes_over_from_a_motion_that_never_hands_back(self):
        self.stuck_motion()
        self.step()
        self.command('ManagedLand', speed=0.5)
        self.assertIsNone(self.engine.tick())                    # waits for the hand-back first
        self.assertEqual(self.d.abandoned, 0)
        t = 0.0
        while t < MOTION_WAIT_S + 0.5:
            self.step(0.5); t += 0.5
            result = self.engine.tick()
            self.assertIsNone(result)                             # never fails on ownership
        self.assertEqual(self.d.abandoned, 1)
        self.assertTrue(any(m[0] == 'vertical' and m[1][0] == 'ManagedLand' for m in self.moves()))

    def test_other_operations_still_refuse_a_busy_motion(self):
        self.stuck_motion()
        self.step()
        self.command('ManagedTakeoff', height=10.0, speed=0.5)
        result = None
        t = 0.0
        while result is None and t < MOTION_WAIT_S + 5:
            self.step(0.5); t += 0.5
            result = self.engine.tick()
        self.assertEqual(self.d.abandoned, 0)
        self.assertEqual(result['status'], 'FAILURE')


if __name__ == '__main__':
    unittest.main()

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

    def test_return_waits_out_a_short_data_gap(self):
        # SITL 2026-11-01: ReturnToLaunch failed on its first tick during a 1 s data gap (fresh
        # again 0.3 s later) and the drone was left hovering at 7 m.
        self.step()
        self.engine.home = dict(position=[30.0, 0.0, 0.0])
        self.clock.value += 1.5                                  # vehicle evidence now stale
        self.command('ReturnToLaunch', speed=2.0)
        self.assertIsNone(self.engine.tick())
        self.assertIn('fresh flight-controller data', self.engine.waiting)
        self.clock.value += 1.0
        self.assertIsNone(self.engine.tick())
        self.step()                                              # data back
        self.assertIsNone(self.engine.tick())
        self.assertTrue(self.moves())                            # flying home

    def test_return_still_fails_when_the_data_does_not_come_back(self):
        from as2_rescue_runtime.engine import RECOVERY_DATA_WAIT_S
        self.step()
        self.engine.home = dict(position=[30.0, 0.0, 0.0])
        self.clock.value += 1.5
        self.command('ReturnToLaunch', speed=2.0)
        result = None
        for _ in range(int(RECOVERY_DATA_WAIT_S / 0.5) + 2):
            result = self.engine.tick()
            if result:
                break
            self.clock.value += 0.5
        self.assertEqual(result['status'], 'FAILURE')
        self.assertIn('Fresh valid aircraft position', result['reason'])


if __name__ == '__main__':
    unittest.main()

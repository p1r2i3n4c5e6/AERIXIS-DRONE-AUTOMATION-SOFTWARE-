"""ResumeCoverage flies back to the saved resume point and keeps that goal for the whole leg.

PX4 v1.16 SITL, 2026-10-24: the leg's start re-saved the checkpoint from the aircraft's current
position, so the next tick asked the route publisher for a different goal ("Approved leg goal
changed") and coverage never resumed after a geotag. In Gazebo the launch point was 41 m outside
the area: a person seen on the entry transit gave a resume point outside the area, which the
route check refused; an interrupted entry transit now resumes at its end (the first lane)."""
import unittest
import test_mode_change as base   # module import: pytest must not collect its tests twice


class ResumeCoverageTests(unittest.TestCase):
    # Same fake aircraft and engine as the mode-change tests, without re-running their tests.
    setUp, tearDown, step, command = base.ModeChangeTests.setUp, base.ModeChangeTests.tearDown, base.ModeChangeTests.step, base.ModeChangeTests.command
    moves, start_leg = base.ModeChangeTests.moves, base.ModeChangeTests.start_leg

    def record_routes(self):
        self.routes = []
        approve = self.d.request_route
        def request_route(command, pose, frame):
            self.routes.append(list(pose))
            approve(command, pose, frame)
        self.d.request_route = request_route

    def at(self, position):
        self.step()
        vehicle = self.e.get('vehicle')
        self.clock.value += 0.01                 # evidence ignores a second sample at the same instant
        self.e.put('vehicle', dict(vehicle, position=list(position)))

    def checkpoint_at(self, position):
        self.at(position)
        self.command('CheckpointCoverage')                 # as the BT does before leaving the lane
        self.assertEqual(self.engine.tick()['status'], 'SUCCESS')
        return [round(v, 3) for v in self.engine.journal.state['missions']['mission']['pose']]

    def finish_first_lane(self):
        """Entry transit to 40,0 flown to its end: later legs are survey lanes."""
        self.start_leg()
        self.d.result = 'SUCCESS'
        self.at([40, 0, 10])
        self.assertEqual(self.engine.tick()['status'], 'SUCCESS')
        self.d.idle = True; self.d.result = None
        self.command('SurveyWaypoint', segment_id='lane-2', pose='40;30;10', speed=1.0)
        self.assertIsNone(self.engine.tick())
        self.d.result = 'RUNNING'; self.d.idle = False

    def test_interrupted_entry_transit_resumes_at_the_first_lane(self):
        self.start_leg()                                   # launch 0,0 -> first lane start 40,0
        self.assertEqual(self.checkpoint_at([12, 3, 10]), [40.0, 0.0, 10.0])

    def test_resume_goal_stays_the_saved_point(self):
        self.finish_first_lane()                           # survey lane 40,0 -> 40,30
        self.assertEqual(self.checkpoint_at([40, 10, 10]), [40.0, 10.0, 10.0])
        # The aircraft went to a person off the lane, then resumes coverage.
        self.d.idle = True; self.d.result = None; self.d.calls.clear(); self.record_routes()
        self.at([46, 14, 10])
        self.command('ResumeCoverage', speed=1.0)
        self.assertIsNone(self.engine.tick())
        self.assertEqual(self.moves(), [('move', [40.0, 10.0, 10.0])])
        self.d.idle = False; self.d.result = 'RUNNING'
        for position in ([44, 13, 10], [42, 12, 10], [41, 11, 10]):
            self.at(position)
            self.assertIsNone(self.engine.tick())
        self.assertEqual({tuple(round(v, 3) for v in p) for p in self.routes}, {(40.0, 10.0, 10.0)})
        self.d.result = 'SUCCESS'
        self.at([40, 10, 10])
        self.assertEqual(self.engine.tick()['status'], 'SUCCESS')


if __name__ == '__main__':
    unittest.main()

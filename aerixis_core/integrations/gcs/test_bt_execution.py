import json
import unittest
from bt_execution import EventState, LocalTreeRun, PREFIX


class EventTests(unittest.TestCase):
    def setUp(self):
        self.state = EventState('mission')
        self.seq = 0

    def event(self, kind, **values):
        self.seq += 1
        return PREFIX + json.dumps(dict(schema_version=1, mission_id='mission',
                                       run_id='attempt', sequence=self.seq,
                                       event=kind, **values))

    def started(self):
        return self.event('started', nodes=[dict(node_uid=i, node_type='GoTo',
                          node_name='GoTo', node_path=f'GoTo::{i}') for i in (1, 2)])

    def test_missing_result_is_error_even_on_zero_exit(self):
        self.state.finish_process(0)
        self.assertEqual(self.state.outcome, 'ERROR')

    def test_failure_and_unticked_nodes_are_not_success(self):
        self.assertTrue(self.state.accept(self.started()))
        self.state.accept(self.event('node_status', node_uid=1, node_path='GoTo::1', status='FAILURE'))
        self.state.accept(self.event('finished', outcome='FAILURE', exit_code=1))
        self.state.finish_process(1)
        self.assertEqual(self.state.outcome, 'FAILURE')
        self.assertEqual([s['status'] for s in self.state.steps()], ['failed', 'pending'])

    def test_repeated_types_and_idle_reset_keep_correct_results(self):
        self.state.accept(self.started())
        for status in ('SUCCESS', 'IDLE'):
            self.state.accept(self.event('node_status', node_uid=1, node_path='GoTo::1', status=status))
        self.assertEqual([s['status'] for s in self.state.steps()], ['success', 'pending'])

    def test_stale_wrong_mission_and_malformed_events_are_ignored(self):
        start = self.started()
        self.assertTrue(self.state.accept(start))
        for line in [start, 'SUCCESS', PREFIX+'{}', PREFIX+'[]', PREFIX+'broken',
                     self.event('finished', outcome='SUCCESS', exit_code=0).replace('mission"', 'other"')]:
            self.assertFalse(self.state.accept(line))
        self.assertIsNone(self.state.outcome)

    def test_conflicting_exit_is_error(self):
        self.state.accept(self.event('finished', outcome='SUCCESS', exit_code=0))
        self.state.finish_process(1)
        self.assertEqual(self.state.outcome, 'ERROR')

    def test_cancel_is_distinct_from_success(self):
        self.state.accept(self.started())
        self.state.accept(self.event('node_status', node_uid=1, node_path='GoTo::1', status='RUNNING'))
        self.state.accept(self.event('finished', outcome='CANCELLED', exit_code=2))
        self.state.finish_process(2)
        self.assertEqual(self.state.outcome, 'CANCELLED')
        self.assertEqual(self.state.steps()[0]['status'], 'cancelled')

    def test_namespace_injection_rejected_before_creating_files(self):
        for name in ['drone0; touch /tmp/bad', '../drone0', 'drone0/x', '']:
            with self.assertRaises(ValueError):
                LocalTreeRun(name, '<root><BehaviorTree ID="t"><AlwaysSuccess/></BehaviorTree></root>')


if __name__ == '__main__':
    unittest.main()

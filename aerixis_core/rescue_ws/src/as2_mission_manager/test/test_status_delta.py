import unittest

from as2_mission_manager.status_delta import bt_key, compact


def state(logs=3, status='IDLE'):
    nodes = [dict(node_uid=1, node_type='Sequence', node_name='Sequence', node_path='Sequence::1', children_uids=[2],
                  node_category='Control', status='RUNNING', display_status='running'),
             dict(node_uid=2, node_type='ScanArea', node_name='ScanArea', node_path='ScanArea::2', children_uids=[],
                  node_category='Action', status=status, display_status='pending')]
    return dict(drone_id='drone0', mission=dict(id='m', nodes=['x'] * 50, node_types={'a': 1}, sha256='s'),
                logs=[dict(id='%032d' % i, time=i, level='INFO', message='m%d' % i) for i in range(logs)],
                bt_nodes=nodes, registry_counts={}, ros=dict(nodes=['/drone0/a'], rescue=dict(
                    data=dict(flags=dict(VehicleHealthy=True, AbortRequested=False, reason=''), payload=dict(truth='UNKNOWN', stamp=1)))))


class StatusDeltaTests(unittest.TestCase):
    def test_first_poll_carries_structure(self):
        out = compact(state(), {'lite': '1'})
        self.assertTrue(out['lite'])
        self.assertEqual(len(out['bt_nodes']), 2)
        self.assertNotIn('nodes', out['mission'])
        self.assertEqual(out['ros']['rescue']['data']['flags'], {'VehicleHealthy': True})
        self.assertEqual(out['log_cursor'], '%032d' % 2)

    def test_steady_poll_sends_only_changes(self):
        first = compact(state(), {'lite': '1'})
        out = compact(state(logs=5, status='SUCCESS'),
                      {'lite': '1', 'log': first['log_cursor'], 'bt': first['bt_key'], 'ros': first['ros']['nodes_key']})
        self.assertNotIn('bt_nodes', out)
        self.assertEqual(out['bt_status'], 'RrSp')
        self.assertEqual([l['message'] for l in out['logs']], ['m3', 'm4'])
        self.assertNotIn('nodes', out['ros'])

    def test_new_tree_resends_structure(self):
        old = bt_key(state()['bt_nodes'])
        changed = state()
        changed['bt_nodes'][1]['node_type'] = 'OrbitTarget'
        self.assertIn('bt_nodes', compact(changed, {'lite': '1', 'bt': old}))


if __name__ == '__main__':
    unittest.main()

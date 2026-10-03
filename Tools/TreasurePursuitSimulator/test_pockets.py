import json
import unittest
from simulate import Game, ROOT, TrackMap


class PocketTests(unittest.TestCase):
    def setUp(self):
        self.game = Game(TrackMap(), json.loads((ROOT/'config.json').read_text(encoding='utf-8')))
        self.game.now = 30
        self.game.owner, self.game.police_checked_treasure = 'B', True
        self.actor, self.belief = self.game.robots['P'], self.game.beliefs['P']

    def test_upper_right_loop_has_one_gate_and_no_hiding_sites(self):
        g = self.game
        for node in ('(9,0)', '(11,0)', '(10,1)', '(11,1)', '(10,2)'):
            self.assertIn(node, g.pockets['(9,2)'])
            self.assertFalse(g.hide_site_safe(node))
            self.assertNotIn(node, g.hide_candidates)
        self.assertFalse(g.hide_site_safe('(9,2)'))

    def test_detected_thief_in_loop_causes_exit_block_without_hidden_truth(self):
        g = self.game
        self.actor.node, self.actor.facing = '(8,2)', 'E'
        self.belief.last_seen, self.belief.last_xy = '(10,2)', (10,2)
        self.belief.seen, self.belief.last_time = True, 30
        for hidden in ('(0,0)', '(11,5)'):
            g.robots['B'].node = hidden
            self.assertEqual(g.decide('P'), ('(9,2)', 'cutoff', '(9,2)'))
        self.actor.node = '(9,2)'
        self.assertEqual(g.decide('P'), (None, 'cutoff', '(9,2)'))
        g.now = 33
        self.belief.seen = False
        self.assertIsNone(g.block_pocket_exit(self.actor, self.belief))

    def test_block_not_used_when_too_late_or_new_detection_outside_loop(self):
        g = self.game
        self.actor.node, self.actor.facing = '(0,9)', 'N'
        self.belief.last_seen, self.belief.last_xy = '(10,2)', (10,2)
        self.belief.seen, self.belief.last_time = True, 30
        self.assertIsNone(g.block_pocket_exit(self.actor, self.belief))
        g.pocket_block = {'gate':'(9,2)', 'observed_at':30, 'arrived_at':None}
        self.belief.last_seen, self.belief.last_xy = '(8,2)', (8,2)
        self.assertIsNone(g.block_pocket_exit(self.actor, self.belief))
        self.assertIsNone(g.pocket_block)

    def test_past_last_sighting_continues_forward_to_straight_end(self):
        g = self.game
        self.actor.node, self.actor.facing, self.actor.last_node = '(6,5)', 'S', '(6,4)'
        self.belief.last_seen, self.belief.last_xy, self.belief.last_time = '(6,3)', (6,3), 29.9
        g.threat_active['P'] = True
        g.police_trail = {'node':'(6,3)', 'xy':(6,3), 'direction':'S', 'reached':False, 'last_time':29.9}
        self.assertEqual(g.decide('P'), ('(6,6)', 'trail', '(6,9)'))

    def test_resume_ahead_does_not_return_to_old_patrol_waypoint(self):
        g = self.game
        self.actor.node, self.actor.facing = '(6,5)', 'S'
        g.search_route = [f'(6,{y})' for y in range(1,10)]
        g.search_route_target = '(6,9)'
        self.assertEqual(g.decide('P'), ('(6,6)', 'search', '(6,9)'))
        self.assertNotIn('(6,1)', g.search_covered)
        self.assertEqual(g.search_sweeps[-1]['event'], 'patrol_rejoin')


if __name__ == '__main__':
    unittest.main()

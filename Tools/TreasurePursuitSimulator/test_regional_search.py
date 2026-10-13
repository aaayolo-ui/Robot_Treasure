"""Finite observation, local clue search and hide/relocate regressions."""
import json
import unittest
from simulate import Game, ROOT, TrackMap, relative_sector


class RegionalSearchTests(unittest.TestCase):
    def test_n052_does_not_reverse_east_toward_observed_police(self):
        g = Game(self.track, self.config, thief_policy='refuge')
        g.now, g.owner = 140.172, 'B'
        actor, belief = g.robots['B'], g.beliefs['B']
        actor.node, actor.facing = '(9,5)', 'W'
        belief.last_seen, belief.last_xy = '(10,5)', (9.97776,5)
        belief.last_time, belief.last_heading, belief.seen = 139.8, 'S', True
        result = g.destination_route(actor, belief, set(g.hide_candidates)-{actor.node}, parking=True)
        self.assertIsNotNone(result)
        self.assertEqual(result[0][1], '(9,6)')

    def test_cleared_n029_cancels_old_deadend_patrol(self):
        g = Game(self.track, self.config, thief_policy='refuge')
        actor = g.robots['P']
        actor.node, actor.facing = '(11,4)', 'N'
        g.search_route, g.search_route_target = ['(11,3)', '(11,2)'], '(11,2)'
        g.arm_clear_counts['(11,2)'] = 3
        self.assertIsNone(g.continue_search_route(actor))
        self.assertEqual(g.search_route, [])

    def test_uncleared_deadend_still_needs_inspection(self):
        g = Game(self.track, self.config, thief_policy='refuge')
        actor = g.robots['P']
        actor.node, actor.facing = '(11,4)', 'N'
        g.search_route, g.search_route_target = ['(11,3)', '(11,2)'], '(11,2)'
        g.arm_clear_counts['(11,2)'] = 2
        self.assertEqual(g.continue_search_route(actor)[0], '(11,3)')

    def test_cleared_leaf_is_not_immediately_selected_again(self):
        g = Game(self.track, self.config, thief_policy='refuge')
        actor = g.robots['P']
        actor.node, actor.facing = '(2,2)', 'S'
        g.search_covered = set(g.search_sites)-{'(3,3)'}
        g.arm_clear_counts['(2,3)'] = 3
        action = g.search(actor, g.beliefs['P'])
        self.assertNotEqual(action[2], '(2,3)')

    @classmethod
    def setUpClass(cls):
        cls.track = TrackMap()
        cls.config = json.loads((ROOT / 'config.json').read_text(encoding='utf-8'))

    def game(self):
        g = Game(self.track, dict(self.config, miss_probability=0), thief_policy='refuge')
        g.now, g.owner, g.police_checked_treasure = 40, 'B', True
        actor, belief = g.robots['P'], g.beliefs['P']
        actor.node, actor.facing = '(0,2)', 'W'  # N020, after passing N021.
        belief.last_seen, belief.last_xy = '(1,4)', (1,4)
        belief.last_time, belief.last_heading, belief.seen = 25, 'S', False
        g.police_trail = dict(node='(1,4)', xy=(1,4), last_time=25,
                              direction='S', reached=False, extended=False)
        g.search_route, g.search_route_target = ['(6,0)', '(6,3)', '(10,3)'], '(10,3)'
        return g, actor, belief

    def test_n020_does_not_resume_remote_patrol_after_twelve_seconds(self):
        g, actor, belief = self.game()
        action = g.decide('P')
        self.assertEqual(action[1], 'local_search')
        self.assertNotIn(action[2], ('(6,0)','(6,3)','(10,3)'))
        self.assertEqual(g.search_route_target, '(10,3)')

    def test_old_empty_region_is_not_proof_after_new_detection(self):
        g, actor, belief = self.game()
        g.search_observed_at = {n:24 for n in g.search_sites}
        g.search_covered = set(g.search_sites)
        self.assertEqual(g.decide('P')[1], 'local_search')
        self.assertTrue(g.local_search['sites']-g.search_covered)

    def test_dead_corners_before_patrol_and_finite_completion(self):
        g, actor, belief = self.game()
        g.decide('P')
        state = g.local_search
        leaves = state['sites'] & set(g.deadend_arms)
        self.assertTrue(leaves)
        for n in state['sites']-leaves:
            g.search_observed_at[n] = 39
        self.assertEqual(g.decide('P')[1], 'local_deadend')
        for n in leaves:
            g.search_observed_at[n] = 39
        self.assertEqual(g.decide('P')[1], 'search')
        self.assertTrue(state['completed'])

    def test_hidden_opponent_truth_does_not_change_regional_goal(self):
        actions=[]
        for n in ('(0,0)','(11,9)'):
            g, actor, belief = self.game()
            g.robots['B'].node = n
            actions.append(g.decide('P'))
        self.assertEqual(*actions)

    def test_positive_detection_interrupts_local_search(self):
        g, actor, belief = self.game()
        g.decide('P')
        belief.seen, belief.last_time = True, g.now
        belief.last_seen, belief.last_xy = '(0,4)', (0,4)
        self.assertEqual(g.decide('P')[1], 'chase')
        self.assertIsNone(g.local_search)

    def test_world_aligned_quadrants_and_axes(self):
        self.assertEqual([relative_sector((2,2), p) for p in
                          [(3,1),(1,1),(1,3),(3,3),(2,1),(2,2)]],
                         ['I','II','III','IV','N','O'])

    def test_escape_prefers_not_entering_measured_threat_half_plane(self):
        g, _, _ = self.game()
        actor, belief = g.robots['B'], g.beliefs['B']
        actor.node = '(3,2)'
        belief.last_xy = (5,1)  # Northeast, quadrant I.
        away = ['(3,2)','(3,3)','(4,3)']
        toward = ['(3,2)','(3,1)','(4,1)']
        self.assertGreater(g.quadrant_escape_preference(away,actor,belief),
                           g.quadrant_escape_preference(toward,actor,belief))


if __name__ == '__main__':
    unittest.main()

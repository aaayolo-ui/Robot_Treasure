"""Regressions for concrete unsafe choices reported in the replay."""
import json
import unittest

from simulate import Game, ROOT, TrackMap, landmark_type


class RefugeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.track = TrackMap()
        cls.config = json.loads((ROOT / 'config.json').read_text(encoding='utf-8'))

    def measured_game(self, own, facing, opponent, opponent_heading, **kwargs):
        game = Game(self.track, dict(self.config, miss_probability=0), **kwargs)
        game.owner, game.now = 'B', 10
        actor, belief = game.robots['B'], game.beliefs['B']
        actor.node, actor.facing = own, facing
        belief.last_seen, belief.last_xy = opponent, self.track.xy[opponent]
        belief.last_time, belief.last_heading, belief.seen = 10, opponent_heading, True
        game.update_safe_block(actor, belief)
        return game, actor, belief

    def test_right_side_threat_uses_upper_left_exit_not_police_approach(self):
        for mode in ('smart', 'preset'):
            game, actor, belief = self.measured_game('(3,2)', 'N', '(5,2)', 'S', escape_mode=mode)
            self.assertEqual(game.hide(actor, belief)[0], '(3,1)')
            self.assertEqual(game.active_escape_route[:3], ['(3,2)', '(3,1)', '(2,1)'])

    def test_keep_east_then_south_route_instead_of_horizontal_ping_pong(self):
        for mode in ('smart', 'preset'):
            game, actor, belief = self.measured_game('(7,7)', 'E', '(5,7)', 'E', escape_mode=mode)
            self.assertEqual(game.hide(actor, belief)[0], '(8,7)')
            self.assertEqual(game.active_escape_route[:3], ['(7,7)', '(8,7)', '(8,8)'])
            actor.last_node, actor.node = actor.node, '(8,7)'
            game.now += .3
            belief.last_time = game.now
            self.assertEqual(game.hide(actor, belief)[0], '(8,8)')
            belief.seen = False
            actor.last_node, actor.node, actor.facing = '(8,7)', '(8,8)', 'S'
            game.now += .7
            self.assertNotEqual(game.hide(actor, belief)[0], '(8,7)')

    def test_new_refuge_policy_stays_until_actual_detection(self):
        game = Game(self.track, self.config, thief_policy='refuge')
        actor = game.robots['B']
        actor.node, actor.facing = '(4,2)', 'W'
        game.owner, game.hide_goal, game.hide_facing = 'B', actor.node, actor.facing
        for now in (10, 40, 100):
            game.now = now
            self.assertEqual(game.decide('B'), (None, 'hide', '(4,2)'))
        belief = game.beliefs['B']
        belief.last_seen, belief.last_xy, belief.last_time, belief.seen = '(4,1)', (4,1), 100, True
        self.assertIsNotNone(game.decide('B')[0])
        self.assertNotEqual(game.hide_goal, '(4,2)')

    def test_no_hiding_at_plain_subdivision_or_dead_end(self):
        game = Game(self.track, self.config)
        for n in game.hide_candidates:
            self.assertIsNotNone(landmark_type(self.track, n))
            self.assertIn(n, game.cycle_core)
            self.assertGreaterEqual(len(game.hide_exits(n)), 2)
        for n in ('(4,3)', '(2,3)', '(1,0)'):
            self.assertFalse(game.hide_site_safe(n))

    def test_all_core_preset_cards_are_simple_and_avoid_dead_end_arms(self):
        game = Game(self.track, self.config, escape_mode='preset')
        for source in game.cycle_core:
            for path in game.preset_routes[source]:
                self.assertEqual(len(path), len(set(path)))
                self.assertTrue(set(path) <= game.cycle_core)
                self.assertIsNotNone(landmark_type(self.track, path[-1]))

    def test_police_has_three_cells_while_thief_has_two(self):
        game = Game(self.track, dict(self.config, miss_probability=0))
        game.robots['P'].node, game.robots['B'].node = '(6,3)', '(6,6)'
        game.observe()
        self.assertTrue(game.beliefs['P'].seen)
        self.assertFalse(game.beliefs['B'].seen)

    def test_visible_empty_dead_end_is_cleared_without_entering(self):
        game = Game(self.track, dict(self.config, miss_probability=0))
        actor = game.robots['P']
        actor.node, actor.facing = '(2,2)', 'E'
        game.robots['B'].node = '(11,5)'
        for now in (10, 10.02, 10.04):
            game.now = now
            game.observe()
        self.assertIn('(2,3)', game.deadend_checked)
        command = game.inspect_deadends(actor)
        self.assertTrue(command is None or command[2] != '(2,3)')
        self.assertEqual(actor.traveled_m, 0)
        self.assertTrue(any(e['event']=='deadend_cleared_by_sensor' and e['node']=='(2,3)' for e in game.events))

    def test_coverage_search_targets_uninspected_sites_and_not_hidden_truth(self):
        choices = []
        for hidden in ('(11,0)', '(0,0)'):
            game = Game(self.track, self.config)
            actor = game.robots['P']; actor.node, actor.facing = '(6,6)', 'N'
            game.owner, game.police_checked_treasure, game.now = 'B', True, 30
            game.robots['B'].node = hidden
            game.search_covered = set(game.search_sites) - {'(4,2)'}
            choices.append(game.search(actor, game.beliefs['P']))
            self.assertEqual(game.search_sweeps[-1]['new_sites'], ['(4,2)'])
        self.assertEqual(choices[0], choices[1])


if __name__ == '__main__':
    unittest.main()

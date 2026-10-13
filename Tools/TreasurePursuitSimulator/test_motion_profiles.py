"""Optional empirical road speeds must affect routing, motion and collision alike."""
import copy
import json
import unittest
from simulate import ROOT, Robot, Routing, TrackMap, capture_delay, validate_config


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.track=TrackMap()
        self.config=json.loads((ROOT/'config.json').read_text(encoding='utf-8'))
        self.config.pop("measured_motion", None)  # Legacy profile compatibility.
        self.config.update(police_speed_m_s=.3,thief_speed_m_s=.3,turn90_s=.5)
        self.config['motion_profiles']={'B':{'reference_speed_m_s':.3,'turn90_s':.8,
            'edge_speed_m_s':{'(5,0)|(5.5,0)':.15}}}
        validate_config(self.config)
        self.routing=Routing(self.track,self.config)

    def test_same_nominal_speed_has_separate_vehicle_tables(self):
        start,end='(5,0)','(5.5,0)'
        self.assertAlmostEqual(self.routing.edge_time(start,end,'E',.3,'P')[0],.5)
        self.assertAlmostEqual(self.routing.edge_time(start,end,'E',.3,'B')[0],1)
        self.assertNotEqual(self.routing.table(start,'E',.3,'P')[0][end],self.routing.table(start,'E',.3,'B')[0][end])

    def test_physical_motion_and_velocity_use_measured_road_speed(self):
        actor=Robot('B','(5,0)','E',.3)
        actor.command(self.track,self.routing,'(5.5,0)','treasure')
        self.assertAlmostEqual(actor.remaining,1)
        actor.advance(self.track,.5)
        self.assertAlmostEqual(actor.xy(self.track)[0],5.25)
        self.assertAlmostEqual(actor.traveled_m,.075)
        self.assertAlmostEqual(actor.velocity(self.track)[0],.5)

    def test_turn_delay_and_following_move_agree(self):
        actor=Robot('B','(5,0)','N',.3)
        actor.command(self.track,self.routing,'(5.5,0)','treasure')
        self.assertAlmostEqual(actor.remaining,.8)
        actor.advance(self.track,.8); actor.finish(self.track)
        self.assertEqual(actor.phase,'move')
        self.assertAlmostEqual(actor.remaining,1)

    def test_missing_road_and_opposite_direction_keep_base_speed(self):
        self.assertEqual(self.routing.moving_speed('(5.5,0)','(5,0)',.3,'B'),.3)
        self.assertEqual(self.routing.moving_speed('(5,0)','(5.5,0)',.3,'P'),.3)

    def test_speed_factor_scales_profile(self):
        self.assertAlmostEqual(self.routing.moving_speed('(5,0)','(5.5,0)',.36,'B'),.18)

    def test_invalid_profile_and_unknown_edge_rejected(self):
        for value in (0,-1,float('nan')):
            config=copy.deepcopy(self.config)
            config['motion_profiles']['B']['edge_speed_m_s']['(5,0)|(5.5,0)']=value
            with self.assertRaises(ValueError): validate_config(config)
        config=copy.deepcopy(self.config)
        config['motion_profiles']['B']['edge_speed_m_s']['not|a-road']=.3
        with self.assertRaises(ValueError): Routing(self.track,config)

    def test_contact_uses_profile_velocity(self):
        thief=Robot('B','(5,0)','E',.3)
        thief.command(self.track,self.routing,'(5.5,0)','treasure')
        police=Robot('P','(5.5,0)','W',.3)
        police.command(self.track,self.routing,'(5,0)','treasure')
        self.assertAlmostEqual(capture_delay(police,thief,self.track,.03,.3),(.15-.03)/(.15+.3))


if __name__=='__main__': unittest.main()

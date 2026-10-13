"""Measured-mode routing, physical execution, observation limits and contact."""
import copy
import json
import unittest
from simulate import ROOT, TrackMap, Routing, Robot, Game, heading, turn, capture_delay, sensor_channel, validate_config


def complete_command(actor, track):
    elapsed=0.0
    while actor.phase!='ready':
        dt=actor.remaining
        elapsed+=dt
        actor.advance(track,dt)
        actor.finish(track)
    return elapsed


class MeasuredMotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.track=TrackMap()
        cls.config=json.loads((ROOT/'config.json').read_text(encoding='utf-8'))

    def test_reverse_speed_and_switch_delays(self):
        routing=Routing(self.track,self.config)
        speed=self.config['thief_speed_m_s']
        actor=Robot('B','(5,0)','W',speed)
        actor.command(self.track,routing,'(5.5,0)','evade')
        self.assertAlmostEqual(actor.remaining,self.config['measured_motion']['forward_to_reverse_s'])
        self.assertEqual(actor.velocity(self.track),(0,0))
        self.assertAlmostEqual(actor.move_speed,speed*self.config['measured_motion']['reverse_speed_ratio'])
        complete_command(actor,self.track)
        actor.command(self.track,routing,'(5,0)','evade')
        self.assertAlmostEqual(actor.remaining,self.config['measured_motion']['reverse_to_forward_s'])
        self.assertFalse(actor.reversing)
        self.assertEqual(actor.body_facing,'W')

    def test_reverse_corner_keeps_reverse_mode_and_body_sensor_frame(self):
        routing=Routing(self.track,self.config)
        triple=next((a,b,c) for a in self.track.graph for b in self.track.graph[a] for c in self.track.graph[b]
                    if turn(heading(self.track.xy[a],self.track.xy[b]),heading(self.track.xy[b],self.track.xy[c])) in ('LEFT','RIGHT'))
        a,b,c=triple
        body=heading(self.track.xy[b],self.track.xy[a])
        actor=Robot('B',a,body,self.config['thief_speed_m_s'])
        actor.command(self.track,routing,b,'evade');complete_command(actor,self.track)
        actor.command(self.track,routing,c,'evade')
        self.assertTrue(actor.reversing)
        self.assertEqual(turn(actor.body_facing,actor.facing),'UTURN')
        self.assertIsNone(sensor_channel(actor,self.track.xy[c],self.track))
        actor.advance(self.track,actor.remaining);actor.finish(self.track)
        self.assertEqual(sensor_channel(actor,self.track.xy[c],self.track),'rear')

    def test_route_table_matches_executed_route_in_both_modes(self):
        routing=Routing(self.track,self.config)
        for mode in (False,True):
            actor=Robot('B','(6,6)','N',self.config['thief_speed_m_s'],body_facing='S' if mode else 'N')
            target=self.track.resolve('T')
            path,times,_=routing.route(actor.node,actor.facing,actor.speed,target,'B',mode)
            self.assertAlmostEqual(times[-1],routing.table(actor.node,actor.facing,actor.speed,'B',mode)[0][target])
            elapsed=0
            for node in path[1:]:
                actor.command(self.track,routing,node,'evade')
                elapsed+=complete_command(actor,self.track)
            self.assertAlmostEqual(elapsed,times[-1])

    def test_uncertain_opponent_mode_uses_earliest_without_true_state(self):
        routing=Routing(self.track,self.config)
        tables=[routing.table('(6,6)','N',.7,'P',m)[0] for m in (False,True,None)]
        for node in self.track.graph:
            self.assertAlmostEqual(tables[2][node],min(tables[0][node],tables[1][node]))

    def test_collision_during_switch_wait(self):
        routing=Routing(self.track,self.config)
        speed=self.config['police_speed_m_s']
        thief=Robot('B','(5,0)','W',speed)
        police=Robot('P','(5.5,0)','W',speed)
        thief.command(self.track,routing,'(5.5,0)','evade')
        police.command(self.track,routing,'(5,0)','chase')
        collision=capture_delay(police,thief,self.track,.03,min(thief.remaining,police.remaining))
        self.assertIsNotNone(collision)
        self.assertAlmostEqual(collision,.12/speed)

    def test_risk_and_execution_share_slow_reverse_speed(self):
        config=copy.deepcopy(self.config)
        config['measured_motion']['reverse_speed_ratio']=.25
        g=Game(self.track,config)
        actor=Robot('B','(6,6)','N',config['thief_speed_m_s'])
        path=['(6,6)','(6,7)','(6,8)']
        times=g.timed_path(path,actor)
        total=0
        for node in path[1:]:
            actor.command(self.track,g.routing,node,'evade')
            total+=complete_command(actor,self.track)
        self.assertAlmostEqual(times[-1],total)
        self.assertAlmostEqual(total,.6/(actor.speed*.25)+config['measured_motion']['forward_to_reverse_s'])

    def test_measured_patrol_keeps_committed_long_corridor(self):
        g=Game(self.track,dict(self.config,miss_probability=0))
        g.owner,g.now,g.police_checked_treasure='B',30,True
        actor=g.robots['P'];actor.node,actor.facing='(6,0)','S'
        g.search_route=[f'(6,{y})' for y in range(1,10)]
        g.search_route_target='(6,9)'
        g.search_covered=set(g.search_sites)
        for y in range(9):
            actor.node=f'(6,{y})'
            result=g.continue_search_route(actor)
            self.assertEqual(result[0],f'(6,{y+1})')

    def test_sweep_rejects_radius_tangent_but_accepts_usable_window(self):
        g=Game(self.track,dict(self.config,miss_probability=0))
        actor=g.robots['P']
        run=['(5,0)','(5,5)']
        self.assertFalse(g.sweep_observes(run,'(2,1)',actor))
        self.assertTrue(g.sweep_observes(run,'(5,1)',actor))
        # Those boundary sites must lead to a different observation position,
        # rather than promising new coverage by repeating this same corridor.
        g.owner,g.now,g.police_checked_treasure='B',30,True
        actor.node,actor.facing='(5,0)','S'
        g.search_covered=set(g.search_sites)-{'(2,1)','(2,2)','(2,3)','(2,4)'}
        g.search(actor,g.beliefs['P'])
        event=g.search_sweeps[-1]
        self.assertFalse(event['kind']=='straight_sweep' and event['node']=='(5,5)')

    def test_nonfinite_and_negative_mode_values_rejected(self):
        for key in ('reverse_speed_ratio','forward_to_reverse_s','reverse_to_forward_s'):
            for value in (-1,0,float('nan')):
                config=copy.deepcopy(self.config);config['measured_motion'][key]=value
                with self.assertRaises(ValueError):validate_config(config)

    def test_completed_escape_does_not_retrace_fresh_corridor(self):
        g=Game(self.track,dict(self.config,miss_probability=0))
        g.now,g.owner=17.594,'B'
        actor=g.robots['B'];actor.node,actor.facing,actor.last_node='(2,4)','N','(2,5)'
        g.visits['B'].update({'(2,6)':15.935,'(2,5)':17.0,'(2,4)':17.594})
        choice=g.destination_route(actor,g.beliefs['B'],g.branches|set(g.hide_candidates))
        self.assertTrue(any(n not in g.visits['B'] for n in choice[0][1:]))


if __name__=='__main__':unittest.main()

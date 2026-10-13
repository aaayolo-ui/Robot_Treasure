"""Offline map coverage and source-time pipeline tests. All log data are synthetic."""

import binascii
import copy
import json
import math
import unittest
from pathlib import Path

from analyze_routes import analyze, decode, make_config
from build_routes import MAP, ROOT, action, build_plan


def frame(values):
    payload=('MT2,'+','.join(str(n) for n in values)).encode('ascii')
    return payload+b'*'+f'{binascii.crc_hqx(payload,0xFFFF):04X}'.encode()+b'\r\n'


def synthetic(plan, route_index=0, tick_start=100):
    route=plan['routes'][route_index]
    records=[]
    yaw=0
    sequence=0
    def add(tick,event,index,arg=0,angle=None):
        nonlocal sequence,yaw
        sequence+=1
        if angle is not None: yaw=(angle+18000)%36000-18000
        step=route['steps'][index]
        count=round((tick-tick_start)/1000*.3/.204204*1560)
        values=[1,sequence,tick%2**32,event,index+(event in (3,4,5,6,7,8,9,15,17)),
                int(route['id'][1:]),index,int(step['from_node'][1:]),int(step['to_node'][1:]),arg,
                yaw,1,0,sequence,-count,count,-count,count,0]
        records.append(values)
    add(tick_start,2,0,plan['map_route_signature'])
    begin=previous_arrival=tick_start
    for index,step in enumerate(route['steps']):
        add(begin,16,index)
        add(begin+100,14,index,1)
        add(begin+200,14,index,1)
        extra=800 if index and route['steps'][index-1]['arrival_action'] in ('LEFT','RIGHT') else 0
        arrival=previous_arrival+round(step['length_m']/.3*1000)+extra
        add(arrival,15,index,4095)
        add(arrival,3,index,3)
        turn=step['arrival_action']
        if turn=='STOP':
            add(arrival,13,index,3)
            break
        if turn=='STRAIGHT':
            add(arrival,17,index)
            add(arrival+150,9,index,3)
            begin=arrival+150
        else:
            sign=-1 if turn=='LEFT' else 1
            initial=yaw
            add(arrival,4,index,sign)
            add(arrival+200,5,index,sign)
            add(arrival+300,6,index,sign)
            add(arrival+500,7,index,0,initial+sign*5000)
            add(arrival+550,8,index)
            add(arrival+650,9,index,3,initial+sign*9000)
            begin=arrival+650
        previous_arrival=arrival
    return records


def raw(records):
    return b''.join(frame(r) for r in records)


class RouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.map=json.loads(MAP.read_text(encoding='utf-8'))
        cls.plan=build_plan(cls.map)
        cls.base=json.loads((ROOT.parent/'TreasurePursuitSimulator/config.json').read_text(encoding='utf-8'))

    def test_forward_routes_cover_every_segment_both_directions(self):
        measured={(s['id'],s['from'],s['to']) for r in self.plan['routes'] for leg in r['steps'] for s in leg['segments']}
        expected={(s['id'],a,b) for s in self.map['segments'] for a,b in ((s['start'],s['end']),(s['end'],s['start']))}
        self.assertEqual(measured,expected)
        self.assertEqual(len(expected),212)

    def test_connections_lengths_and_actions_are_valid(self):
        for route in self.plan['routes']:
            self.assertEqual(route['start_node'],route['steps'][0]['from_node'])
            self.assertEqual(route['steps'][-1]['arrival_action'],'STOP')
            for i,step in enumerate(route['steps']):
                self.assertAlmostEqual(step['length_m'],sum(s['length_m'] for s in step['segments']))
                self.assertEqual(step['segments'][0]['from'],step['from_node'])
                self.assertEqual(step['segments'][-1]['to'],step['to_node'])
                for a,b in zip(step['segments'],step['segments'][1:]):
                    self.assertEqual(a['to'],b['from'])
                if i+1<len(route['steps']):
                    following=route['steps'][i+1]
                    self.assertEqual(step['to_node'],following['from_node'])
                    self.assertEqual(step['arrival_action'],action(step['heading'],following['heading']))
                    self.assertNotEqual(step['arrival_action'],'UTURN')

    def test_numbering_and_plan_stable_when_input_order_changes(self):
        shuffled=copy.deepcopy(self.map)
        shuffled['nodes'].reverse()
        shuffled['segments'].reverse()
        self.assertEqual(build_plan(shuffled),self.plan)

    def test_treasure_is_combined_not_fake_gray_waypoint(self):
        steps=[s for r in self.plan['routes'] for s in r['steps']]
        self.assertFalse(any('N005' in (s['from_node'],s['to_node']) for s in steps))
        combined=[s for s in steps if len(s['segments'])>1]
        self.assertTrue(combined)
        self.assertTrue(all(len(s['segments'])==2 for s in combined))

    def test_all_deadends_stop_if_entered(self):
        ends={n['id'] for n in self.map['nodes'] if n['degree']==1}
        for r in self.plan['routes']:
            for step in r['steps']:
                if step['to_node'] in ends:
                    self.assertEqual(step['arrival_action'],'STOP')

    def test_invalid_max_steps(self):
        for maximum in (0,65):
            with self.assertRaises(ValueError): build_plan(self.map,maximum)

    def test_source_records_map_automatically(self):
        report=analyze(raw(synthetic(self.plan)),self.plan)
        self.assertTrue(report['sessions'][0]['valid'],report['sessions'][0]['issues'])
        self.assertTrue(report['sessions'][0]['complete'])
        self.assertEqual(len(report['intervals']),len(self.plan['routes'][0]['steps']))
        self.assertTrue(all(i['valid'] for i in report['intervals']))
        self.assertTrue(all(t['valid'] for t in report['turns']))
        self.assertAlmostEqual(report['simulator_candidate']['speed_m_s'],.3)
        self.assertAlmostEqual(report['simulator_candidate']['turn90_extra_s'],.8,delta=.03)
        self.assertTrue(all(abs(t['spin_angle_deg'])==50 and abs(t['full_angle_deg'])==90 for t in report['turns']))
        self.assertTrue(all(abs(t['mean_spin_deg_s']-250)<1e-6 for t in report['turns']))

    def test_crc_corruption_does_not_output_valid_calibration(self):
        log=raw(synthetic(self.plan))
        log=log.replace(b'MT2,1,1,',b'MT2,1,9,',1)
        report=analyze(log,self.plan)
        self.assertTrue(report['corrupt_lines'])
        self.assertTrue(all(not s['valid'] for s in report['sessions']))
        self.assertIsNone(report['simulator_candidate']['speed_m_s'])

    def test_gap_queue_loss_and_wrong_map_rejected(self):
        source=synthetic(self.plan)
        for column,newvalue in ((1,999),(18,1),(9,self.plan['map_route_signature']+1),(8,999)):
            records=copy.deepcopy(source)
            records[0 if column==9 else 3][column]=newvalue
            report=analyze(raw(records),self.plan)
            self.assertFalse(report['sessions'][0]['valid'])

    def test_missing_end_and_fault_stop_rejected(self):
        source=synthetic(self.plan)
        self.assertFalse(analyze(raw(source[:-1]),self.plan)['sessions'][0]['valid'])
        source[-1][9]=2
        self.assertFalse(analyze(raw(source),self.plan)['sessions'][0]['valid'])

    def test_clock_wrap_and_reboot_separate_runs(self):
        log=raw(synthetic(self.plan,tick_start=2**32-500))
        self.assertTrue(analyze(log,self.plan)['sessions'][0]['valid'])
        report=analyze(raw(synthetic(self.plan))+raw(synthetic(self.plan)),self.plan)
        self.assertEqual([s['id'] for s in report['sessions']],['r1.1','r1.2'])
        self.assertTrue(all(s['valid'] for s in report['sessions']))

    def test_combined_road_measurements_are_labelled(self):
        report=analyze(raw(synthetic(self.plan)),self.plan)
        grouped=[r for r in report['roads'] if not r['measured_individually']]
        self.assertEqual(len(grouped),2)
        self.assertTrue(all(not r['distance_calibrated'] for r in report['roads']))

    def test_file_noise_does_not_change_vehicle_time(self):
        log=raw(synthetic(self.plan))
        a=analyze(log,self.plan)
        b=analyze(b'host date unrelated\n'+log.replace(b'MT2,',b'delayed transport: MT2,'),self.plan)
        self.assertEqual(a['intervals'],b['intervals'])

    def test_premature_done_rejected(self):
        records=synthetic(self.plan)[:5]
        ending=copy.deepcopy(records[-1]); ending[1]+=1; ending[3]=13; ending[9]=3
        records.append(ending)
        self.assertFalse(analyze(raw(records),self.plan)['sessions'][0]['valid'])

    def test_draft_updates_only_selected_vehicle_and_preserves_reverse(self):
        report=analyze(raw(synthetic(self.plan)),self.plan)
        before=copy.deepcopy(self.base)
        config=make_config(report,self.plan,self.base,'B')
        self.assertEqual(self.base,before)
        self.assertEqual(config['police_speed_m_s'],before['police_speed_m_s'])
        self.assertEqual(config['uturn_s'],before['uturn_s'])
        self.assertAlmostEqual(config['thief_speed_m_s'],.3)
        self.assertEqual(set(config['motion_profiles']),{'B'})

    def test_no_data_and_unknown_role_cannot_make_config(self):
        report=analyze(b'',self.plan)
        with self.assertRaises(ValueError): make_config(report,self.plan,self.base,'B')
        with self.assertRaises(ValueError): make_config(report,self.plan,self.base,'unknown')


if __name__=='__main__': unittest.main()

"""Synthetic/offline tests only; not physical car measurements."""
import binascii
import copy
import json
import tempfile
import unittest
from pathlib import Path
from build_full_route import build_plan
from build_routes import MAP, ROOT, action, write_tables
from analyze_routes import analyze, decode, make_config


def raw(records):
    lines=[]
    for r in records:
        payload=('MT3,'+','.join(map(str,r))).encode()
        lines.append(payload+b'*'+f'{binascii.crc_hqx(payload,0xffff):04X}'.encode()+b'\r\n')
    return b''.join(lines)


def synthetic(plan):
    rows=[]
    tick=100
    yaw=0
    mode=0
    counts=0
    previous_tick=tick
    def add(at,event,step,arg=0,angle=None):
        nonlocal counts,previous_tick,yaw
        if angle is not None: yaw=(angle+18000)%36000-18000
        # Constant per-mode speed in SAMPLE pairs; other positions are synthetic.
        counts+=round((at-previous_tick)/1000*(.25 if mode==0 else -.18)/.204204*1560)
        previous_tick=at
        rows.append([1,len(rows)+1,at%2**32,event,step['index'],1,step['index'],
                     int(step['from_node'][1:]),int(step['to_node'][1:]),arg,yaw,1,0,len(rows)+1,
                     -counts,counts,-counts,counts,0,mode])
    steps=plan['routes'][0]['steps']
    add(tick,2,steps[0],plan['map_route_signature'])
    previous_arrival=tick
    for i,s in enumerate(steps):
        assert mode==s['drive_mode']
        add(tick,16,s)
        add(tick+100,14,s,1)
        add(tick+200,14,s,1)
        entry='START' if i==0 else steps[i-1]['arrival_action']
        extra=800 if entry in ('LEFT','RIGHT','REVERSE') else 0
        arrival=previous_arrival+round(s['length_m']/(.25 if mode==0 else .18)*1000)+extra
        add(arrival,15,s,4095)
        add(arrival,3,s,3)
        act=s['arrival_action']
        if act=='STOP':
            add(arrival,13,s,3)
            break
        elif act=='STRAIGHT':
            add(arrival,17,s)
            tick=arrival+60
            add(tick,9,s,3)
        elif act=='REVERSE':
            add(arrival,18,s,mode^1)
            add(arrival,5,s,4)
            mode^=1
            add(arrival+100,19,s,mode)
            add(arrival+100,8,s)
            tick=arrival+140
            add(tick,9,s,3)
        else:
            sign=-1 if act=='LEFT' else 1
            initial=yaw
            add(arrival,4,s,sign)
            add(arrival+200,5,s,sign)
            add(arrival+300,6,s,sign)
            add(arrival+500,7,s,0,initial+sign*5000)
            add(arrival+550,8,s)
            tick=arrival+650
            add(tick,9,s,3,initial+sign*9000)
        previous_arrival=arrival
    return rows


class FullRouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.map=json.loads(MAP.read_text(encoding='utf-8'))
        cls.plan=build_plan(cls.map)
        cls.records=synthetic(cls.plan)
        cls.report=analyze(raw(cls.records),cls.plan)

    def test_continuous_exact_cover(self):
        self.assertEqual(len(self.plan['routes']),1)
        route=self.plan['routes'][0]
        self.assertEqual((route['start_node'],route['end_node']),('N081','N081'))
        steps=route['steps']
        self.assertEqual(len(steps),210)
        self.assertEqual(sum(s['arrival_action']=='STOP' for s in steps),1)
        edges=[(s['id'],s['from'],s['to']) for leg in steps for s in leg['segments']]
        expected={(s['id'],a,b) for s in self.map['segments'] for a,b in ((s['start'],s['end']),(s['end'],s['start']))}
        self.assertEqual(set(edges),expected)
        self.assertEqual(len(edges),len(expected))
        self.assertAlmostEqual(sum(s['length_m'] for s in steps),82.8)
        for a,b in zip(steps,steps[1:]):
            self.assertEqual(a['to_node'],b['from_node'])
            act=action(a['heading'],b['heading'])
            self.assertEqual(a['arrival_action'],'REVERSE' if act=='UTURN' else act)
            self.assertEqual(b['drive_mode'],a['drive_mode']^(act=='UTURN'))
        self.assertFalse(any('N005' in (s['from_node'],s['to_node']) for s in steps))
        endpoints={n['id'] for n in self.map['nodes'] if n['degree']==1}
        self.assertTrue(all(s['arrival_action'] in ('REVERSE','STOP') for s in steps if s['to_node'] in endpoints))

    def test_stable_under_input_reordering_and_generated_tables(self):
        data=copy.deepcopy(self.map)
        data['nodes'].reverse();data['segments'].reverse()
        self.assertEqual(build_plan(data),self.plan)
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)
            write_tables(self.plan,target)
            existing=ROOT.parents[1]/'STM32/Keil_Project/Gray12_MapFullRoute_Calibration_Test/Algorithm/MapRoute'
            for name in ('MapRoutePlan.c','MapRoutePlan.h'):
                self.assertEqual((target/'Algorithm/MapRoute'/name).read_bytes(),(existing/name).read_bytes())

    def test_full_capture_and_mode_speeds(self):
        report=self.report
        self.assertTrue(report['sessions'][0]['valid'],report['sessions'][0]['issues'])
        self.assertTrue(report['sessions'][0]['complete'])
        self.assertEqual(len(report['roads']),212)
        self.assertFalse(report['missing_directed_segments'])
        self.assertEqual(len(report['reversals']),self.plan['reverse_switch_count'])
        self.assertTrue(all(r['valid'] for r in report['reversals']))
        self.assertTrue(all(t['valid'] for t in report['turns']))
        self.assertEqual(len(report['turns']),self.plan['quarter_turn_count'])
        self.assertAlmostEqual(report['mode_candidates']['forward']['speed_m_s'],.25,delta=.002)
        self.assertAlmostEqual(report['mode_candidates']['reverse']['speed_m_s'],.18,delta=.002)

    def test_bad_mode_and_missing_switch_rejected(self):
        for change in ('mode','ready','begin','extra_ready'):
            rows=copy.deepcopy(self.records)
            at=next(i for i,r in enumerate(rows) if r[3]==19)
            if change=='mode': rows[at][-1]^=1
            elif change=='ready': rows[at][3]=14
            elif change=='begin': rows[at-2][3]=14
            else:
                rows.insert(at+1,rows[at][:])
                for i,r in enumerate(rows):r[1]=i+1
            report=analyze(raw(rows),self.plan)
            self.assertFalse(report['sessions'][0]['valid'],change)
            self.assertIsNone(report['simulator_candidate']['speed_m_s'])

    def test_no_mixing_reverse_into_config(self):
        base=json.loads((ROOT.parent/'TreasurePursuitSimulator/config.json').read_text(encoding='utf-8'))
        config=make_config(self.report,self.plan,base,'P')
        profile=config['motion_profiles']['P']
        self.assertAlmostEqual(profile['reference_speed_m_s'],.25,delta=.002)
        self.assertTrue(all(abs(v-.25)<.002 for v in profile['edge_speed_m_s'].values()))
        reverse_edges=[]
        for leg in self.plan['routes'][0]['steps']:
            if not leg['drive_mode']:continue
            for segment in leg['segments']:
                s=next(s for s in self.map['segments'] if s['id']==segment['id'])
                nodes=s['original_nodes'] if segment['from']==s['start'] else list(reversed(s['original_nodes']))
                reverse_edges.extend(a+'|'+b for a,b in zip(nodes,nodes[1:]))
        self.assertTrue(all(edge not in profile['edge_speed_m_s'] for edge in reverse_edges))
        self.assertEqual(config.get('uturn_s'),base.get('uturn_s'))

    def test_truncated_and_fault_capture_not_complete(self):
        for change in ('truncated','fault','crc'):
            rows=copy.deepcopy(self.records)
            if change=='truncated':rows=rows[:-1]
            elif change=='fault':rows[-1][9]=2
            data=raw(rows)
            if change=='crc':data=data.replace(b'MT3,1,1,',b'MT3,1,8,',1)
            report=analyze(data,self.plan)
            self.assertFalse(report['sessions'][0]['valid'])

    def test_tick_wrap_valid_and_invalid_mode_decode(self):
        rows=copy.deepcopy(self.records)
        for r in rows:r[2]=(r[2]+2**32-150)%2**32
        self.assertTrue(analyze(raw(rows),self.plan)['sessions'][0]['valid'])
        rows[0][-1]=2
        with self.assertRaises(ValueError):decode(raw(rows[:1]))


if __name__=='__main__':unittest.main()

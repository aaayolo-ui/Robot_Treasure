"""Compare actual ARM C variants under identical limited-observation opponents.

Development seeds select the default; held-out seeds check for regressions.
Fixture scores are separate from complete matches. No real hardware is used.
"""
from pathlib import Path
import argparse,copy,json,statistics,sys
from arm_adapter import build,FIRMWARE
from simulate_16e import FirmwareGame,base,parameter,HERE,SIM

CASES=[('full_equal',1,None),('full_fast20',1.2,None),
       ('fixture_close_N023',1,(23,'S',31,'N')),
       ('fixture_side_N059',1,(59,'W',75,'N'))]
VARIANTS={
 '16D_baseline':{'firmware':FIRMWARE.parent/'Thief16D_RiskAware_FourLaser_Test','defines':()},
 'escape_fallback':{'defines':('THIEF_REACTION_HIDE_SCORE=0','THIEF_FAST_NEAR_CONFIRM=0')},
 'escape_fast':{'defines':('THIEF_REACTION_HIDE_SCORE=0',)},
 'escape_fast_reaction40':{'defines':('THIEF_REACTION_HIDE_SCORE=1','THIEF_HIDE_REACTION_WEIGHT=40')},
 'escape_fast_reaction64':{'defines':('THIEF_REACTION_HIDE_SCORE=1','THIEF_HIDE_REACTION_WEIGHT=64')},
 'escape_turn_reach':{'defines':('THIEF_REACTION_HIDE_SCORE=0','THIEF_TURN_REACH_MODEL=1')},
 **{f'initial_hide_N{n:03d}':{'defines':('THIEF_REACTION_HIDE_SCORE=0',f'THIEF_INITIAL_HIDE_NODE={n}')} for n in (23,39,44,59,62,63)},
 **{f'junction_hide_N{n:03d}':{'defines':('THIEF_REACTION_HIDE_SCORE=0','THIEF_BRANCH_JUNCTION_HIDES=1',f'THIEF_INITIAL_HIDE_NODE={n}')} for n in (11,38,76)},
}

def config():
    cfg=json.loads((SIM/'config.json').read_text(encoding='utf-8'))
    cfg.update(thief_speed_m_s=parameter('THIEF_FORWARD_MM_S')/1000,
        police_speed_m_s=parameter('THIEF_FORWARD_MM_S')/1000,
        turn90_s=parameter('THIEF_TURN_EXTRA_MS')/1000,
        uturn_s=parameter('THIEF_SWITCH_TO_REV_MS')/1000)
    cfg['measured_motion'].update(reverse_speed_ratio=parameter('THIEF_REVERSE_MM_S')/parameter('THIEF_FORWARD_MM_S'),
        forward_to_reverse_s=parameter('THIEF_SWITCH_TO_REV_MS')/1000,
        reverse_to_forward_s=parameter('THIEF_SWITCH_TO_FWD_MS')/1000)
    return cfg

def run(variants,seeds,output,cases=CASES):
    results=[];track=base.TrackMap();cfg=config();hashes={}
    for label in variants:
        choice=VARIANTS[label]
        elf,hashes[label]=build(choice['defines'],label,choice.get('firmware'))
        for case,factor,stage in cases:
            for seed in seeds:
                cc=copy.deepcopy(cfg);cc['police_speed_m_s']*=factor
                game=FirmwareGame(track,cc,elf,stage,seed)
                m=game.run()
                initial=next((e.get('goal') for e in game.planner_events if e['kind']=='initial'),None)
                row={'variant':label,'case':case,'seed':seed,'winner':m['winner'],
                    'end_reason':m['end_reason'],'duration_s':m['duration_s'],'initial_hide':initial,
                    'plan_failures':sum(e['kind']=='fault' for e in game.planner_events),
                    'escape_commands':sum(e['kind']=='escape' for e in game.planner_events),
                    'arrived_hides':sum(e['event']=='firmware_hide' for e in m['events']),
                    'max_states':max((e.get('plan',{}).get('states',0) for e in game.planner_events),default=0)}
                results.append(row)
                print(label,case,seed,round(m['duration_s'],3),m['winner'],initial,
                      'faults',row['plan_failures'],'escapes',row['escape_commands'],flush=True)
    summary=[]
    for variant in variants:
        for case,_,_ in cases:
            rows=[r for r in results if r['variant']==variant and r['case']==case]
            summary.append({'variant':variant,'case':case,'runs':len(rows),
                'thief_wins':sum(r['winner']=='thief' for r in rows),
                'mean_s':round(statistics.mean(r['duration_s'] for r in rows),6),
                'min_s':min(r['duration_s'] for r in rows),
                'max_s':max(r['duration_s'] for r in rows),
                'plan_failures':sum(r['plan_failures'] for r in rows),
                'mean_escape_commands':statistics.mean(r['escape_commands'] for r in rows),
                'initial_hides':sorted({r['initial_hide'] for r in rows if r['initial_hide']})})
    report={'seeds':seeds,'config':cfg,'comparison':summary,'runs':results,
        'variants':{k:{'firmware':str(v.get('firmware',FIRMWARE)),
                        'defines':list(v['defines'])} for k,v in VARIANTS.items() if k in variants},
        'source_hashes':hashes,
        'scope':'Actual Cortex-M3 C planner/gate; same roads, physics, police policy, sensors and contact judge. Ideal localization; no hardware proof.'}
    (HERE/output).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
    return report

if __name__=='__main__':
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    parser=argparse.ArgumentParser()
    parser.add_argument('--variants',nargs='+',choices=VARIANTS,default=list(VARIANTS))
    parser.add_argument('--seeds',nargs='+',type=int,default=[7,8,9])
    parser.add_argument('--output',default='strategy_comparison.json')
    parser.add_argument('--full-only',action='store_true')
    args=parser.parse_args();run(args.variants,args.seeds,args.output,CASES[:2] if args.full_only else CASES)

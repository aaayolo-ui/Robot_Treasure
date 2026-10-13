"""Analyze automatically mapped MT2/MT3 route logs and prepare simulator inputs."""

import argparse
import binascii
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent
EVENTS = {1:'BOOT',2:'START',3:'JUNCTION',4:'APPROACH',5:'BRAKE',6:'ROTATE',
          7:'SPIN_END',8:'FOLLOW',9:'LINE_STABLE',10:'GYRO_WAIT',11:'GYRO_RECOVER',
          12:'FRONT_WAIT',13:'END',14:'SAMPLE',15:'NODE_ARRIVE',16:'LEG_BEGIN',17:'PASS_BEGIN',
          18:'REVERSE_BEGIN',19:'REVERSE_READY'}
WHEEL_CIRCUMFERENCE_M = 0.204204  # Nominal 65 mm tire; not calibrated.
COUNTS_PER_REV = 1560


def decode(line):
    positions = [p for p in (line.find(b'MT2,'),line.find(b'MT3,')) if p >= 0]
    marker = min(positions) if positions else -1
    if marker < 0:
        return None
    payload, checksum = line[marker:].strip().rsplit(b'*',1)
    if len(checksum) != 4 or binascii.crc_hqx(payload,0xFFFF) != int(checksum,16):
        raise ValueError('CRC mismatch or incomplete measurement')
    fields = payload.decode('ascii').split(',')
    version = int(fields[0][-1])
    if len(fields) != (21 if version == 3 else 20):
        raise ValueError('wrong MT field count')
    mode = int(fields[-1]) if version == 3 else 0
    if mode not in (0,1):
        raise ValueError('invalid drive mode')
    run,seq,tick,event,junction,route,step,start,end,arg,yaw,valid,age,frames,a,b,c,d,lost = map(int,fields[1:20])
    if event not in EVENTS or valid not in (0,1) or not 0 <= tick < 2**32 or seq <= 0 or min(run,route,step,age,frames,lost) < 0:
        raise ValueError('invalid record values')
    return dict(run=run, sequence=seq, source_ms=tick, event=EVENTS[event], junction=junction,
                route=f'R{route:03}',step=step,from_node=f'N{start:03}',to_node=f'N{end:03}',
                argument=arg,yaw_deg=yaw/100,yaw_valid=bool(valid),yaw_age_ms=age,yaw_frames=frames,
                encoder_counts=[a,b,c,d],lost=lost,protocol_version=version,drive_mode=mode)


def delta32(a,b):
    return ((b-a+2**31)%2**32)-2**31


def yaw_change(a,b):
    return (b['yaw_deg']-a['yaw_deg']+180)%360-180


def fresh(record):
    return record['yaw_valid'] and record['yaw_age_ms'] <= 150


def sample_speed(records):
    samples = [r for r in records if r['event']=='SAMPLE' and r['argument']==1]
    speeds = []
    for a,b in zip(samples,samples[1:]):
        elapsed = (b['elapsed_ms']-a['elapsed_ms'])/1000
        if (a['drive_mode'] != b['drive_mode'] or not 0 < elapsed <= .5
                or not fresh(a) or not fresh(b) or abs(yaw_change(a,b)) > 10):
            continue
        counts = sum(sign*delta32(x,y) for sign,x,y in zip((-1,1,-1,1),a['encoder_counts'],b['encoder_counts']))/4
        speed = counts * (-1 if a['drive_mode'] else 1)/COUNTS_PER_REV*WHEEL_CIRCUMFERENCE_M/elapsed
        if speed > 0:
            speeds.append(speed)
    return statistics.median(speeds) if speeds else None


def analyze(raw,plan):
    routes = {r['id']:r for r in plan['routes']}
    bad, sessions = [], []
    current = None
    occurrences = defaultdict(int)
    for lineno,line in enumerate(raw.splitlines(),1):
        try:
            record=decode(line)
        except (ValueError,UnicodeError,IndexError) as exc:
            bad.append(dict(line=lineno,error=str(exc)))
            continue
        if record is None or record['run']==0:
            continue
        if record['event']=='START' or current is None or record['run']!=current['run']:
            occurrences[record['run']]+=1
            current=dict(id=f'r{record["run"]}.{occurrences[record["run"]]}',run=record['run'],route=record['route'],records=[],issues=[])
            sessions.append(current)
        records=current['records']
        route=routes.get(current['route'])
        if not route or record['route']!=current['route'] or record['step']>=len(route['steps']):
            current['issues'].append('unknown_route_or_step')
        else:
            expected=route['steps'][record['step']]
            if (record['from_node'],record['to_node'])!=(expected['from_node'],expected['to_node']):
                current['issues'].append('map_context_mismatch')
            if plan['schema_version'] >= 3:
                if record['protocol_version'] != 3:
                    current['issues'].append('MT3_required')
                switched = (record['event']=='REVERSE_READY' or any(
                    r['step']==record['step'] and r['event']=='REVERSE_READY' for r in records))
                expected_mode = expected['drive_mode'] ^ (1 if switched and expected['arrival_action']=='REVERSE' else 0)
                if record['drive_mode'] != expected_mode:
                    current['issues'].append('drive_mode_mismatch')
                if record['event'] in ('REVERSE_BEGIN','REVERSE_READY') and expected['arrival_action']!='REVERSE':
                    current['issues'].append('unexpected_reverse')
        if records:
            previous=records[-1]
            if record['sequence']!=previous['sequence']+1:
                current['issues'].append('sequence_gap_or_duplicate')
            elapsed=(record['source_ms']-previous['source_ms'])%2**32
            if elapsed>3_600_000:
                current['issues'].append('source_clock_reset')
            record['elapsed_ms']=previous['elapsed_ms']+elapsed
            if record['step']<previous['step'] or record['step']>previous['step']+1:
                current['issues'].append('route_step_order')
            if previous['event']=='END':
                current['issues'].append('record_after_end')
        else:
            record['elapsed_ms']=0
            if record['event']!='START' or record['sequence']!=1 or record['step']!=0:
                current['issues'].append('start_missing')
            if record['argument']!=plan['map_route_signature']:
                current['issues'].append('map_route_signature_mismatch')
        if record['lost']:
            current['issues'].append('firmware_queue_loss')
        records.append(record)

    intervals,turns,reversals=[],[],[]
    for session in sessions:
        records=session['records']
        route=routes.get(session['route'])
        if records[-1]['event']!='END':
            session['issues'].append('end_missing')
        elif records[-1]['argument'] not in (1,3):
            session['issues'].append('abnormal_stop')
        if bad:
            session['issues'].append('capture_contains_corrupt_measurement')
        arrivals=[r for r in records if r['event']=='NODE_ARRIVE']
        if [r['step'] for r in arrivals]!=list(range(len(arrivals))):
            session['issues'].append('arrival_order_or_duplicates')
        complete=bool(route and len(arrivals)==len(route['steps']) and records[-1]['event']=='END' and records[-1]['argument']==3)
        if records[-1]['event']=='END' and records[-1]['argument']==3 and not complete:
            session['issues'].append('premature_route_done')
        if any(r['event'] in ('FRONT_WAIT','GYRO_WAIT','GYRO_RECOVER') for r in records):
            session['issues'].append('sensor_recovery')
        if route:
            for step in route['steps']:
                group=[r for r in records if r['step']==step['index']]
                begin=[r for r in group if r['event']=='REVERSE_BEGIN']
                ready=[r for r in group if r['event']=='REVERSE_READY']
                if step['arrival_action']!='REVERSE' or not begin:
                    if step['arrival_action']=='REVERSE' and any(r['event']=='NODE_ARRIVE' for r in group) and complete:
                        session['issues'].append('reverse_begin_missing')
                    continue
                names=('NODE_ARRIVE','REVERSE_BEGIN','BRAKE','REVERSE_READY','FOLLOW','LINE_STABLE')
                # LINE_STABLE/FOLLOW before arrival belong to this leg's entry.
                after=[r for r in group if r['sequence']>=begin[0]['sequence']-1]
                arrived=next((r for r in group if r['event']=='NODE_ARRIVE'),None)
                if arrived:
                    after=[r for r in group if r['sequence']>=arrived['sequence']]
                phases={n:[r for r in after if r['event']==n] for n in names}
                issues=[]
                if any(len(v)!=1 for v in phases.values()):
                    issues.append('incomplete_or_repeated_reverse_phase')
                reverse=dict(session=session['id'],step=step['index'],node=step['to_node'],
                             from_mode=step.get('drive_mode',0),to_mode=step.get('drive_mode',0)^1)
                if not issues:
                    ordered=[phases[n][0] for n in names]
                    if any(a['sequence']>=b['sequence'] or a['elapsed_ms']>b['elapsed_ms'] for a,b in zip(ordered,ordered[1:])):
                        issues.append('reverse_phase_order')
                    arrival,start,brake,switched,follow,stable=ordered
                    if start['argument']!=reverse['to_mode'] or switched['argument']!=reverse['to_mode']:
                        issues.append('reverse_target_mode_mismatch')
                    reverse.update(brake_and_switch_s=(switched['elapsed_ms']-brake['elapsed_ms'])/1000,
                                   full_flow_s=(stable['elapsed_ms']-arrival['elapsed_ms'])/1000)
                reverse.update(valid=not issues,issues=issues)
                reversals.append(reverse)
                session['issues'].extend(issues)
            for index in range(1,len(route['steps'])):
                began=[r for r in records if r['step']==index and r['event']=='LEG_BEGIN']
                if not began:
                    continue
                prior=[r for r in records if r['step']==index-1 and r['sequence']<began[0]['sequence']]
                arrived=next((r for r in prior if r['event']=='NODE_ARRIVE'),None)
                if arrived is None or not any(r['event']=='LINE_STABLE' and r['sequence']>arrived['sequence'] for r in prior):
                    session['issues'].append('advance_without_stable_arrival')
        session['issues']=sorted(set(session['issues']))
        session.update(valid=not session['issues'],complete=complete,completed_legs=len(arrivals))
        if not route:
            continue
        previous_arrival=records[0]
        for arrived in arrivals:
            if arrived['step']>=len(route['steps']):
                continue
            step=route['steps'][arrived['step']]
            group=[r for r in records if r['step']==arrived['step']]
            beginnings=[r for r in group if r['event']=='LEG_BEGIN' and r['sequence']<arrived['sequence']]
            issues=list(session['issues'])
            if len(beginnings)!=1:
                issues.append('leg_begin_missing_or_duplicate')
            total=(arrived['elapsed_ms']-previous_arrival['elapsed_ms'])/1000
            if total<=0:
                issues.append('nonpositive_node_to_node_time')
            entry='START' if arrived['step']==0 else route['steps'][arrived['step']-1]['arrival_action']
            sample_records=[r for r in group if r['sequence']<arrived['sequence']]
            nominal_speed=step['length_m']/total if total>0 and entry=='STRAIGHT' else None
            item=dict(session=session['id'],route=session['route'],step=arrived['step'],from_node=step['from_node'],
                      drive_mode=step.get('drive_mode',0),
                      to_node=step['to_node'],segments=step['segments'],entry_action=entry,nominal_length_m=step['length_m'],
                      node_to_node_s=total,travel_window_s=(arrived['elapsed_ms']-beginnings[0]['elapsed_ms'])/1000 if len(beginnings)==1 else None,
                      nominal_map_speed_m_s=nominal_speed,encoder_estimated_speed_m_s=sample_speed(sample_records),
                      valid=not issues,issues=sorted(set(issues)))
            intervals.append(item)
            previous_arrival=arrived
            if step['arrival_action'] not in ('LEFT','RIGHT'):
                continue
            phase_names=('JUNCTION','APPROACH','BRAKE','ROTATE','SPIN_END','FOLLOW','LINE_STABLE')
            after=[r for r in group if r['sequence']>arrived['sequence']]
            phases={name:[r for r in after if r['event']==name] for name in phase_names}
            turn_issues=list(session['issues'])
            if any(len(v)!=1 for v in phases.values()):
                turn_issues.append('incomplete_or_repeated_turn_phase')
            turn=dict(session=session['id'],route=session['route'],step=arrived['step'],node=step['to_node'],
                      drive_mode=step.get('drive_mode',0),
                      direction=step['arrival_action'],issues=turn_issues,valid=False)
            if 'incomplete_or_repeated_turn_phase' not in turn_issues:
                ordered=[phases[n][0] for n in phase_names]
                if any(a['sequence']>=b['sequence'] or a['elapsed_ms']>b['elapsed_ms'] for a,b in zip(ordered,ordered[1:])):
                    turn_issues.append('turn_phase_order')
                junction,approach,brake,rotate,spin,follow,stable=ordered
                spin_time=(spin['elapsed_ms']-rotate['elapsed_ms'])/1000
                if spin_time<=0:
                    turn_issues.append('nonpositive_spin_time')
                if spin['argument']!=0:
                    turn_issues.append('gray_backup_turn')
                if not (fresh(rotate) and fresh(spin) and fresh(stable) and stable['yaw_frames']>rotate['yaw_frames']):
                    turn_issues.append('yaw_not_fresh')
                turn.update(full_flow_s=(stable['elapsed_ms']-arrived['elapsed_ms'])/1000,
                            stationary_brake_and_spin_s=(spin['elapsed_ms']-brake['elapsed_ms'])/1000,
                            spin_s=spin_time,
                            spin_angle_deg=yaw_change(rotate,spin),full_angle_deg=yaw_change(rotate,stable))
                turn['mean_spin_deg_s']=abs(turn['spin_angle_deg'])/spin_time if spin_time>0 else None
                if not 70<=abs(turn['full_angle_deg'])<=110:
                    turn_issues.append('quarter_turn_not_aligned')
                turn['valid']=not turn_issues
            turn['issues']=sorted(set(turn_issues))
            turns.append(turn)

    # Current simulator has no persistent body/travel mode: export forward only.
    speed_values=[i['nominal_map_speed_m_s'] for i in intervals if i['valid'] and i['drive_mode']==0 and i['nominal_map_speed_m_s']]
    fallback=[i['encoder_estimated_speed_m_s'] for i in intervals if i['valid'] and i['drive_mode']==0 and i['encoder_estimated_speed_m_s']]
    global_speed=statistics.median(speed_values or fallback) if speed_values or fallback else None
    roads=defaultdict(list)
    for item in intervals:
        if not item['valid']:
            continue
        speed=item['nominal_map_speed_m_s'] or item['encoder_estimated_speed_m_s']
        for segment in item['segments']:
            roads[segment['id'],segment['from'],segment['to'],item['drive_mode']].append((item,speed))
    road_output=[]
    for (sid,start,end,mode),samples in sorted(roads.items()):
        speeds=[speed for _,speed in samples if speed]
        road_output.append(dict(segment=sid,from_node=start,to_node=end,drive_mode=mode,samples=len(samples),
                                model_speed_m_s=statistics.median(speeds) if speeds else None,
                                speed_range_m_s=[min(speeds),max(speeds)] if speeds else None,
                                timing_leg_ids=sorted({f'{i["from_node"]}->{i["to_node"]}' for i,_ in samples}),
                                measured_individually=all(len(i['segments'])==1 for i,_ in samples),
                                distance_calibrated=plan['calibrated']))
    extra=[]
    for item in intervals:
        if item['entry_action'] not in ('LEFT','RIGHT') or not item['valid'] or item['drive_mode']!=0:
            continue
        turn=next((t for t in turns if t['session']==item['session'] and t['step']==item['step']-1 and t['valid']),None)
        speed=item['encoder_estimated_speed_m_s'] or global_speed
        if turn and speed:
            residual=item['node_to_node_s']-item['nominal_length_m']/speed
            if residual>=0:
                extra.append(dict(node=turn['node'],direction=turn['direction'],extra_s=residual,
                                  method='node_to_node_time_minus_full_nominal_length_over_reference_speed'))
    measured={(r['segment'],r['from_node'],r['to_node']) for r in road_output}
    speed_measured={(r['segment'],r['from_node'],r['to_node']) for r in road_output if r['model_speed_m_s'] is not None}
    expected={(s['id'],a,b) for s in plan['segments'] for a,b in ((s['start'],s['end']),(s['end'],s['start']))}
    for reversal in reversals:
        session=next(s for s in sessions if s['id']==reversal['session'])
        reversal['issues']=sorted(set(reversal['issues']+session['issues']))
        reversal['valid']=not reversal['issues']
    mode_speeds={}
    for mode,name in ((0,'forward'),(1,'reverse')):
        valid=[i for i in intervals if i['valid'] and i['drive_mode']==mode]
        values=[i['nominal_map_speed_m_s'] for i in valid if i['nominal_map_speed_m_s']]
        estimates=[i['encoder_estimated_speed_m_s'] for i in valid if i['encoder_estimated_speed_m_s']]
        mode_speeds[name]=dict(speed_m_s=statistics.median(values or estimates) if values or estimates else None,
                              method='nominal_map_straight_transit' if values else 'nominal_encoder_scale',
                              completed_legs=len(valid))
    return dict(schema_version=plan['schema_version'],map_route_signature=plan['map_route_signature'],source_sha256=plan['source_sha256'],
                corrupt_lines=bad,sessions=sessions,intervals=intervals,turns=turns,reversals=reversals,roads=road_output,
                mode_candidates=mode_speeds,
                missing_directed_segments=[dict(segment=s,from_node=a,to_node=b) for s,a,b in sorted(expected-measured)],
                missing_speed_entries=[dict(segment=s,from_node=a,to_node=b) for s,a,b in sorted(expected-speed_measured)],
                simulator_candidate=dict(speed_m_s=global_speed,turn90_extra_s=statistics.median([e['extra_s'] for e in extra]) if extra else None,
                                         speed_method='nominal_map_straight_transit' if speed_values else 'nominal_encoder_scale',turn_extra_samples=extra),
                hardware_verified=False,distance_calibrated=plan['calibrated'],
                note='Source times are measured; speed and extra-turn cost are model estimates using nominal map/tire dimensions. The full turning flow includes translation and must not be added again as turn90_s. Operator must confirm starting placement and actual route before use.')


def make_config(report,plan,base,role):
    if role not in ('P','B'):
        raise ValueError('Select police or thief explicitly')
    speed=report['simulator_candidate']['speed_m_s']
    if speed is None or not math.isfinite(speed) or speed<=0:
        raise ValueError('No valid speed samples; cannot prepare simulation config')
    config=dict(base)
    key='police_speed_m_s' if role=='P' else 'thief_speed_m_s'
    config[key]=speed
    edge_speeds={}
    segments={s['id']:s for s in plan['segments']}
    for road in report['roads']:
        if road.get('drive_mode',0)!=0 or not road['model_speed_m_s']:
            continue
        s=segments[road['segment']]
        originals=s['original_nodes'] if road['from_node']==s['start'] else list(reversed(s['original_nodes']))
        for a,b in zip(originals,originals[1:]):
            edge_speeds[a+'|'+b]=road['model_speed_m_s']
    profiles=dict(base.get('motion_profiles',{}))
    profile=dict(reference_speed_m_s=speed,edge_speed_m_s=edge_speeds,
                 distance_calibrated=report['distance_calibrated'],source_sha256=report['source_sha256'])
    turn_extra=report['simulator_candidate']['turn90_extra_s']
    if turn_extra is not None:
        profile['turn90_s']=turn_extra
    profiles[role]=profile
    config['motion_profiles']=profiles
    config['status']='根据 MT2/MT3 实跑日志的前进样本生成；名义尺寸尚需标定。倒车单列于报告，当前仿真不区分车身模式，未混入前进参数；另一辆车沿用原配置'
    return config


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs',type=Path,nargs='+')
    parser.add_argument('--routes',type=Path,default=ROOT/'map_routes.json')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config-out',type=Path)
    parser.add_argument('--role',choices=('police','thief'))
    parser.add_argument('--confirm-start-placements',action='store_true')
    parser.add_argument('--base-config',type=Path,default=ROOT.parent/'TreasurePursuitSimulator/config.json')
    args=parser.parse_args()
    plan=json.loads(args.routes.read_text(encoding='utf-8-sig'))
    # Each file is a separate capture: a corrupt file must not invalidate other files.
    reports=[analyze(p.read_bytes(),plan) for p in args.logs]
    report=reports[0] if len(reports)==1 else None
    if report is None:
        # Reanalyze only clean complete captures, and retain all rejected captures separately.
        accepted=[p.read_bytes() for p,r in zip(args.logs,reports) if r['sessions'] and all(s['valid'] for s in r['sessions'])]
        report=analyze(b'\n'.join(accepted),plan)
        report['capture_reports']=[dict(file=str(p),valid=bool(r['sessions']) and all(s['valid'] for s in r['sessions']),report=r) for p,r in zip(args.logs,reports)]
    report['operator_confirmed_start_placements']=args.confirm_start_placements
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    if args.config_out:
        if not args.role or not args.confirm_start_placements:
            parser.error('--config-out requires --role and --confirm-start-placements')
        base=json.loads(args.base_config.read_text(encoding='utf-8-sig'))
        config=make_config(report,plan,base,'P' if args.role=='police' else 'B')
        args.config_out.parent.mkdir(parents=True,exist_ok=True)
        args.config_out.write_text(json.dumps(config,ensure_ascii=False,indent=2),encoding='utf-8')
    print(f'{len(report["roads"])} directed road entries; {len(report["missing_directed_segments"])} unmeasured; report={args.output}')


if __name__=='__main__':
    main()

"""Replay the same calibration path through Robot; compare STM32 arrival times.

This is an in-sample consistency check, not independent hardware validation.
"""
import json
import sys
from pathlib import Path
from simulate import ROOT,TrackMap,Routing,Robot
sys.path.insert(0,str(ROOT.parent/'MapTimingLogger'))
from analyze_routes import analyze


def evaluate(config,plan,records):
    track=TrackMap();routing=Routing(track,config)
    nodes={n['id']:n['original_node'] for n in plan['nodes']}
    segments={s['id']:s for s in plan['segments']}
    route=plan['routes'][0]
    actor=Robot('B',nodes[route['start_node']],route['start_heading'],config['thief_speed_m_s'])
    elapsed=0.0;rows=[]
    arrivals={r['step']:r['elapsed_ms']/1000 for r in records if r['event']=='NODE_ARRIVE'}
    for step in route['steps']:
        for segment in step['segments']:
            data=segments[segment['id']]
            chain=data['original_nodes'] if segment['from']==data['start'] else list(reversed(data['original_nodes']))
            assert actor.node==chain[0]
            for node in chain[1:]:
                actor.command(track,routing,node,'calibration')
                while actor.phase!='ready':
                    dt=actor.remaining;elapsed+=dt
                    actor.advance(track,dt);actor.finish(track)
        actual=arrivals[step['index']]
        rows.append(dict(step=step['index'],node=step['to_node'],drive_mode=step['drive_mode'],
                         simulated_s=elapsed,recorded_s=actual,error_s=elapsed-actual))
        assert actor.reversing==bool(step['drive_mode'])
    return dict(total_s=elapsed,recorded_s=actual,error_s=elapsed-actual,error_percent=100*(elapsed/actual-1),
                max_abs_node_error_s=max(abs(r['error_s']) for r in rows),
                node_mean_abs_error_s=sum(abs(r['error_s']) for r in rows)/len(rows),nodes=rows)


def main():
    logger=ROOT.parent/'MapTimingLogger'
    plan=json.loads((logger/'full_map_route.json').read_text(encoding='utf-8'))
    report=analyze((logger/'logs/20261006-210343-671.log').read_bytes(),plan)
    session=report['sessions'][0]
    assert session['valid'] and session['complete']
    config=json.loads((ROOT/'config.json').read_text(encoding='utf-8'))
    legacy=dict(config,police_speed_m_s=1.95,thief_speed_m_s=1.95,turn90_s=.5,uturn_s=.5)
    legacy.pop('measured_motion',None)
    old,new=evaluate(legacy,plan,session['records']),evaluate(config,plan,session['records'])
    result=dict(source_log='20261006-210343-671.log',in_sample=True,old=old,measured=new,
                note='Same capture estimates parameters and checks timing; independent repeated run still needed. Stationary delays approximate moving turn phases.')
    (ROOT/'measured_route_validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    lines=['# 实测运动模型路线核对','','本次使用同一份日志标定并核对，属于样本内一致性检查，不是独立预测验证。', '',
           '| 模型 | 全图时间/s | 相对实车误差 | 节点累计时间平均绝对误差/s |', '| --- | ---: | ---: | ---: |']
    for label,data in (('之前 1.95 m/s、0.5 s 动作',old),('本次模式区分模型',new)):
        lines.append(f'| {label} | {data["total_s"]:.3f} | {data["error_percent"]:+.2f}% | {data["node_mean_abs_error_s"]:.3f} |')
    lines+=['',f'实车记录 {new["recorded_s"]:.3f} s。使用原地图 N/S 连接、全图 210 步和实际 Robot 执行动作进行对照。','','| 步骤 | 节点 | 仿真/s | 实车/s | 偏差/s |','| --- | --- | ---: | ---: | ---: |']
    for i in (0,49,99,149,209):
        row=new['nodes'][i]
        lines.append(f'| {i} | {row["node"]} | {row["simulated_s"]:.3f} | {row["recorded_s"]:.3f} | {row["error_s"]:+.3f} |')
    lines+=['','模型采用名义尺寸速度，转弯为额外时间代价、换向为重新识线时间；未模拟完整加减速、惯性滑移、旋转车身包络。双方共用一辆车数据属于对抗假设。','']
    (ROOT/'measured_route_validation.md').write_text('\n'.join(lines),encoding='utf-8')
    print(json.dumps({label:{k:v for k,v in data.items() if k!='nodes'} for label,data in (('old',old),('measured',new))},indent=2))


if __name__=='__main__':main()

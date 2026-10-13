"""Summarize validated MT3 captures into basic physical-motion measurements.

No firmware or simulation configuration is changed. Position/velocity estimates
retain the nominal geometry limitation of analyze_routes.py.
"""
import argparse
import hashlib
import json
import statistics
from pathlib import Path
from analyze_routes import analyze


def describe(values):
    values=sorted(values)
    if not values:
        return None
    def percentile(p):
        x=(len(values)-1)*p
        lo=int(x)
        hi=min(lo+1,len(values)-1)
        return values[lo]+(values[hi]-values[lo])*(x-lo)
    return dict(n=len(values),median=statistics.median(values),mean=statistics.mean(values),
                p10=percentile(.1),p90=percentile(.9),minimum=values[0],maximum=values[-1])


def summarize(report):
    valid_sessions={s['id']:s for s in report['sessions'] if s['valid']}
    rows=[]
    for turn in report['turns']:
        if not turn['valid'] or turn['session'] not in valid_sessions:
            continue
        records=valid_sessions[turn['session']]['records']
        arrival=next(r for r in records if r['step']==turn['step'] and r['event']=='NODE_ARRIVE')
        phases={r['event']:r for r in records if r['step']==turn['step'] and r['sequence']>arrival['sequence']}
        brake,rotate,spin,stable=[phases[n] for n in ('BRAKE','ROTATE','SPIN_END','LINE_STABLE')]
        rows.append(dict(step=turn['step'],node=turn['node'],mode=turn['drive_mode'],direction=turn['direction'],
                         brake_to_rotate_s=(rotate['elapsed_ms']-brake['elapsed_ms'])/1000,
                         rotation_phase_s=turn['spin_s'],rotation_phase_angle_deg=abs(turn['spin_angle_deg']),
                         brake_to_aligned_s=(stable['elapsed_ms']-brake['elapsed_ms'])/1000,
                         node_to_aligned_s=turn['full_flow_s'],full_angle_deg=abs(turn['full_angle_deg'])))
    measurements={}
    for mode,name in ((0,'forward'),(1,'reverse')):
        legs=[i for i in report['intervals'] if i['valid'] and i['drive_mode']==mode]
        measurements[name+'_map_speed_m_s']=describe([i['nominal_map_speed_m_s'] for i in legs if i['nominal_map_speed_m_s'] is not None])
        measurements[name+'_encoder_speed_m_s']=describe([i['encoder_estimated_speed_m_s'] for i in legs if i['encoder_estimated_speed_m_s'] is not None])
    for key in ('brake_to_rotate_s','rotation_phase_s','rotation_phase_angle_deg','brake_to_aligned_s','node_to_aligned_s','full_angle_deg'):
        measurements[key]=describe([r[key] for r in rows])
    by_turn={}
    for mode,name in ((0,'forward'),(1,'reverse')):
        for direction in ('LEFT','RIGHT'):
            selected=[r for r in rows if r['mode']==mode and r['direction']==direction]
            by_turn[name+'_'+direction.lower()]={k:describe([r[k] for r in selected]) for k in ('rotation_phase_s','brake_to_rotate_s','brake_to_aligned_s','node_to_aligned_s')}
    for mode,name in ((0,'forward_to_reverse'),(1,'reverse_to_forward')):
        reversals=[r for r in report['reversals'] if r['valid'] and r['from_mode']==mode]
        measurements[name+'_ready_s']=describe([r['brake_and_switch_s'] for r in reversals])
        measurements[name+'_stable_s']=describe([r['full_flow_s'] for r in reversals])
    extra=report['simulator_candidate']['turn_extra_samples']
    measurements['forward_turn90_model_extra_s']=describe([r['extra_s'] for r in extra])
    return dict(sessions=[dict(id=s['id'],valid=s['valid'],complete=s['complete'],issues=s['issues'],
                                records=len(s['records']),duration_s=s['records'][-1]['elapsed_ms']/1000,
                                completed_legs=s['completed_legs']) for s in report['sessions']],
                measurements=measurements,turns_by_mode_and_direction=by_turn,turn_measurements=rows,
                limitations=['Active braking, not motor-off coast-down.',
                             'Standstill criterion: all wheel speeds <= 2 RPM for two feedback samples, minimum 80 ms; not absolute rest.',
                             'BRAKE to ROTATE also includes minimum dwell and fresh-yaw waiting; actual physical stop time not separately logged.',
                             'Rotation phase targets 50 degrees then translates to reacquire line; not a standalone 90-degree spin.',
                             'REVERSE_READY is sensor/controller readiness, not measured first backward displacement.',
                             'LINE_STABLE is three ordinary valid leading-gray samples, not settled cruise speed.',
                             'Map and tire dimensions are nominal, speeds need physical scale calibration.',
                             'One complete run, variation across nodes is not repeated-trial confidence.'])


def markdown(result):
    angles=result['measurements']['full_angle_deg']
    angle_note=(f'本趟该事件的角度中位数 {angles["median"]:.2f}°、范围 {angles["minimum"]:.2f}°–{angles["maximum"]:.2f}°。'
                if angles else '本次没有有效的重新识线角度样本。')
    names={
        'forward_map_speed_m_s':('直行速度（名义地图直段）','m/s'),
        'reverse_map_speed_m_s':('倒车速度（名义地图直段）','m/s'),
        'forward_encoder_speed_m_s':('直行速度（编码器参考）','m/s'),
        'reverse_encoder_speed_m_s':('倒车速度（编码器参考）','m/s'),
        'brake_to_rotate_s':('主动刹车指令→允许开始旋转','s'),
        'rotation_phase_s':('实际旋转阶段（目标 50°）','s'),
        'rotation_phase_angle_deg':('该旋转阶段实测角度','°'),
        'brake_to_aligned_s':('刹车指令→转弯重新识线判据','s'),
        'node_to_aligned_s':('路口识别→转弯重新识线判据','s'),
        'full_angle_deg':('重新识线时记录的车身角度变化','°'),
        'forward_to_reverse_ready_s':('前进刹车→倒车控制就绪','s'),
        'forward_to_reverse_stable_s':('前进换向触发→倒车重新识线','s'),
        'reverse_to_forward_ready_s':('倒车刹车→前进控制就绪','s'),
        'reverse_to_forward_stable_s':('倒车换向触发→前进重新识线','s'),
        'forward_turn90_model_extra_s':('前进 90°额外代价（模型估计）','s')}
    lines=['# 全图实车基础运动数据','',f'来源：`{result["source_log"]}`。原始文件 SHA256：`{result["source_sha256"]}`。','',
           '时间来自 STM32 事件时间戳。表中样本是同一趟内不同节点/路段，P10–P90 表示本趟分布，不是重复试验置信区间。','',
           '| 测量项 | 样本数 | 中位数 | P10–P90 | 最小–最大 |','| --- | ---: | ---: | --- | --- |']
    for key,(name,unit) in names.items():
        s=result['measurements'][key]
        if s:
            lines.append(f'| {name} | {s["n"]} | {s["median"]:.3f} {unit} | {s["p10"]:.3f}–{s["p90"]:.3f} | {s["minimum"]:.3f}–{s["maximum"]:.3f} |')
    lines+=['','## 完整性','']
    for s in result['sessions']:
        lines.append(f'- {s["id"]}：完整={s["complete"]}，校验有效={s["valid"]}，{s["records"]} 条记录，完成 {s["completed_legs"]} 步，耗时 {s["duration_s"]:.3f} s。问题：{s["issues"]}。')
    lines+=['','## 用于策略时的含义','',
            '当前使用主动短刹车：四轮轮速都不超过约 2 RPM，连续两次检测，且至少等待 80 ms，才允许换向/转弯。BRAKE→ROTATE 还可能包含新陀螺帧等待；不能称为自由滑行停止时间，也不能认定轮速绝对为零。','',
            '旋转阶段目标是 50°，之后车辆移动并循线。重新识线采用三个有效灰度样本及 70°–110°航向范围。'+angle_note+'这不能证明真正转到 90°后完全停稳。因此“刹车→重新识线”只能表示程序允许进入下一段的流程。其中包含位移，不能在按全长计算的直行时间上再原样叠加。前进 90°额外代价是逻辑直角路口的模型估计，不是实测原地旋转 90°时间。','',
            '前进→倒车“就绪”是停稳判据满足、新前端有效线确认后的控制切换时刻；后面的“重新识线”是连续三个有效灰度样本。它们没有直接记录车轮首次反向运动、车身首次倒退或恢复巡航速度的时刻，不能等同机械响应延迟。','',
            '名义地图法与名义轮径编码器法速度都保留，差异需要用实测距离、轮径、打滑和传感器触发位置核对。当前不能承诺绝对精度。','',
            '建议策略先采用前进/倒车各自的速度中位数，转弯/换向按定义一致的模型使用；对安全避碰同时考虑 P90 动作耗时。当前只输出报告，不自动覆盖仿真参数或修改固件。','',
            '若要精确补齐“刹车到物理停止”和“首次反向运动”，下一版需在制动/换向过程增加轮速和编码器周期记录；若关心轮胎打滑时的车身残余运动，还需同步视频或独立位置测量。现有日志没有这些数据，不能回推成精确值。','']
    return '\n'.join(lines)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',type=Path)
    parser.add_argument('--routes',type=Path,default=Path(__file__).with_name('full_map_route.json'))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    raw=args.log.read_bytes()
    report=analyze(raw,json.loads(args.routes.read_text(encoding='utf-8-sig')))
    result=summarize(report)
    result.update(source_log=str(args.log.resolve()),source_sha256=hashlib.sha256(raw).hexdigest())
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.with_suffix('.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    args.output.with_suffix('.md').write_text(markdown(result),encoding='utf-8')
    print(json.dumps({k:v for k,v in result['measurements'].items()},ensure_ascii=False,indent=2))


if __name__=='__main__':main()

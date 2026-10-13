"""Deterministic continuous directed-edge cover; reverse travel, no 180-degree spin."""
import argparse
import copy
import hashlib
import json
import random
from pathlib import Path
import build_routes as batches

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parents[1] / 'STM32/Keil_Project/Gray12_MapFullRoute_Calibration_Test'


def build_plan(annotations):
    base = batches.build_plan(annotations)
    legs = {(s['from_node'], s['to_node']): s for r in base['routes'] for s in r['steps']}
    outgoing = {}
    for a, b in sorted(legs):
        outgoing.setdefault(a, []).append(b)
    # Hierholzer gives each directed leg exactly once. Try fixed tie orders to
    # reduce switches/turns, without claiming a globally fastest circuit.
    best = None
    for seed in range(96):
        rng = random.Random(seed)
        remaining = copy.deepcopy(outgoing)
        stack, circuit = ['N081'], []
        while stack:
            u = stack[-1]
            if remaining[u]:
                candidates = remaining[u][:]
                rng.shuffle(candidates)
                if len(stack) > 1:
                    heading = legs[stack[-2], u]['heading']
                    candidates.sort(key=lambda v: (
                        batches.action(heading, legs[u, v]['heading']) == 'UTURN',
                        batches.action(heading, legs[u, v]['heading']) != 'STRAIGHT'))
                v = candidates[0]
                remaining[u].remove(v)
                stack.append(v)
            else:
                circuit.append(stack.pop())
        path = list(reversed(circuit))
        actions = [batches.action(legs[a, b]['heading'], legs[b, c]['heading'])
                   for a, b, c in zip(path, path[1:], path[2:])]
        score = (actions.count('UTURN'), sum(a in ('LEFT', 'RIGHT') for a in actions), tuple(path))
        if best is None or score < best[0]:
            best = score, path
    score, path = best
    if len(path) != len(legs) + 1 or path[0] != path[-1] or set(zip(path, path[1:])) != set(legs):
        raise ValueError('Map must admit a connected continuous directed cover')
    steps, mode = [], 0
    for i, edge in enumerate(zip(path, path[1:])):
        step = copy.deepcopy(legs[edge])
        act = batches.action(step['heading'], legs[path[i+1], path[i+2]]['heading']) if i+2 < len(path) else 'STOP'
        step.update(index=i, drive_mode=mode, arrival_action='REVERSE' if act == 'UTURN' else act)
        steps.append(step)
        if act == 'UTURN':
            mode ^= 1
    base.update(schema_version=3, mode='continuous_forward_reverse', routes=[dict(
        id='R001', start_node=path[0], start_heading=steps[0]['heading'], end_node=path[-1], steps=steps)],
        reverse_switch_count=score[0], quarter_turn_count=score[1],
        nominal_total_length_m=round(sum(s['length_m'] for s in steps), 6),
        note='One known start; leading gray changes only after braking. Every geographic direction once, not every body mode. No absolute localization; map and tire lengths uncalibrated.')
    canonical = json.dumps({k: base[k] for k in ('nodes','segments','routes')}, sort_keys=True, separators=(',', ':'))
    digest = hashlib.sha256(canonical.encode()).hexdigest()
    base.update(source_sha256=digest, map_route_signature=int(digest[:8],16)&0x7fffffff)
    return base


def instructions(plan):
    lines = ['# 全图连续采集路线', '',
        '独立工程：Gray12_MapFullRoute_Calibration_Test。前灰度对准 N081 起点黑线，车头朝地图上方 N；按 KEY2 一次启动，最终回到 N081 自动停车。运行中任意 KEY1/WK_UP/KEY2 停车，松手不会重新启动。', '',
        f'一条连续路线，{len(plan["routes"][0]["steps"])} 个可识别节点到达步骤，覆盖 {plan["segment_count"]} 条路段的两个方向，共 {plan["planned_directed_segment_count"]} 项。名义距离 {plan["nominal_total_length_m"]} m；90° 转弯 {plan["quarter_turn_count"]} 次；前进/倒车切换 {plan["reverse_switch_count"]} 次。', '',
        '切换方向先确认轮速停稳，再切换前/后灰度，不原地转 180°。转弯按当前行驶方向判断左右。N005 不是灰度路口，与相邻直段合并测量。F=前进，R=倒车。', '',
        '路线长度沿用名义地图尺寸，尚未实地标定。未完成硬件验证；节点靠已知起点、灰度顺序和编码器距离合理性检查推断，不能保证识别跳漏后自动重新定位。', '',
        '| 步骤 | 模式 | 起点 | 终点 | 路段 | 名义长度/m | 到达动作 |', '| --- | --- | --- | --- | --- | --- | --- |']
    words = dict(LEFT='左转',RIGHT='右转',STRAIGHT='直行',REVERSE='停稳并切换前进/倒车',STOP='全图结束停车')
    for s in plan['routes'][0]['steps']:
        lines.append(f'| {s["index"]} | {"R" if s["drive_mode"] else "F"} | {s["from_node"]} | {s["to_node"]} | {" + ".join(x["id"] for x in s["segments"])} | {s["length_m"]:.2f} | {words[s["arrival_action"]]} |')
    return '\n'.join(lines)+'\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',type=Path,default=batches.MAP)
    parser.add_argument('--project',type=Path,default=PROJECT)
    parser.add_argument('--output',type=Path,default=ROOT/'full_map_route.json')
    args = parser.parse_args()
    plan = build_plan(json.loads(args.map.read_text(encoding='utf-8-sig')))
    batches.write_tables(plan,args.project)
    args.output.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    args.output.with_suffix('.md').write_text(instructions(plan),encoding='utf-8')
    print(f'1 route; {plan["planned_directed_segment_count"]} directed segments; {plan["nominal_total_length_m"]} m; signature={plan["map_route_signature"]}')


if __name__ == '__main__':
    main()

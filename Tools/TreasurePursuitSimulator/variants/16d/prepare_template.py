"""Reuse the established annotated replay without modifying the original page."""
from pathlib import Path
HERE=Path(__file__).resolve().parent
text=(HERE.parents[1]/'replay.template.html').read_text(encoding='utf-8')
text=text.replace('密室夺宝 双车追逃仿真','16D 固件策略回放')
text=text.replace('<p id="detail"', '<div id="firmware" class="map-info"></div>\n<div class="controls"><button id="hide-jump">跳到首次藏匿</button><button id="decision-jump">跳到检测决策</button></div>\n<p id="detail"')
a=text.index('  el("intro").textContent=c.measured_motion?')
b=text.index('  const directions=',a)
text=text[:a]+'''  el("intro").textContent="小偷使用实际ARMCC编译的16D规划器和激光门控C代码；警方沿用现有有限检测策略。移动期激光关闭、停稳后检测；原回放保留。专项场景从已持宝藏点开始，不作为完整比赛胜率。";
'''+text[b:]
text=text.replace('      const x=X(f[off]),y=Y(f[off+1]),r=',
                  '      if(off===9&&!f[29])continue;\n      const x=X(f[off]),y=Y(f[off+1]),r=')
lines=text.splitlines()
for i,line in enumerate(lines):
    if 'el("detail").textContent=' in line:
        lines[i]='''    el("detail").textContent=`警方：${reasons[f[6]]}，${phases[f[5]]}；小偷：${f[28]?"无安全首段，故障停车":reasons[f[13]]}，${phases[f[12]]}，${f[27]?"倒车":"前进"}。小偷激光：${f[28]?"故障状态":f[29]?"停稳检测":"移动／停稳等待时关闭"}。${captured?"达到0.18米接触阈值，立即判警方获胜。":""}`;'''
    if 'el("parameters").textContent=' in line:
        lines[i]='''    el("parameters").textContent=`警方 ${fmt(match.police_speed_m_s)} m/s，小偷 ${fmt(match.thief_speed_m_s)} m/s；${motionText(match)}。两格0.6米检测，20ms周期、5%漏帧；实际C代码检查3/5稳定帧、道路投影和场界。小偷只在停稳后检测，行驶时不获取警方位置。${match.fixture?"本场为指定起始位置的藏匿阶段专项测试。":"本场从双方出生点正常开始，取宝不停车。"}机械运动、定位、转向等待是理想模型；不模拟100ms板上计算超时、串口电气与灰度误识别。规划器中的时间余量不是实测防撞保证。`;'''
    if 'el("metrics").textContent=' in line:
        lines[i]+='''
    const decision=(match.planner_events||[]).filter(d=>d.t<=f[0]+.0001).at(-1), p=decision?.plan;
    el("firmware").textContent=decision?.kind==="fault"?`16D实际C返回：${decision.code}。当前模型没有可接受的逃跑首段，保持停车；这不代表成功躲藏。`:
      p?`16D实际C决策：目标 ${decision.goal}；预计路线 ${(p.travel_ms/1000).toFixed(2)}秒；展开 ${p.states}/704 状态；有效威胁 ${p.threats}；${p.threats?`短期最小时间余量 ${p.margin_ms}ms`:"暂无检测线索，使用地图先验选藏点"}；${p.degraded?"退化选择：完整路线仍有风险，首段通过筛选":"优先级正常"}。`:"尚未产生16D决策。";'''
text='\n'.join(lines)+'\n'
text=text.replace('  function reset(){','''  function jumpTo(t){playing=false;el("play").textContent="播放";index=match.frames.findIndex(f=>f[0]>=t-.0001);if(index<0)index=match.frames.length-1;replayTime=match.frames[index][0];render();}
  el("hide-jump").addEventListener("click",()=>{const e=(match.events||[]).find(e=>e.event==="firmware_hide");jumpTo(match.fixture?0:e?.t??match.duration_s);});
  el("decision-jump").addEventListener("click",()=>{const e=(match.planner_events||[]).find(e=>e.kind!=="initial");jumpTo(e?.t??match.duration_s);});
  function reset(){''')
text=text.replace('红外','激光').replace('修改 config.json 后运行 simulate.py 可重新生成回放。','运行 variants/16d/simulate_16d.py 生成本页。')
(HERE/'replay.template.html').write_text(text,encoding='utf-8')

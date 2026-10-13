"""Add turn-window diagnostics to the existing local replay controls."""
from pathlib import Path
HERE=Path(__file__).resolve().parent
s=(HERE.with_name('16g')/'replay.template.html').read_text(encoding='utf-8').replace('16G','16H')
s=s.replace('16H 多出口藏点与停车朝向','16H 转弯安全窗口与车身碰撞')
s=s.replace('<div class="controls"><button id="hide-jump">',
    '<div class="controls"><button id="risk-jump">跳到预计转弯风险</button><button id="hide-jump">')
start=s.index('  el("intro").textContent=');end=s.index('  const directions=',start)
s=s[:start]+'''  el("intro").textContent="16H：逃跑规划核对制动、旋转、转完后离开路口的时间；避开必须先拐弯才能抵达下一分岔的藏点。保留16G/16F，两格检测、车速、转向时间与车身抓捕判据均保持一致。无安全路线时明确标记保守风险，继续选择可执行退路。";
  const comparison=report.comparison?.summary||[];
  const value=(name,factor)=>comparison.find(r=>r.variant===name&&r.factor===factor)?.mean_s;
  el("comparison").textContent=`转弯检查：先算警方从已检测位置沿地图最快到达的时间，再检查停车旋转以及沿新道路离开前275mm的全过程；转完之后仍按移动中的真实位置估算。
车身：规划按两辆19×19.8cm车的半对角线之和向上取整为275mm，覆盖旋转时扩大的外包络；这是保守规划余量。实际抓捕沿用连续矩形接触判据。
藏点：仅剩 N011、N012、N038、N062。停车轴前后都须先抵达能换线的分岔，途中无强制90度拐弯；选点和朝向由实际到达路线完成。
优先级：有正余量的路线优先于新旧藏点；无正余量时，优先推迟第一次危险停留，再比较曝光和路线质量。6秒之后的信息不足，未标危险不代表安全。
有限观测：只使用自己的地图位置和已检测激光线索；不输入警方全局坐标。移动时激光接收沿用基线关闭方案，途中突遇车辆仍可能碰撞。
${comparison.length?`完整比赛复核：同速 16G ${value("16G",1)?.toFixed(2)}s → 16H ${value("16H",1)?.toFixed(2)}s；警方快20% ${value("16G",1.2)?.toFixed(2)}s → ${value("16H",1.2)?.toFixed(2)}s。均未持宝到限时。同速16F为${value("16F",1)?.toFixed(2)}s，仍保留供对比。`:"最终复核见比较结果文件。"}
前进→倒车仍为0.282秒，倒车→前进0.274秒，转弯额外代价0.768秒。名义尺寸19×19.8cm尚未实测，找线位移、惯性与加速度尚未标定。
同轴近距追击专项故意保留不利停车；若已没有可执行安全退路，模型仍正确判抓捕。`;
''' + s[end:]
anchor='    if(match.geometry_demo)'
idx=s.index(anchor)
s=s[:idx]+'''    if(p?.firmware_version===1608){
      const limit=report.turn_safety.horizon_ms, risk=p.first_pause_risk_ms;
      el("firmware").textContent+=`；转弯/停车窗口：${risk<=limit?`从本次决策起约${(risk/1000).toFixed(3)}秒开始存在保守风险`:`前${limit/1000}秒未标记危险停留`}。超出预测窗口不代表安全；DEGRADED=${Number(p.degraded)}。`;
    }
''' + s[idx:]
anchor='  el("hide-jump").addEventListener'
idx=s.index(anchor)
s=s[:idx]+'''  el("risk-jump").addEventListener("click",()=>{
    const e=(match.planner_events||[]).find(e=>e.kind==="escape"&&e.plan.first_pause_risk_ms<=report.turn_safety.horizon_ms);
    jumpTo(e?e.t+e.plan.first_pause_risk_ms/1000:match.duration_s);
  });
''' + s[idx:]
(HERE/'replay.template.html').write_text(s,encoding='utf-8')
if (HERE/'report.json').exists():
    import json
    data=json.loads((HERE/'report.json').read_text(encoding='utf-8'))
    if (HERE/'heldout_comparison.json').exists():
        data['comparison']=json.loads((HERE/'heldout_comparison.json').read_text(encoding='utf-8'))
    payload=json.dumps(data,ensure_ascii=False,separators=(',',':')).replace('</','<\\/')
    (HERE/'report.json').write_text(json.dumps(data,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
    (HERE/'replay.html').write_text(s.replace('__REPORT_DATA__',payload).replace('__ANNOTATION_SCRIPT__',
        (HERE/'map_annotations.js').read_text(encoding='utf-8')),encoding='utf-8')
print('Prepared 16H turn-window page')

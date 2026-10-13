"""Multi-exit/parking-axis overlay on the preserved 16F car replay page."""
from pathlib import Path
import json
HERE=Path(__file__).resolve().parent
s=(HERE.with_name('16f')/'replay.template.html').read_text(encoding='utf-8')
s=s.replace('16F 小车转向与车身碰撞回放','16G 多出口藏点与停车朝向')
s=s.replace('16E','16G')
s=s.replace('<a id="download-map">','<label><input id="show-hides" type="checkbox" checked>显示可藏点与停车轴</label><a id="download-map">')
s=s.replace('<div id="firmware"','<div id="parking-info" class="map-info"></div><div id="firmware"')
s=s.replace('viewport:el("map-viewport")','viewport:el("map-viewport"),hideCatalog:report.hide_catalog')
start=s.index('  el("intro").textContent=');end=s.index('  const directions=',start)
s=s[:start]+'''  const hides=new Map(report.hide_catalog.map(n=>[n.node,n]));
  el("intro").textContent="独立16G多出口实验：首次藏点固定为四出口N038，后续按激光线索选择至少三条活路的藏点与实际到达朝向。排除死路和单入口口袋区，沿前后方向先离开再换线。完整比赛复核未优于16F，原版保留；朝向对照场景用于观察同一来车下的差异。";
  const comparison=report.comparison?.summary||[];
  const value=(name,factor)=>comparison.find(r=>r.variant===name&&r.factor===factor)?.mean_s;
  el("comparison").textContent=`藏点：绿色圈标出本版候选，仅选真实T/十字路口。出口须通向不同的下一处有效分岔，死支路不计数。
停车：车头与车尾都有真实活路；比较两端换线耗时、前方的分岔耗时和抵达路线风险。由到达路线完成该朝向，不瞬间原地摆车头。
遇车：保留原C激光门控，沿未被已见警方占据的前后方向先离开；选路仍检查已检测威胁，不使用警方全局坐标。
三/四出口仍可能在远处汇合。邻近分岔、曝光距离和到达路线都影响安全，更多出口不保证能逃脱。
地图先验的来向评分试验未同时改善两种速度场景，默认关闭，避免将猜测当作已观测事实。
${comparison.length?`相同车身/判据的复核：同速 16F ${value("16F_baseline",1)?.toFixed(2)} s → 16G ${value("selected",1)?.toFixed(2)} s；警方快20% ${value("16F_baseline",1.2)?.toFixed(2)} s → ${value("selected",1.2)?.toFixed(2)} s。完整比赛各复核3种子，均未小偷获胜；不作为默认替代策略。`:'比较结果见开发记录；尚不声称优于16F。'}
车身名义尺寸19×19.8cm，未实测；转向为原地旋转近似，找线位移和惯性尚未标定。`;
''' + s[end:]
anchor='    const tx=X(5.5),ty=Y(0);'
start=s.index(anchor)
s=s[:start]+'''    if(el("show-hides").checked){
      for(const h of report.hide_catalog){
        const n=report.annotations.nodes.find(n=>Number(n.id.slice(1))===h.node);
        const mark=node("circle",{class:"candidate-hide",cx:X(n.xy_grid[0]),cy:Y(n.xy_grid[1]),r:10,fill:"none",stroke:"#40bb91","stroke-width":1.4,"pointer-events":"none"});
        node("title",{},`${n.id} · ${h.escape_exits}条活路`,mark);
      }
      if(f[16]!==null&&f[18]){
        const [dx,dy]={N:[0,-1],E:[1,0],S:[0,1],W:[-1,0]}[f[18]],x=X(f[16]),y=Y(f[17]),r=.55*k;
        node("line",{id:"parking-axis",x1:x-dx*r,y1:y-dy*r,x2:x+dx*r,y2:y+dy*r,stroke:"#40bb91","stroke-width":2,"stroke-dasharray":"4 3","pointer-events":"none"});
        for(const sign of [-1,1]){
          const cx=x+sign*dx*r,cy=y+sign*dy*r;
          node("polygon",{points:`${cx},${cy} ${cx-sign*dx*8-dy*4},${cy-sign*dy*8+dx*4} ${cx-sign*dx*8+dy*4},${cy-sign*dy*8-dx*4}`,fill:"#40bb91","pointer-events":"none"});
        }
      }
    }
''' + s[start:]
anchor='    slider.value=index;'
extra='''    const goalNode=report.annotations.nodes.find(n=>n.xy_grid[0]===f[16]&&n.xy_grid[1]===f[17]);
    const hide=goalNode?hides.get(Number(goalNode.id.slice(1))):null;
    if(hide&&f[18]){
      const heading=["N","E","S","W"].indexOf(f[18]),forward=hide.exits[String(heading)],rear=hide.exits[String((heading+2)%4)];
      const ref=n=>`N${String(n).padStart(3,"0")}`;
      el("parking-info").textContent=`当前藏点 ${goalNode.id}：${hide.escape_exits}条真实活路；计划车头朝${directions[f[18]]}。前方到下一分岔 ${ref(forward.node)} 约${(forward.time_ms/1000).toFixed(3)}秒，后方到 ${ref(rear.node)} 约${(rear.time_ms/1000).toFixed(3)}秒（规划估计）。绿色双向箭头为停车轴，橙色点线为实际已选路线。抵达后保持该朝向，检出警方才离开。`;
    }else el("parking-info").textContent="尚未选择藏点。绿色圈表示多出口候选，点击节点查看各方向的活路。";
'''
s=s.replace(anchor,extra+anchor)
s=s.replace('  el("show-envelope").addEventListener("change",render);','  el("show-envelope").addEventListener("change",render);\n  el("show-hides").addEventListener("change",render);')
s=s.replace('新藏点仅选真实拐角或路口，至少保留两条环路出口。','新藏点只选真实T/十字路口，至少三条活路；到达朝向由实际路线实现。')
(HERE/'replay.template.html').write_text(s,encoding='utf-8')
if (HERE/'report.json').exists():
    report=json.loads((HERE/'report.json').read_text(encoding='utf-8'))
    payload=json.dumps(report,ensure_ascii=False,separators=(',',':')).replace('</','<\\/')
    (HERE/'replay.html').write_text(s.replace('__REPORT_DATA__',payload).replace('__ANNOTATION_SCRIPT__',
        (HERE/'map_annotations.js').read_text(encoding='utf-8')),encoding='utf-8')
print('Prepared 16G hide-axis page')

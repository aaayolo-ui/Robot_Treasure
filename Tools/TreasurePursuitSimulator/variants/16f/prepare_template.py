"""Extend the preserved 16E page; write only this 16F directory."""
from pathlib import Path
import json,re
HERE=Path(__file__).resolve().parent
s=(HERE.with_name('16e')/'replay.template.html').read_text(encoding='utf-8')
s=s.replace('<title>16E 固件策略回放</title>','<title>16F 小车转向与车身碰撞回放</title>')
s=s.replace('<h1>16E 固件策略回放</h1>','<h1>16F 小车转向与车身碰撞回放</h1>')
s=s.replace('跳到抓捕时刻','跳到车身接触')
s=s.replace('<label>回放速度<select id="speed">','<button id="turn-jump" type="button">跳到首次转向</button><label>回放速度<select id="speed"><option value="0.25">0.25 倍</option><option value="0.5">0.5 倍</option>')
s=s.replace('<a id="download-map">','<label><input id="show-envelope" type="checkbox" checked>显示车身碰撞轮廓</label><a id="download-map">')
s=s.replace('● 警方：蓝色实线','警方：蓝色车身与实线').replace('◆ 小偷：橙色虚线','小偷：橙色车身与虚线')
s=s.replace('候选策略分类与对比','转向与碰撞模型说明')
s=s.replace('未模拟循迹误差、掉宝、身体转动包络及实际传感器误差分布。运行 variants/16e/simulate_16e.py 生成本页。',
    '车身尺寸、转向角速度曲线尚未实测；目前车身中心在路口原地转向，找线前移、惯性滑行、打滑与循迹误差未建模。检测仍沿用16E的地图轴向理想光束，转弯时真实光束待标定。本版是运动学近似，不是实车精确动力学。')
s=s.replace('橙色圆环标记藏身目标','橙色圆环标记藏身目标').replace('地图黑线是道路。','地图黑线是道路，不作为实体墙。车身和虚线轮廓按实际地图比例绘制；白色箭头指向车头，倒车时车头不翻转。')
s=s.replace('"转向等待"','"连续转向"')
start=s.index('  el("intro").textContent=')
end=s.index('  const directions=',start)
s=s[:start]+'''  const model=report.body_model, bodies=model.bodies;
  el("intro").textContent="两车采用19×19.8厘米名义外包络：连续显示转向，前进/倒车保持真实车头方向，按旋转矩形的首次接触判抓捕。16E策略、路线、两格检测和不同颜色轨迹保留。最后一个场景仅验证转弯扫角，不计比赛结果。";
  el("comparison").textContent=`车身：警/偷各 ${bodies.P.length_m*100}×${bodies.P.width_m*100} cm；${model.dimensions_status}。
转向：先制动等待，再用连续角度曲线旋转到目标航向；45°是控制阶段标记。转向中心固定在地图路口。
时间：沿用16E的0.768秒额外转弯预算，并未把它宣称为实测纯90°旋转时间。制动占比${(model.turn.brake_fraction*100).toFixed(0)}%与角速度曲线是可调近似。
判抓捕：有朝向的两车矩形外包络接触即结束；连续检查位移和车角扫过，不采用旧0.18米中心距离阈值。绘图笔画宽度不参与计算。
数值接触容差：${model.collision.contact_tolerance_m*1000} mm，不是实车测量精度。抓捕后冻结两车，红点标出接触边界。
尚未模拟：找线前移、真实制动滑行、轮胎打滑、安装偏置、转弯时真实激光光束。数据由同一辆车的旧全图记录提供，另一辆车未单独标定。
尺寸改动必须重新运行simulate_16f.py，确保画面和裁判采用同一尺寸。`;
''' + s[end:]
s=s.replace('function node(tag,attrs={},text){','function node(tag,attrs={},text,parent=svg){').replace('svg.append(n);return n;','parent.append(n);return n;')
s=s.replace('r:15,fill:"none",stroke:"var(--red)"','r:Math.hypot(bodies.B.length_m,bodies.B.width_m)/2/report.map.grid_spacing_m*k+5,fill:"none",stroke:"var(--red)"')
anchor='    annotations.draw({node,X,Y,k,width,height,vehicles:[[f[2],f[3]],[f[9],f[10]]]});'
replacement='''    const bodyPolygon=(off,angle,body)=>{
      const cx=f[off],cy=f[off+1],c=Math.cos(angle),s=Math.sin(angle),space=report.map.grid_spacing_m;
      return [[body.length_m/2,body.width_m/2],[-body.length_m/2,body.width_m/2],[-body.length_m/2,-body.width_m/2],[body.length_m/2,-body.width_m/2]].map(([u,v])=>[X(cx+(u*c-v*s)/space),Y(cy+(u*s+v*c)/space)]);
    };
    const vehiclePolygons=[bodyPolygon(2,f[33],bodies.P),bodyPolygon(9,f[34],bodies.B)];
    const vehicleBounds=vehiclePolygons.map(points=>{const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);return {x:Math.min(...xs)-5,y:Math.min(...ys)-5,w:Math.max(...xs)-Math.min(...xs)+10,h:Math.max(...ys)-Math.min(...ys)+10};});
    annotations.draw({node,X,Y,k,width,height,vehicles:[[f[2],f[3]],[f[9],f[10]]],vehicleBounds});'''
assert anchor in s
s=s.replace(anchor,replacement)
start=s.index('    for(const [off,label,color]')
end=s.index('    const ended=',start)
s=s[:start]+'''    for(const [j,off,label,color,body] of [[0,2,"警","var(--blue)",bodies.P],[1,9,"偷","var(--red)",bodies.B]]){
      const x=X(f[off]),y=Y(f[off+1]),theta=f[33+j],l=body.length_m/report.map.grid_spacing_m*k,w=body.width_m/report.map.grid_spacing_m*k;
      const g=node("g",{id:j===0?"police-body":"thief-body",class:"car-body",transform:`translate(${x} ${y}) rotate(${theta*180/Math.PI})`,"data-angle-rad":theta,"data-length-m":body.length_m,"data-width-m":body.width_m,"pointer-events":"none","aria-label":`${label}车车身，朝向${(theta*180/Math.PI).toFixed(1)}度`});
      for(const xx of [-.29*l,.29*l])for(const yy of [-.43*w,.43*w])
        node("rect",{x:xx-.16*l,y:yy-.06*w,width:.32*l,height:.12*w,rx:2,fill:"#263442",stroke:color,"stroke-width":1},undefined,g);
      node("rect",{x:-.43*l,y:-.35*w,width:.86*l,height:.7*w,rx:4,fill:color,stroke:"var(--panel)","stroke-width":1.3},undefined,g);
      node("path",{class:"car-front",d:`M${.4*l},0 L${.17*l},${-.12*w} L${.17*l},${.12*w}Z`,fill:"#fff"},undefined,g);
      node("line",{x1:-.37*l,x2:-.37*l,y1:-.24*w,y2:.24*w,stroke:"#94333e","stroke-width":3},undefined,g);
      node("text",{x:-.1*l,y:4,"text-anchor":"middle",fill:"#fff","font-size":12,transform:`rotate(${-theta*180/Math.PI} ${-.1*l} 0)`},label,g);
      if(f[1]===(j===0?1:2))node("rect",{x:-.08*l,y:-.29*w,width:.14*l,height:.13*w,fill:"var(--gold)",stroke:"var(--panel)"},undefined,g);
      if(el("show-envelope").checked)node("polygon",{id:j===0?"police-envelope":"thief-envelope",class:"body-envelope",points:vehiclePolygons[j].map(p=>p.join(",")).join(" "),fill:"none",stroke:color,"stroke-width":1.2,"stroke-dasharray":"3 2","pointer-events":"none"});
    }
''' + s[end:]
start=s.index('    if(captured){node("circle"')
end=s.index('    el("status").replaceChildren()',start)
s=s[:start]+'''    if(captured&&match.collision){
      const point=match.collision.contact_point_m,x=X(point[0]/report.map.grid_spacing_m),y=Y(point[1]/report.map.grid_spacing_m);
      node("circle",{id:"contact-marker",cx:x,cy:y,r:6,fill:"#ef405b",stroke:"#fff","stroke-width":2,"pointer-events":"none"});
      node("path",{d:`M${x-10},${y}H${x+10}M${x},${y-10}V${y+10}`,stroke:"#ef405b","stroke-width":1.3,"pointer-events":"none"});
    }
''' + s[end:]
s=s.replace('达到0.18米接触阈值，立即判警方获胜。','两车外包络首次接触，立即结束并判警方获胜。')
anchor='    el("parameters").textContent='
start=s.index(anchor);end=s.index('\n    if(f[6]',start)
s=s[:start]+'''    const stages={brake:"制动等待",pivot:"差速旋转",align:"航向收敛（找线位移未标定）",switch:"刹车后换向",forward:"前进",reverse_move:"倒车",hold:"停车"};
    el("detail").textContent+=`车身动作：警方${stages[f[35]]}，小偷${stages[f[36]]}；当前外包络分离量 ${(f[37]*1000).toFixed(2)} mm。${match.geometry_demo?"这是模型碰撞专项，不运行比赛策略。":""}`;
    el("parameters").textContent=`两车长${bodies.P.length_m*100}、宽${bodies.P.width_m*100} cm，未实测外包络；警方 ${fmt(match.police_speed_m_s)} m/s，小偷 ${fmt(match.thief_speed_m_s)} m/s；${motionText(match)}。两格0.6米、20ms有限检测；小偷行驶时关闭检测，停稳后读取实际16E C门控。转向为路口原地差速旋转近似，找线前移与真实惯性未标定。尺寸与碰撞计算采用同一配置。`;
''' + s[end:]
s=s.replace('    const decision=(match.planner_events','    const decision=(match.planner_events')
anchor='    slider.value=index;'
s=s.replace(anchor,'    if(match.geometry_demo)el("firmware").textContent="仅模型验证：旁车中心静止，转向车辆的车角扫过其外包络；展示连续碰撞检查，不作为策略比赛。";\n'+anchor)
s=s.replace('  el("hide-jump").addEventListener(','''  el("turn-jump").addEventListener("click",()=>{const f=match.frames.find(f=>["brake","pivot","align"].includes(f[35])||["brake","pivot","align"].includes(f[36]));jumpTo(f?.[0]??0);});
  el("show-envelope").addEventListener("change",render);
  el("hide-jump").addEventListener(''')
# Contact marker and outer rectangles are supplementary overlays; all map
# identifiers/export JSON still come from the original stable annotation data.
(HERE/'replay.template.html').write_text(s,encoding='utf-8')
if (HERE/'report.json').exists():
    report=json.loads((HERE/'report.json').read_text(encoding='utf-8'))
    payload=json.dumps(report,ensure_ascii=False,separators=(',',':')).replace('</','<\\/')
    html=s.replace('__REPORT_DATA__',payload).replace('__ANNOTATION_SCRIPT__',(HERE/'map_annotations.js').read_text(encoding='utf-8'))
    (HERE/'replay.html').write_text(html,encoding='utf-8')
print('16F body page prepared')

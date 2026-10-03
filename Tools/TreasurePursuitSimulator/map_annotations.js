/* Map-only overlay. It never changes simulator state, connectivity or strategy. */
function createMapAnnotations(data, elements, requestRender, pauseReplay) {
  "use strict";
  const { nodesSwitch, segmentsSwitch, legend, info, picker, download, viewport } = elements;
  const nodes = new Map(data.nodes.map(n => [n.id, n]));
  const segments = new Map(data.segments.map(s => [s.id, s]));
  const dirs = {N:"北 ↑", E:"东 →", S:"南 ↓", W:"西 ←"};
  const styles = {
    corner: ["#9360c8", "triangle", "△ 直角拐弯"],
    t_junction: ["#168577", "square", "□ T 型路口"],
    cross: ["#cb468e", "cross", "＋ 十字路口"],
    endpoint: ["#738391", "circle", "○ 死路端点"],
    police_spawn: ["#268fe8", "pentagon", "⬟ 警方出生点"],
    thief_spawn: ["#db7821", "hexagon", "⬡ 小偷出生点"],
    treasure: ["#aa8100", "star", "★ 宝藏点"]
  };
  let selected = null;
  for (const [color, , label] of Object.values(styles)) {
    const span = document.createElement("span");
    span.style.color = color; span.textContent = label; legend.append(span);
  }
  for (const n of data.nodes) {
    const option = document.createElement("option");
    option.value = n.id; option.textContent = `${n.id} · ${n.labels.join(" / ")}`; picker.append(option);
  }
  // Serve the generated file over HTTP; keep standalone HTML export offline.
  download.href = /^https?:$/.test(location.protocol) ? "map_annotations.json" :
    URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], {type:"application/json"}));
  download.download = "map_annotations.json";
  function switchesChanged() {
    viewport.classList.toggle("annotated", nodesSwitch.checked || segmentsSwitch.checked);
    legend.hidden = !nodesSwitch.checked;
    requestRender();
  }
  nodesSwitch.addEventListener("change", switchesChanged);
  segmentsSwitch.addEventListener("change", switchesChanged);
  function addText(tag, text) { const e=document.createElement(tag); e.textContent=text; info.append(e); return e; }
  function select(kind, id) {
    pauseReplay(); selected={kind,id}; info.replaceChildren();
    if(kind === "node") {
      const n=nodes.get(id); picker.value=id;
      addText("h3", `${id} · ${n.labels.join(" / ")}`);
      addText("p", `地图坐标 (${n.xy_grid.join(", ")}) 格；推算坐标 (${n.xy_m.join(", ")}) 米。原地图节点 ${n.original_node}。`);
      if(!n.is_track_landmark) addText("p", "任务标记位于直道上，不是新增灰度路口；到达位置需要路段里程或其他定位依据。");
      const list=addText("ul", "");
      for(const c of n.connections) {
        const item=document.createElement("li"), button=document.createElement("button");
        button.type="button"; button.textContent=`${c.segment} → ${c.neighbor} · ${dirs[c.direction]} · ${c.length_m.toFixed(2)} 米`;
        button.addEventListener("click", ()=>select("segment",c.segment)); item.append(button); list.append(item);
      }
    } else {
      const s=segments.get(id); picker.value="";
      addText("h3", `${s.id} · ${s.start} ↔ ${s.end}`);
      addText("p", `长度 ${s.length_m.toFixed(2)} 米；${s.start} → ${s.end}：${dirs[s.direction_from_start]}；反向：${dirs[s.direction_from_end]}。`);
      addText("p", `原地图对应 ${s.original_nodes.join(" → ")}。中间的普通网格点是路段里程位置，不额外当作灰度路口。`);
      for(const id of [s.start,s.end]) {
        const b=addText("button", `查看 ${id}`); b.type="button"; b.addEventListener("click",()=>select("node",id));
      }
    }
    addText("p", data.length_note + "。" + data.localization_note);
    requestRender();
  }
  picker.addEventListener("change", ()=>{if(picker.value) {nodesSwitch.checked=true; switchesChanged(); select("node",picker.value);}});
  function interactive(shape, kind, id, label) {
    shape.setAttribute("role", "button"); shape.setAttribute("tabindex", "0");
    shape.setAttribute("aria-label", label); shape.style.cursor="pointer";
    const tooltip=document.createElementNS("http://www.w3.org/2000/svg","title");
    tooltip.textContent=label; shape.append(tooltip);
    shape.addEventListener("click", ()=>select(kind,id));
    shape.addEventListener("keydown", e=>{if(e.key==="Enter" || e.key===" "){e.preventDefault(); select(kind,id);}});
  }
  function draw({node,X,Y,k,width,height,vehicles}) {
    const occupied = vehicles.map(([x,y])=>({x:X(x)-11,y:Y(y)-23,w:38,h:37}));
    for(const n of data.nodes) occupied.push({x:X(n.xy_grid[0])-7,y:Y(n.xy_grid[1])-7,w:14,h:14});
    // Label boxes are displaced from intersections, routes, cars and each other.
    const overlap=(a,b)=>a.x < b.x+b.w+2 && a.x+a.w+2>b.x && a.y<b.y+b.h+2 && a.y+a.h+2>b.y;
    function label(kind,id,x,y,text,color,candidates) {
      const w=text.length*6.2+6,h=16;
      let box;
      const extra=[[36,-10],[-68,-10],[36,24],[-68,24],[-16,-42],[10,-42],[-42,-42],[10,40],[-42,40]];
      for(const [dx,dy] of [...candidates,...extra]) {
        const candidate={x:x+dx,y:y+dy-12,w,h};
        if(candidate.x<2 || candidate.y<22 || candidate.x+w>width-2 || candidate.y+h>height-2) continue;
        if(!occupied.some(other=>overlap(candidate,other))) {box=candidate; break;}
      }
      if(!box) return; // Exact info remains available through the marker/picker.
      occupied.push(box);
      const closestX=Math.max(box.x,Math.min(box.x+w,x)),closestY=Math.max(box.y,Math.min(box.y+h,y));
      if(Math.hypot(closestX-x,closestY-y)>20) node("line",{x1:x,y1:y,x2:closestX,y2:closestY,stroke:color,"stroke-width":.7,opacity:.6,"pointer-events":"none"});
      const rect=node("rect", {x:box.x,y:box.y,width:w,height:h,rx:3,fill:"var(--panel)",opacity:.94,"pointer-events":"none"});
      rect.setAttribute("class", "annotation-label-bg");
      const textNode=node("text", {x:box.x+3,y:box.y+12,fill:color,"font-size":11,"font-family":"system-ui, sans-serif","class":"annotation-label","data-label-for":id}, text);
      interactive(textNode,kind,id,`查看 ${id}`);
    }
    if(segmentsSwitch.checked) for(const s of data.segments) {
      const a=nodes.get(s.start).xy_grid,b=nodes.get(s.end).xy_grid;
      const active=selected?.kind==="segment" && selected.id===s.id;
      if(active) node("line", {x1:X(a[0]),y1:Y(a[1]),x2:X(b[0]),y2:Y(b[1]),stroke:"#a566cc","stroke-width":6,"pointer-events":"none"});
      const vertical=a[0]===b[0];
      const hit=node("rect", {x:Math.min(X(a[0]),X(b[0]))-(vertical?7:0),y:Math.min(Y(a[1]),Y(b[1]))-(vertical?0:7),
        width:Math.max(14,Math.abs(X(a[0])-X(b[0]))),height:Math.max(14,Math.abs(Y(a[1])-Y(b[1]))),
        fill:"transparent","pointer-events":"all","class":"annotation-segment","data-segment-id":s.id});
      interactive(hit,"segment",s.id,`${s.id} ${s.start} 至 ${s.end}，${s.length_m.toFixed(2)} 米，${dirs[s.direction_from_start]}`);
    }
    if(nodesSwitch.checked) for(const n of data.nodes) {
      const x=X(n.xy_grid[0]),y=Y(n.xy_grid[1]),[color,shape]=styles[n.type],r=5;
      const attr={fill:"var(--panel)",stroke:color,"stroke-width":2,"class":"annotation-node","data-node-id":n.id};
      let marker;
      if(shape==="circle") marker=node("circle",{...attr,cx:x,cy:y,r});
      else if(shape==="square") marker=node("rect",{...attr,x:x-r,y:y-r,width:r*2,height:r*2});
      else if(shape==="cross") marker=node("path",{...attr,d:`M${x-6},${y}H${x+6}M${x},${y-6}V${y+6}`,fill:"none","stroke-width":3});
      else {
        const count=shape==="triangle"?3:shape==="pentagon"?5:shape==="hexagon"?6:10;
        const points=Array.from({length:count},(_,i)=>{const radius=shape==="star"&&i%2?3:6, a=i*Math.PI*2/count-Math.PI/2;return `${x+Math.cos(a)*radius},${y+Math.sin(a)*radius}`;}).join(" ");
        marker=node("polygon",{...attr,points});
      }
      interactive(marker,"node",n.id,`${n.id} ${n.labels.join("、")}，地图坐标 ${n.xy_grid.join(",")}`);
      if(selected?.kind==="node" && selected.id===n.id) node("circle",{cx:x,cy:y,r:11,fill:"none",stroke:color,"stroke-width":2,"pointer-events":"none"});
      // Expand the tap target without covering neighbouring segment midpoints.
      const hit=node("circle",{cx:x,cy:y,r:10,fill:"transparent","data-node-hit":n.id});
      interactive(hit,"node",n.id,`选择节点 ${n.id}`);
    }
    if(nodesSwitch.checked) for(const n of data.nodes) {
      const x=X(n.xy_grid[0]),y=Y(n.xy_grid[1]);
      label("node",n.id,x,y,n.id,styles[n.type][0],[[10,-9],[-42,-9],[10,24],[-42,24],[-16,-25],[-16,38],[14,7],[-48,7]]);
    }
    if(segmentsSwitch.checked) for(const s of data.segments) {
      const a=nodes.get(s.start).xy_grid,b=nodes.get(s.end).xy_grid,x=X((a[0]+b[0])/2),y=Y((a[1]+b[1])/2);
      const text=s.length_m/data.grid_spacing_m*k>=105?`${s.id} ${s.length_m.toFixed(2)}m`:s.id;
      label("segment",s.id,x,y,text,"var(--muted)",[[8,-5],[8,17],[-38,-8],[-38,25]]);
    }
  }
  viewport.classList.toggle("annotated", nodesSwitch.checked || segmentsSwitch.checked);
  legend.hidden=!nodesSwitch.checked;
  return {draw, reference(original) {
    const ref=data.original_node_mapping[original];
    if(!ref) return original;
    if(ref.node_id) return ref.node_id;
    const s=ref.segments[0];return `${s.segment_id}，距 ${segments.get(s.segment_id).start} ${s.distance_from_start_m.toFixed(2)} 米`;
  }};
}

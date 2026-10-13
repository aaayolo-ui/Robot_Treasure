"""Offline replay keyframes, not browser screenshots or hardware photographs."""
from pathlib import Path
import json,math
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parent
report=json.loads((ROOT/'report.json').read_text(encoding='utf-8'))
font_path='C:/Windows/Fonts/msyh.ttc'
def font(n):return ImageFont.truetype(font_path,n)
def draw_frame(match,t,name):
    f=next((f for f in match['frames'] if f[0]>=t-1e-6),match['frames'][-1])
    image=Image.new('RGB',(1100,990),'#111820');d=ImageDraw.Draw(image)
    X=lambda x:55+x*86
    Y=lambda y:150+y*86
    d.text((30,18),match['label'],font=font(25),fill='#e5eaf0')
    d.text((30,56),f"{f[0]:.2f}秒 · 离线关键帧 · 规划来自16D实际C代码",font=font(17),fill='#b6c4d2')
    d.text((30,85),'● 警方蓝色实线   ◆ 小偷橙色虚线   ☆ 宝藏   ○ 目标藏点',font=font(16),fill='#b6c4d2')
    for y,a,b in report['map']['horizontal_segments']:d.line((X(a),Y(y),X(b),Y(y)),fill='#485765',width=3)
    for x,a,b in report['map']['vertical_segments']:d.line((X(x),Y(a),X(x),Y(b)),fill='#485765',width=3)
    trace=[q for q in match['frames'] if q[0]<=f[0]]
    for off,color,width in [(2,'#62b5ff',5),(9,'#ffad4a',3)]:
        points=[(X(q[off]),Y(q[off+1])) for q in trace]
        if len(points)>1:
            for a,b in zip(points,points[1:]):
                if off==2:d.line([a,b],fill=color,width=width)
                else:
                    length=math.dist(a,b)
                    if not length:continue
                    for s in range(0,max(1,int(length)),12):
                        v=min(length,s+7)
                        d.line([(a[0]+(b[0]-a[0])*s/length,a[1]+(b[1]-a[1])*s/length),
                                (a[0]+(b[0]-a[0])*v/length,a[1]+(b[1]-a[1])*v/length)],fill=color,width=width)
    for node in report['annotations']['nodes']:
        x,y=map(float,node['xy_grid']);px,py=X(x),Y(y);typ=node['geometry_type']
        color={'corner':'#b27de5','t_junction':'#2ac0ad','cross':'#e770af','endpoint':'#8796a3'}.get(typ,'#d7bd50')
        if typ=='corner':d.polygon([(px,py-5),(px-5,py+4),(px+5,py+4)],outline=color)
        elif typ=='t_junction':d.rectangle((px-4,py-4,px+4,py+4),outline=color,width=2)
        elif typ=='cross':d.line((px-5,py,px+5,py),fill=color,width=2);d.line((px,py-5,px,py+5),fill=color,width=2)
        else:d.ellipse((px-4,py-4,px+4,py+4),outline=color,width=2)
        d.text((px+7,py-21 if y!=0 else py-30),node['id'],font=font(12),fill=color)
    tx,ty=report['map']['special_nodes']['T']['xy'];d.rectangle((X(tx)-5,Y(ty)-5,X(tx)+5,Y(ty)+5),fill='#e0b94c')
    if f[16] is not None:d.ellipse((X(f[16])-15,Y(f[17])-15,X(f[16])+15,Y(f[17])+15),outline='#ffad4a',width=2)
    for off,color in [(2,'#62b5ff'),(9,'#ffad4a')]:
        x,y=X(f[off]),Y(f[off+1])
        if off==2:d.ellipse((x-9,y-9,x+9,y+9),fill=color)
        else:d.polygon([(x,y-11),(x+11,y),(x,y+11),(x-11,y)],fill=color)
    decision=next((e for e in reversed(match['planner_events']) if e['t']<=f[0]),None)
    if decision:
        p=decision.get('plan')
        text=(f"目标 {decision['goal']} · 展开 {p['states']}/704状态 · {'完整路线仍有风险' if p['degraded'] else '正常筛选'}" if p else
              'ESCAPE_PLAN_FAILED：没有可接受的逃跑首段，保持停车')
        d.rectangle((20,930,1080,979),fill='#1b2835')
        d.text((30,940),text,font=font(18),fill='#f1c48a')
    image.save(ROOT/name)
draw_frame(report['replays'][0],12.2,'preview_hide.png')
draw_frame(report['replays'][0],16.84,'preview_fault.png')
draw_frame(report['replays'][3],6.0,'preview_escape.png')

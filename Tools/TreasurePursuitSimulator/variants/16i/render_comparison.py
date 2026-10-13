"""Diagram from frozen report poses, not a browser screenshot."""
from pathlib import Path
import json,math
from PIL import Image,ImageDraw
from render_preview import car,font,BG,FG,MUTED,ROAD,P,B,HIT,SPACE,R,HERE
old=json.loads((HERE.with_name('16h')/'report.json').read_text(encoding='utf-8'))['replays'][1]
new=R['replays'][1];time=old['duration_s']
frames=[old['frames'][-1],min(new['frames'],key=lambda f:abs(f[0]-time))]
im=Image.new('RGB',(1150,760),BG);d=ImageDraw.Draw(im)
d.text((28,20),'算法选路对照 · 警方快20% · 近似相同时刻',font=font(27),fill=FG)
d.text((28,65),f'原16H在 {time:.6f} 秒接触；16I约在这一时刻已到另一个藏点等待。',font=font(18),fill=MUTED)
for j,(match,f) in enumerate(zip((old,new),frames)):
    layer=Image.new('RGB',(540,550),BG);draw=ImageDraw.Draw(layer)
    xmin=min(f[2],f[9])-1.;xmax=max(f[2],f[9])+1.
    ymin=min(f[3],f[10])-1.;ymax=max(f[3],f[10])+1.
    k=min(475/(xmax-xmin),390/(ymax-ymin))
    X=lambda x:30+(x-xmin)*k;Y=lambda y:105+(y-ymin)*k
    for y,a,b in R['map']['horizontal_segments']:draw.line((X(a),Y(y),X(b),Y(y)),fill=ROAD,width=3)
    for x,a,b in R['map']['vertical_segments']:draw.line((X(x),Y(a),X(x),Y(b)),fill=ROAD,width=3)
    for n in R['annotations']['nodes']:
        x,y=n['xy_grid'];px,py=X(x),Y(y)
        if not 12<px<516 or not 92<py<490:continue
        draw.rectangle((px-4,py-4,px+4,py+4),outline='#40bb91',width=2)
        draw.text((px+8 if px<460 else px-52,py+15),n['id'],font=font(14),fill='#40bb91')
    for role,off,ang,color,title in (('P',2,33,P,'警'),('B',9,34,B,'偷')):
        car(draw,(X(f[off]),Y(f[off+1])),f[ang],R['body_model']['bodies'][role],color,title,k/SPACE)
    if j==0:
        c=old['collision']['contact_point_m'];cx,cy=X(c[0]/SPACE),Y(c[1]/SPACE)
        draw.ellipse((cx-5,cy-5,cx+5,cy+5),fill=HIT,outline='white',width=2)
    draw.rectangle((0,0,540,87),fill=BG)
    draw.text((15,13),('16H：转移途中车身接触' if j==0 else '16I：推演后选择另一条退路'),font=font(20),fill=FG)
    draw.text((15,50),f"t={f[0]:.3f}s · 小偷阶段：{f[36]}",font=font(17),fill=MUTED)
    draw.rectangle((0,498,540,550),fill=BG)
    draw.text((15,515),('红点为车身首次接触，立即结束' if j==0 else f"仍未接触；后来在 {new['duration_s']:.3f}s 被抓"),font=font(17),fill=FG)
    im.paste(layer,(24+j*562,118));d.rounded_rectangle((24+j*562,118,564+j*562,668),radius=9,outline='#334455',width=2)
d.text((28,699),'两图缩放不同，车身均按各图道路比例绘制；使用相同的尺寸、车速和接触判据。',font=font(17),fill=MUTED)
d.text((28,731),'报告数据绘制的静态示意图，非浏览器截图，非实车验证。',font=font(16),fill=MUTED)
im.save(HERE/'rollout_comparison.png')
print('Saved rollout_comparison.png')

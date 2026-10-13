"""Pillow keyframes from the actual report poses, not browser screenshots."""
from pathlib import Path
import json,math
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent
R=json.loads((HERE/'report.json').read_text(encoding='utf-8'))
SPACE=R['map']['grid_spacing_m']
FONT='C:/Windows/Fonts/msyh.ttc'
BG='#111820';ROAD='#536170';FG='#e5eaf0';MUTED='#afbcc9';P='#62b5ff';B='#ffad4a';HIT='#ff405b'
def font(size):return ImageFont.truetype(FONT,size)


def poly(center,theta,l,w):
    c,s=math.cos(theta),math.sin(theta)
    return [(center[0]+u*c-v*s,center[1]+u*s+v*c) for u,v in ((l/2,w/2),(-l/2,w/2),(-l/2,-w/2),(l/2,-w/2))]


def car(draw,center,theta,body,color,label,scale):
    l,w=body['length_m']*scale,body['width_m']*scale
    shape=poly(center,theta,l,w)
    draw.polygon(shape,fill=BG,outline=color,width=2)
    c,s=math.cos(theta),math.sin(theta)
    T=lambda u,v:(center[0]+u*c-v*s,center[1]+u*s+v*c)
    for xx in (-.29*l,.29*l):
        for yy in (-.43*w,.43*w):
            points=[T(u,v) for u,v in ((xx-.16*l,yy-.06*w),(xx+.16*l,yy-.06*w),(xx+.16*l,yy+.06*w),(xx-.16*l,yy+.06*w))]
            draw.polygon(points,fill='#263442',outline=color)
    draw.polygon(poly(center,theta,.86*l,.70*w),fill=color,outline=FG)
    draw.polygon([T(.4*l,0),T(.17*l,-.12*w),T(.17*l,.12*w)],fill='white')
    draw.line([T(-.37*l,-.24*w),T(-.37*l,.24*w)],fill='#94333e',width=3)
    x,y=T(-.1*l,0)
    draw.text((x,y),label,font=font(15),fill='white',anchor='mm')


def map_preview():
    match=R['replays'][1]
    f=match['frames'][-1]
    im=Image.new('RGB',(1150,1130),BG);d=ImageDraw.Draw(im)
    k=88.;X=lambda x:72+x*k;Y=lambda y:145+y*k
    d.text((40,24),'16F · 连续车身转向 / 外包络首次接触',font=font(28),fill=FG)
    d.text((40,66),'蓝色警方、橙色小偷；名义尺寸 19 × 19.8 cm，尚未实测',font=font(19),fill=MUTED)
    d.text((40,100),f"警方快20% · {match['duration_s']:.6f} s 车身接触即结束",font=font(17),fill=FG)
    for y,a,b in R['map']['horizontal_segments']:d.line((X(a),Y(y),X(b),Y(y)),fill=ROAD,width=3)
    for x,a,b in R['map']['vertical_segments']:d.line((X(x),Y(a),X(x),Y(b)),fill=ROAD,width=3)
    for off,color,width in ((2,P,5),(9,B,3)):
        points=[(X(frame[off]),Y(frame[off+1])) for frame in match['frames']]
        if off==2:d.line(points,fill=color,width=width)
        else:
            # Dashed trail sampled by pixel distance, independent of phase.
            covered=0.
            for a,b in zip(points,points[1:]):
                length=math.dist(a,b)
                if int(covered/9)%2==0:d.line((a,b),fill=color,width=width)
                covered+=length
    bodies=R['body_model']['bodies']
    vehicle_bounds=[]
    for role,off,ang in (('P',2,33),('B',9,34)):
        p=poly((X(f[off]),Y(f[off+1])),f[ang],bodies[role]['length_m']/SPACE*k,bodies[role]['width_m']/SPACE*k)
        vehicle_bounds.append((min(x for x,y in p)-8,min(y for x,y in p)-8,max(x for x,y in p)+8,max(y for x,y in p)+8))
    occupied=list(vehicle_bounds)
    occupied += [(X(n['xy_grid'][0])-6,Y(n['xy_grid'][1])-6,X(n['xy_grid'][0])+6,Y(n['xy_grid'][1])+6) for n in R['annotations']['nodes']]
    for n in R['annotations']['nodes']:
        x,y=map(lambda v:v,n['xy_grid']);px,py=X(x),Y(y)
        kind=n['type']
        color={'corner':'#9360c8','t_junction':'#168577','cross':'#cb468e','endpoint':'#738391','treasure':'#e0b94c','thief_spawn':B,'police_spawn':P}[kind]
        if kind=='corner':d.polygon([(px,py-6),(px+5,py+4),(px-5,py+4)],outline=color,width=2)
        elif kind=='endpoint':d.ellipse((px-4,py-4,px+4,py+4),outline=color,width=2)
        elif kind=='cross':
            d.line((px-5,py,px+5,py),fill=color,width=2);d.line((px,py-5,px,py+5),fill=color,width=2)
        elif kind in ('treasure','thief_spawn','police_spawn'):
            sides=5 if kind=='police_spawn' else 6
            d.polygon([(px+6*math.cos(a*2*math.pi/sides),py+6*math.sin(a*2*math.pi/sides)) for a in range(sides)],outline=color,width=2)
        else:d.rectangle((px-4,py-4,px+4,py+4),outline=color,width=2)
        for dx,dy in ((8,-22),(-44,-22),(8,9),(-44,9),(14,-40),(-46,-40),(14,30),(-46,30)):
            tx,ty=px+dx,py+dy;box=(tx-1,ty-1,tx+39,ty+18)
            if not any(box[0]<b[2]+2 and box[2]+2>b[0] and box[1]<b[3]+2 and box[3]+2>b[1] for b in occupied):
                occupied.append(box);d.text((tx,ty),n['id'],font=font(12),fill=color);break
    for role,off,ang,color,label in (('P',2,33,P,'警'),('B',9,34,B,'偷')):
        car(d,(X(f[off]),Y(f[off+1])),f[ang],bodies[role],color,label,k/SPACE)
    contact=match['collision']['contact_point_m'];cx,cy=X(contact[0]/SPACE),Y(contact[1]/SPACE)
    d.ellipse((cx-5,cy-5,cx+5,cy+5),fill=HIT,outline='white',width=2)
    d.text((40,1064),'红点为两车外包络接触位置；检测扇形未在此静态图中绘制。',font=font(17),fill=MUTED)
    d.text((40,1092),'由回放数据绘制的静态预览，非浏览器截图；原地转向中心、角速度曲线未实车标定。',font=font(15),fill=MUTED)
    im.save(HERE/'preview_map.png')


def turn_preview():
    match=R['replays'][-1]
    frames=match['frames']
    selected=[frames[0],min(frames,key=lambda f:abs(f[0]-.34)),frames[-1]]
    im=Image.new('RGB',(1500,625),BG);d=ImageDraw.Draw(im)
    d.text((32,22),'转向扫角验证：中心不动，车角也可能碰到旁车',font=font(28),fill=FG)
    d.text((32,64),'模型专项，不计比赛结果；两车原点相距 22.8 cm。',font=font(19),fill=MUTED)
    for j,(label,f) in enumerate(zip(('转向前：没有接触','旋转中：车头连续变化','首次接触：立即结束'),selected)):
        left=30+j*492;top=115
        d.rounded_rectangle((left,top,left+470,top+399),radius=12,outline='#334455',width=2)
        d.text((left+18,top+16),label,font=font(22),fill=FG)
        scale=900;origin=(left+128,top+227)
        bxy=(f[9]*SPACE,f[10]*SPACE)
        T=lambda off:(origin[0]+(f[off]*SPACE-bxy[0])*scale,origin[1]+(f[off+1]*SPACE-bxy[1])*scale)
        d.line((left+16,origin[1],left+454,origin[1]),fill=ROAD,width=3)
        for role,off,ang,color,title in (('B',9,34,B,'偷'),('P',2,33,P,'警')):
            car(d,T(off),f[ang],R['body_model']['bodies'][role],color,title,scale)
        d.text((left+18,top+341),f"t = {f[0]:.6f} s · 车头 {f[34]*180/math.pi:.1f}°",font=font(18),fill=MUTED)
        if j==2:
            q=match['collision']['contact_point_m'];cx,cy=origin[0]+(q[0]-bxy[0])*scale,origin[1]+(q[1]-bxy[1])*scale
            d.ellipse((cx-7,cy-7,cx+7,cy+7),fill=HIT,outline='white',width=2)
    d.text((32,539),'白箭头指向车头，红边为车尾；细外框就是碰撞判定使用的矩形包络。',font=font(20),fill=FG)
    d.text((32,579),'静态帧由report.json绘制，非浏览器截图。完整回放含0.25倍慢放、转向跳转和接触跳转。',font=font(17),fill=MUTED)
    im.save(HERE/'preview_turn_contact.png')


if __name__=='__main__':
    map_preview();turn_preview();print('Saved map and rotating-contact keyframes')

from pathlib import Path
import re,json,hashlib,math,textwrap
from datetime import datetime
from docx import Document
from docx.shared import Inches,Pt,RGBColor
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.enum.text import WD_ALIGN_PARAGRAPH
from PIL import Image,ImageDraw,ImageFont

OUT=Path(__file__).resolve().parent
ROOT=OUT.parent
P=ROOT/'Robot_Treasure-main/STM32/Keil_Project/Thief16V_UARTCh4_RouteOnly_Test'
OUT.mkdir(exist_ok=True)
QA=OUT/'排版检查';QA.mkdir(exist_ok=True)
read=lambda n:(P/n).read_text(encoding='utf-8',errors='replace')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
validation=json.loads(read('VALIDATION_SPEED.json'))
diff=[n for n,h in validation['source_hashes'].items() if not(P/n).exists() or sha(P/n)!=h]
source=read('User/App_ThiefStrategy.c')
maptext=read('Algorithm/MapRoute/MapRoutePlan.c')
nodes=[tuple(map(int,x)) for x in re.findall(r'\{(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\}',maptext.split('const MapRouteSegment_t')[0])]
edges=[tuple(map(int,x)) for x in re.findall(r'\{(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\}',maptext.split('const MapRouteSegment_t')[1].split('};')[0])]
fonts=Path('C:/Windows/Fonts')
fontpath=fonts/'msyh.ttc'
def ft(n):return ImageFont.truetype(str(fontpath),n)
im=Image.new('RGB',(2100,1740),'white');dr=ImageDraw.Draw(im)
xy={n:(120+x*80,100+y*80) for n,x,y,k,d in nodes}
for _,a,b,l,h in edges:dr.line((*xy[a],*xy[b]),fill='#C2CCD5',width=7)
route=[54,53,44,36,35,26,17,7,6,5,4,3,2,11]
for a,b in zip(route,route[1:]):dr.line((*xy[a],*xy[b]),fill='#D27619',width=12)
for n,x,y,k,d in nodes:
    xx,yy=xy[n];dr.ellipse((xx-6,yy-6,xx+6,yy+6),fill='#275675')
    dr.text((xx+8,yy-25),f'N{n:03}',font=ft(21),fill='#153C59')
dr.text((120,1600),'橙色为首次取宝到 N011 路线   北在上 东在右',font=ft(35),fill='#25495F')
im.save(QA/'map.png')

doc=Document();sec=doc.sections[0]
sec.page_width=Inches(8.5);sec.page_height=Inches(11)
sec.top_margin=Inches(.8);sec.bottom_margin=Inches(.75);sec.left_margin=Inches(.8);sec.right_margin=Inches(.8)
for name in ['Normal','Body Text']:
    st=doc.styles[name];st.font.name='Calibri';st.font.size=Pt(11)
    st.element.rPr.rFonts.set(qn('w:eastAsia'),'宋体')
    st.paragraph_format.line_spacing=1.22;st.paragraph_format.space_after=Pt(6)
for name,size in [('Title',29),('Subtitle',14),('Heading 1',18),('Heading 2',13),('Heading 3',11.5)]:
    st=doc.styles[name];st.font.name='Calibri';st.font.size=Pt(size);st.font.color.rgb=RGBColor(0,0,0)
    st.element.rPr.rFonts.set(qn('w:eastAsia'),'微软雅黑')
    st.paragraph_format.space_before=Pt(12);st.paragraph_format.space_after=Pt(8)
doc.styles['Heading 1'].paragraph_format.page_break_before=False
for border in list(doc.styles.element.xpath('.//w:pBdr')):
    border.getparent().remove(border)
header=sec.header.paragraphs[0];header.text='Robot Treasure  技术文档内容底稿';header.style='Caption'
footer=sec.footer.paragraphs[0];footer.alignment=WD_ALIGN_PARAGRAPH.CENTER
footer.add_run('技术草稿  ·  ')
fld=OxmlElement('w:fldSimple');fld.set(qn('w:instr'),'PAGE');footer._p.append(fld)
def p(t,style=None):return doc.add_paragraph(t,style)
def h(t,level=1):
    para=doc.add_heading(t,level)
    if level==1 and (t.startswith('1 ') or t.startswith('附录') and not t.startswith('附录G') or t=='阅读导航'):
        para.paragraph_format.page_break_before=True
def eq(t):
    para=doc.add_paragraph();para.alignment=WD_ALIGN_PARAGRAPH.CENTER
    om=OxmlElement('m:oMath');rr=OxmlElement('m:r');tt=OxmlElement('m:t');tt.text=t;rr.append(tt);om.append(rr);para._p.append(om)
def table(head,rows,widths=None):
    t=doc.add_table(rows=1, cols=len(head));t.style='Light Shading Accent 1'
    for c,v in zip(t.rows[0].cells,head):c.text=str(v)
    pr=t.rows[0]._tr.get_or_add_trPr();repeat=OxmlElement('w:tblHeader');pr.append(repeat)
    for row in rows:
        cells=t.add_row().cells
        for c,v in zip(cells,row):c.text=str(v)
    for row in t.rows:
        trpr=row._tr.get_or_add_trPr();avoid=OxmlElement('w:cantSplit');trpr.append(avoid)
        for i,c in enumerate(row.cells):
            tcpr=c._tc.get_or_add_tcPr();mar=OxmlElement('w:tcMar')
            for tag in ['top','bottom','left','right']:
                item=OxmlElement('w:'+tag);item.set(qn('w:w'),'90');item.set(qn('w:type'),'dxa');mar.append(item)
            tcpr.append(mar)
            if widths:c.width=Inches(widths[i])
            for para in c.paragraphs:
                para.paragraph_format.space_after=Pt(3);para.paragraph_format.line_spacing=1.08
                for run in para.runs:run.font.size=Pt(9)
    p('')
    return t
def chapter(title,parts):
    h(title)
    for sub,body in parts:
        h(sub,2)
        for text in body.split('\n\n'):p(text)
def code(text):
    for line in text.splitlines():
        for part in textwrap.wrap(line,94,replace_whitespace=False,drop_whitespace=False) or ['']:
            para=p(part);para.paragraph_format.space_after=Pt(0);para.paragraph_format.line_spacing=1.05
            for run in para.runs:run.font.name='Consolas';run.font.size=Pt(8)
    p('')

p('密室夺宝机器人系统技术文档','Title')
p('STM32 四轮差速底盘与小偷自主逃跑系统','Subtitle')
p('内容扩展草稿  版本 D1  2026年10月31日')
p('对象工程  Thief16V_UARTCh4_RouteOnly_Test')
p('固件证据基线  '+validation['firmware'])
p('编写用途  为后续学校模板、竞赛技术报告、答辩材料和代码交接准备可复用内容。当前没有固定版式要求，因此优先保留技术细节、实现边界、参数出处与验证证据。')
p('核心结论  系统采用双灰度循线、四轮编码器速度闭环、陀螺仪转弯辅助、拓扑地图执行、有限状态风险搜索和实际路径逆序回退。当前控制器不依赖摄像头、YOLO、ROS或运行在车上的Python。')
p('证据说明  本文以本地代码快照为主，GitHub项目与官方资料为补充。代码存在、编译通过、离线模拟通过和真实车辆通过分别表述；没有实车证据的性能指标不写成实测结论。仅编译通过，尚未完成硬件验证。')
table(['项目','本稿记录'],[['本地工程',str(P)],['公开仓库','https://github.com/aaayolo-ui/Robot_Treasure'],['作者信息','项目所有者 aaayolo-ui  技术整理草稿'],['最终用途','待补充比赛正式规则 学校模板 队员分工和实测结果'],['更新原则','以最新源文件和映像散列优先 历史README仅作沿革']],[1.1,5.8])
h('阅读导航')
p('正文按硬件、底层控制、地图和策略、通信监控、验证与资料来源组织。附录提供代码实际宏定义、88个地图节点、106条道路、源码入口和离线测试目录。可先阅读第1至4章理解整体，再按模块查阅。')
toc=OxmlElement('w:fldSimple');toc.set(qn('w:instr'),'TOC \\o "1-1" \\h \\z \\u');doc.add_paragraph()._p.append(toc)
p('目录采用Word标题样式生成；后续套模板时可整体更新。')

chapter('1 系统目标与任务边界',[
('1.1 项目背景','本项目面向密室夺宝类对抗场景：车辆沿固定黑线道路取宝，进入规定藏点，在传感器确认附近存在威胁后选择离开路径，并在途中前方受阻时回退。比赛具体判分、复位规则、运行时长和安全要求仍应以正式规则为准，本稿不把聊天中讨论的经验当成官方条款。\n\n系统关注的是道路约束下的可执行策略。纯粹的几何直线距离不能代替沿道路行驶的距离；纯粹的最短路也可能把车辆送向敌车。策略层必须同时考虑朝向、倒车、路口停车和转弯耗时，执行层则必须以真实灰度维持车轮运动。'),
('1.2 设计目标','主要目标包括稳定识别黑线和路口、保留经过验证的普通动作、快速响应前方拦截、减少无意义的左右改路、在有限主控存储内完成搜索，以及通过同一条遥测链同时展示原始串口、激光和地图轨迹。\n\n本项目没有已验证的全局绝对定位系统。节点位置是从已知起点和路口序列推断的地图位置；激光提供距离回波，不能独立识别人、墙或另一辆小车。报告应描述“疑似威胁”“障碍回波”，而不是把条件标签当成目标识别真值。'),
('1.3 文档范围','本文覆盖当前小偷工程以及关联的Windows监控助手。其他16U、16B、Gray12和历史试验版本可作为对照，但不自动继承其功能。BRANCH1支路漏读修复与LR1移动重定位不在当前被确认的控制链内，不应作为现行功能宣传。')])

chapter('2 当前版本与工程证据',[
('2.1 基线识别','当前已保存验证报告的固件标识为 '+validation['firmware']+'。工程应通过对应uvprojx构建。HEX是烧录映像，AXF同时包含可执行代码和调试信息；只有二者与源码、报告匹配时，验证结论才可复核。\n\n最新参数仍可能被人工调整。例如用户可修改LineFollowPID.h中的巡线基速或ThiefStrategyConfig.h中的测距上限。本文生成时重新读取实际文件，并与最近验证报告逐项比较，不把历史数值默认为当前值。'),
('2.2 源码一致性','本次静态核对与VALIDATION_SPEED.json中生产输入散列相比，不一致文件数为 '+str(len(diff))+'。'+('当前所列生产输入与该报告一致。' if not diff else '存在修改，报告只证明历史映像；差异文件为：'+ '、'.join(diff))+'\n\n记录最终AXF SHA256为 '+validation['axf_sha256']+'。散列能证明文件身份，不能证明车轮实物映射、地面附着、接线电压或避障成功率。'),
('2.3 证据优先级','本地源码与构建产物回答“当前程序做什么”；离线测试回答“在给定模拟输入下控制分支是否成立”；串口实录回答“设备当时报告了什么”；录像和实物位置回答“车实际上在哪里”。四类证据要相互印证。\n\n例如停车后输出的TW记录是历史缓存导出，其中非零PWM或旧时间戳不能直接解释为停车后仍实时驱动。又如网页显示N006而实物在N007，首先要检查节点推进、原始灰度和编码器锚点，而不是把图像当成独立定位传感器。')])
table(['资源项目','最近验证记录'],[['ROM',str(validation['rom_bytes'])+' bytes'],['RAM',str(validation['ram_bytes'])+' bytes'],['剩余RAM',str(validation['ram_remaining_bytes'])+' bytes'],['预留栈',str(validation['reserved_stack_bytes'])+' bytes'],['链接器已知最大栈',str(validation['linker_known_stack_bytes'])+' bytes'],['离线套件数',len(validation['suites'])],['硬件验证状态',validation['hardware_tested']]], [2.5,4.4])

chapter('3 硬件平台与系统连接',[
('3.1 主控与底盘','工程以STM32F103RCT6兼容目标配置，项目规则说明实物使用兼容的APM32F103RCT6。资源预算按48KB RAM、256KB Flash核算，不能把其他F103封装型号的容量直接代入。\n\n公开项目README记录R5四轮底盘、四个MC520P30_12V有刷编码器电机、约65mm轮胎与四路TB6612FNG类驱动板。型号和购买信息属于项目资料，电池状态、安装尺寸及最终实车BOM仍需实物复核。[W1]'),
('3.2 传感器分工','前后各一块NCHD12十二路灰度板承担道路观测。两块板是前后分布，不是横向连续二十四路；转向误差来自两个独立观测，不能简单把位图拼接后统一求中心。\n\n编码器用于轮速反馈、相对里程和运动约束；JY61P提供YAW变化；四个AI-1811方向通道提供距离回波。不同传感器的失效模式不同，组合的关键是明确各自负责什么，而不是声称进行了完整概率融合。'),
('3.3 电源与串口','动力和逻辑供电需按实际板卡规格确认，所有相关信号共地。公开资料给出3S电池、动力支路及DC-DC控制支路的方案，但文档不能取代电压、极性和逻辑电平实测。\n\n当前四路激光经UART-CH4链路进入主控，主串口配置115200。电脑端经USB转TTL与蓝牙串口链路接收。串口已连接只代表操作系统端口打开，不代表有新传感器字节；应同时看接收字节数、帧计数、帧年龄及解析错误。'),
('3.4 提示与独立通道','PA3保留给上电持续声光提示，当前实现是一次GPIO初始化后维持输出，并不依赖逃跑状态。不能把该脚重新启用为USART2_RX而不评估冲突。\n\n四电机必须保持独立驱动通道和独立编码器反馈。底层FWD/REV电平、驱动板A/B/C/D插口与逻辑前后左右是不同概念：现有映射记录FL对应B、FR对应A、RL对应D、RR对应C；改线后必须逐轮验证，不能仅凭同侧目标相同而把通道并联。')])

chapter('4 软件结构与执行时序',[
('4.1 分层组织','Core负责启动和CubeMX生成初始化；Hardware负责电机、编码器、灰度、陀螺仪和测距接收；Algorithm包含灰度位置、地图、搜索和威胁模型；Control承担轮速、循线、行进坐标变换和恢复；User组织任务与遥测；System提供时间和调试基础设施。\n\n这种分层使策略变更可以尽量限制在规划器，而不反复修改普通巡线。历史问题表明，把某个路口的误停通过大范围更换转弯流程来修补，容易破坏已稳定的基础动作，因此每次修改都应记录差异和基线。'),
('4.2 周期与非阻塞','常规循线控制周期为20ms，快速灰度路口观察约5ms。快速观察用于避免窄路口在两个控制采样之间漏读；它不意味着PID和四轮目标也以5ms重复计算。\n\n激光接收、按键、日志缓存和任务状态都要在主循环中有界执行。主控不适合在高速循线循环中无限扫描文本、同步输出大段日志或临时分配大型数据结构。串口输出应限制频率并保留原始诊断计数。'),
('4.3 状态机的意义','状态机负责把“行驶”“入中心”“制动”“旋转”“接线”“停车”等机械过程拆成可审计阶段。状态切换需要物理证据，例如轮速反馈停稳和新鲜陀螺仪帧，而不能只等待固定延时后假设动作完成。\n\n路线规划给出目标节点和动作，状态机执行动作；灰度是执行阶段的道路证据。规划器输出正确并不能保证车轮转向正确，必须分别检查行进帧、YAW符号、轮速目标以及接线阶段是否提前退出。')])

chapter('5 灰度采集与黑线特征',[
('5.1 位图和坐标','每块NCHD12输出十二路黑白状态，经Extract12整理为低十二位。P1至P12的物理顺序、安装朝向和TravelFrame转换共同决定“左”和“右”。LED亮灭与软件黑线位的关系要按实际模块验证，不能凭肉眼灯亮就断言软件是1。\n\nGrayPosition对有效黑色位求位置，GrayCoordinate把板上位置映射为车辆误差。中心参考、偏差缩放和边缘分类用于控制与路口判断；这些误差值不是毫米，也不是实际车身角度。'),
('5.2 特征分类','现行代码区分普通有效线、左侧黑、右侧黑、全黑、宽黑、无线、I2C错误及后板不可用。不同类别的物理含义不同：无线可能来自间隙或真正出线；全黑可能是路口或文字；I2C错误则是通信失败。\n\n路口通常依据外侧位组合识别，并配合武装、锁存和地图上下文。单次外侧黑位不能无限重复计数；离开路口后需重新看到稳定线，再武装下一个路口。原严格支路确认例如左侧P11/P12同时黑可能漏掉不对称回波，这也是保留原始位图的原因。'),
('5.3 前后板误差','当前位置纠偏以行进前端误差为主；双板有效时再加入前后误差差值作为航向项。程序没有单独的前后灰度板安装间距毫米参数，所以该航向项是经验比例纠偏，不是根据几何基线计算的真实角度。\n\n如果后续测出板间距并希望估计车身角度，需要同时标定单板位移尺度、安装偏心和符号，才能使用几何模型。不能只新增一个间距数字，就把原误差差值解释为准确YAW。')])
eq('e_position = e_leading     e_heading = e_leading − e_trailing')

chapter('6 巡线外环 PID 与平滑',[
('6.1 外环控制律','LineFollowPID计算位置P、I、D修正，双板模式另外叠加航向P。当前实现采用整数缩放，参数名称带X1000，速度目标使用RPM×10；读数变化、参数和单位必须一起理解。\n\n微分项对误差增量作一阶IIR滤波，旧值占3/4，新增量占1/4，再按实际周期换算微分输出并限幅。积分采用条件积分，在同方向饱和时不继续累积，避免退出偏差后长时间反向拉扯。'),
('6.2 双轮目标与饱和','控制输出映射为左基速加修正、右基速减修正；再经FollowSmoothing限制基速或转向变化，最后由TravelFrame映射到四个物理车轮。控制器的基速是目标，不是编码器已经测到的真实速度。\n\n侧边线和中心边缘会使用不同的纠偏增益比例。调参应先核对方向，再调比例，最后考虑积分和微分；不能用增大PID去掩盖车身尚未转到出口、轮子制动锁死或灰度物理映射错误。'),
('6.3 平滑与速度','FollowSmoothing用于减少目标突变，避免起步和大误差时左右轮瞬间反向。轮速PI解决电机速度，循线PID解决道路误差，平滑解决指令变化率，三者不应混成一个“速度参数”。\n\n当前用户要求硬币瞬间遮挡时保持进入遮挡前的平滑基速，辅助代码因此读取follow_smoothing.base_rpm_x10，而不写死50RPM。正常巡线默认基速应看本稿附录提取的实际宏定义，历史注释可能滞后。')])
eq('u = P + I + D + H     v_left = v_base + u     v_right = v_base − u')
eq('delta_filtered = (3 delta_previous + delta_new) / 4')

chapter('7 四轮编码器与轮速 PI',[
('7.1 速度估计','Encoder.h当前定义每圈1560计数。采用增量计数和实际采样间隔换算RPM×10，并根据定时器计数模式处理正负及回绕。该计数必须与实车编码器及减速比配置对应，不能机械套用别的电机参数。\n\n四轮编码器分别采样。车轮逻辑速度与底层电机电气方向并不相同，因此速度反馈在进入闭环之前必须进行经过确认的符号映射。某个轮子实际旋转却始终测零，会使该通道PI持续增加输出。'),
('7.2 内环与执行','WheelSpeedPi按目标与实测速度误差更新控制输出，并限幅到允许PWM范围。闭环控制先解决同一目标下电机差异，再由外环决定左右目标差；同时改外环和内环增益会让问题难以归因。\n\n制动、使能、PI状态复位和方向互换需要明确顺序。若两个轮子被短刹车锁定而另两个轮子尝试小幅转动，声音可能很大但车身几乎不动；应查每轮PWM、方向和反馈，而不是笼统归为定位计算慢。'),
('7.3 里程与误差','MapRoutePosition把轮程转换成毫米，用于地图段进度和辅助判断。轮胎标称直径不等于负载后的有效滚动周长；轮滑、倒车、转弯和刹车滑移会改变里程，因此程序不应单凭累计里程强认一个灰度路口。\n\n半路折返时，必须保留折返前进度与制动滑移，建立反向部分路段。旧入口锚点、前后运动的符号和新路段清零时刻若混用，就会出现“仅退了几十毫米却认为已走满一段”的位置错乱。')])
eq('RPM = delta_count × 60000 / (counts_per_rev × delta_time_ms)')
eq('distance_mm ≈ wheel_revolutions × calibrated_circumference_mm')

chapter('8 行进坐标与倒车循线',[
('8.1 TravelFrame','TravelFrame把物理车头与当前行进前端分开。倒车时后板成为行进前端，相应的位序、误差和车轮目标只允许统一映射一次，不能应用层取反、底层再取反。\n\n地图北东南西是绝对方向枚举，前左右后是相对于车身的传感器方向，前进或倒车是运动模式。三者不能混用。例如车头朝南倒车向北，行进方向左转对应地图向西，转后车头可能朝东仍以倒车向西。'),
('8.2 换向条件','切换行进端前先制动，再读取四轮速度，连续确认停稳后切换。固定延时只说明等待过，不说明惯性已经消失。新方向起步不能因某一帧前端无线而立即永久锁死，也不能在双白时按地图盲冲。\n\n已发生的倒车不流畅问题，常与过早从倒车切回前进、缓存速度未更新、旧路口再次计数或转弯未完成时采用后板旧线有关。现有恢复通过前后线形一致性、停稳握手和重新武装来限制这些误切换。')])

chapter('9 陀螺仪与普通转弯',[
('9.1 YAW的使用','JY61P的YAW用于相对旋转角、方向符号和新鲜度判断。角差需跨越±180°时折返到最短有符号角，避免从179°到−179°被算作358°。旋转前要求取得制动后的新角度帧，减少使用惯性制动前旧值的误差。\n\nYAW原点和地图朝向不是天然相同。程序通过启动参考与学习的顺时针符号建立关系；长时间漂移与安装偏角仍可能存在。灰度纠偏与YAW检查是互补关系，不能把每个普通路口都强制停到地图轴±2°作为常规动作。'),
('9.2 主旋转与接线','当前普通转弯采用约45°主旋转，再进入既有接线和巡线阶段。这不是只转45°就认为已经完成90°道路转向；其余机械转动通过连续接线与PID收敛完成。\n\n入路口动作通常为100RPM持续180ms，保证灰度板识别路口后，车身中心继续靠近转弯中心。随后实测停稳、开始固定PWM主旋转。停车到藏点90°是另一套过程，不应与普通转弯的主旋转参数混淆。'),
('9.3 找线边界','RestartLineSearch只在明确恢复上下文使用。普通巡线、地图直通过程和转弯后接线具有不同线索：转弯未完成时后板可能仍压着旧路，不应因此向前或后退到错误道路。\n\n持续双白两秒停车、真实陀螺仪过期、I2C异常、堵转和按键停止继续存在。当前工程仍可能因严格支路判断而停车；本文没有把另一个未在当前目录中的修复当成已上线功能。')])

chapter('10 拓扑地图与节点执行',[
('10.1 图模型','地图包含'+str(len(nodes))+'个节点和'+str(len(edges))+'条道路。节点存储编号、半格坐标、类型和度数；坐标以150mm为半格单位，道路记录起终点、长度及地图方向。地图坐标y向下增加，北在上，不能直接套用数学坐标y向上而不改符号。\n\nN005是宝藏内部标记，不是普通灰度路口。规划器BuildRoute会把同方向、经过内部节点的段合并，例如N006→N005→N004形成N006→N004动作段，同时保留两条实际道路的长度。'),
('10.2 路线合法性','MapRouteNavigator在开始时校验节点连接、段长度、段数、动作和前后模式是否一致。相邻段航向差决定左转、右转、直行或运动反向；反向会切换drive_mode，不等价于让车身旋转180°。\n\n地图估计不能单独证明车辆真实位置。灰度读到文字或误识别路口时，序列推进仍可能错位；执行器需要保留原始位图、进度和所选动作，才能对照实物复盘。'),
('10.3 路口提交','到达路口不应立即等同于已经走上下一段。直行、旋转和反向应分别完成其动作和接线后再提交，避免一次路口被重复消耗。返回历史也应在实际动作确认后记入，而不是保存未来规划节点。\n\n编码器用于筛除明显过早的残留线索或显示进度，其可靠性低于真实路口特征。观察器的LOC_DISTANCE、OBS_OK等诊断与应用执行器有不同语义，不能把网页VALID=0直接理解为车辆没有道路信息。')])
doc.add_picture(str(QA/'map.png'),width=Inches(6.8));p('图1 当前拓扑地图与首次取宝路径  坐标及连边直接取自MapRoutePlan.c','Caption')

chapter('11 风险搜索与藏点评分',[
('11.1 状态扩展搜索','ThiefRiskSearch将节点、地图朝向和运动模式合并为状态，单节点有四方向乘两模式，因此88节点对应最多704个状态。搜索使用固定长度数组保存代价、到达时间、父状态、访问位和安全余量，避免在主控运行时动态分配。\n\n其结构接近带风险和朝向代价的有界Dijkstra式标签搜索。代码明确说明每个状态只保留一个标签，是内存受限启发式，并非完整SIPP、安全区间最优搜索或严格极小极大追逃求解器。报告应使用符合实现的名称。'),
('11.2 边代价与时间','普通道路时间按长度除以正向/反向模型速度计算，再加转弯和换向额外耗时。模型速度属于规划假设，修改实际RPM之后，若不重新标定模型，预测到达时间与真实用时可能不一致。\n\n转弯风险不能仅计算静止到达时刻，因为车身要经历入中心、制动、旋转和清出路口。代码对路段与清出阶段按不大于75mm间隔采样安全余量；这是一种离散近似，不是连续时间碰撞必然安全证明。'),
('11.3 目标比较','候选藏点先满足允许出口、地图连通和停车轴向。可行候选优先安全层级，再比较与记忆目标的沿图分离、是否刚暴露或刚用过、累计风险代价和出口条件。没有安全路径时可给出DEGRADED，说明取舍降级，而不是保证不被抓。\n\n分离距离有离散格桶，冷却和前次藏点影响偏好，出口数量和直线逃离距离影响反应余量。距离远的藏点并非总是最好：若转弯过多、要穿过敌车方向或进入封闭袋形区域，评分应降低。'),
('11.4 规则与搜索结合','最新首次路线和N011部分方向采用明确规则覆盖，其他情形再由风险搜索选择。规则覆盖适合用户已确认的地图通道，搜索适合多种可行路线的比较；两者优先级应在代码和文档中一致。\n\n固定通道不是数学全局最优证明，也不表示敌车变化时仍永远最佳。它的收益在于减少易受重复观测干扰的改路与犹豫，同时用原前方拦截例外处理真正堵在当前行进前方的障碍。')])
eq('state = (node − 1) × 8 + heading × 2 + drive_mode')
eq('T_edge = ceil(1000 L / v) + T_turn + T_switch')

chapter('12 四路激光接收与有效性',[
('12.1 数据链路','四路激光距离从UART扩展链路进入解析器，再按物理CH1前、CH2左、CH3右、CH4后分发。每路距离、置信度、帧年龄和解析状态应独立维护；不能因右侧坏了就让前侧的软件读数一起失效。\n\n硬件输出频率、主控接收频率、主控日志上报频率、电脑界面刷新频率是四个量。网页每秒刷新五十次，不等于传感器每秒产生五十个有效新测量；重复的157mm也可能是持续收到相同数据，而不是网页卡死。'),
('12.2 阈值与投影','软件有效范围和逃跑触发阈值以ThiefStrategyConfig.h为准。最近确认版本前后原始上限1000mm、左右900mm；对应车身中心投影还使用前后150mm、侧面100mm偏移。原始探头距离与地图中心距离不是同一个量。\n\n这些偏移是工程模型，并不等于已完成精密几何标定。实际安装位置、光轴高度、离地反射、目标材质和斜面都会影响测距。阈值增加之后，隔着另一条路甚至场外墙体也可能进入有效回波，需要继续结合地图方向过滤。'),
('12.3 稳定确认','单帧近距离不立即等于敌车。FrontLaserGate和历史回退触发各有新鲜度、置信度与连续帧条件；原三帧确认的具体代码位置和距离跨度应区分，不能在报告中把所有门控统一写成同一组数字。\n\n原始帧数量能判断通信是否活着，有效帧数量才能判断策略有无可信输入。线缆交换后问题跟随探头或跟随通道，可帮助区分探头、供电、扩展板和软件映射，但不能仅凭一次互换就永久宣告硬件正常。')])

chapter('13 场外过滤与威胁模型',[
('13.1 方向转换','激光通道首先相对物理车身定义，再结合当前车身地图朝向转成北东南西。倒车模式改变行进前方，却不改变物理前激光的位置，因此“前方受阻”应按实际行进端选择CH1或CH4。\n\n地图边缘过滤应依据当前姿态、探头方向和回波落点是否在场内，而不是简单禁用某个边缘节点所有激光。在边缘朝图内的探头仍可能看到真实敌车，不能一概忽略。'),
('13.2 落点与道路','回波按探头偏移投影到地图平面，再评估是否影响当前或下一道路。侧面某条路有回波，不应仅因“同为西侧”就封堵另一条相隔较远的道路。道路连接距离与空间落点都需要参与判断。\n\n位置或姿态错误会使过滤判断一起错，这是一种关联误差。当前观察器没有提供独立绝对定位，所以遇到墙被当敌车时，应同时查坐标、BODY、MODE、YAW和回波，不能只调小量程。'),
('13.3 风险解释','ThiefThreatModel保存有限历史观测，并估计敌车可能沿道路接近的余量。模型只能根据已有回波作保守或启发式预测，并不知道对手真实控制策略。\n\n本项目默认实景为一辆小偷与一辆追车，不能凭多个方向回波就新增多辆敌车的真实结论。多方向回波也可能是同一个目标在转向后的新投影，或场内其他静态物体。')])

chapter('14 首次取宝与当前藏点规则',[
('14.1 首次完整路线','当前首次路径为N054→N053→N044→N036→N035→N026→N017→N007→N006→N005→N004→N003→N002→N011。用户确认先前写的N077是上方N007笔误，程序没有绕到地图下方N077。\n\nN035到N007之间实际经过N026和N017，不能构造35到7的虚拟边。通过宝藏段时实际是N005驶向N004；到N004后沿顶部经N003、N002，在N002转弯向南进入N011。'),
('14.2 首次停车姿态','从N002进入N011时车身朝南，用户要求保持N011横停，因此车辆先完成原180ms居中与真实停稳，再执行单独授权的停车90°转向，目标朝西。它只针对首次1600路线的N011竖向到达，不会自动推广到所有普通转弯。\n\nN011和N035为横向藏点，N026、N038、N063为竖向藏点；N078是规则指定的临时等待点，不是通用第六藏点。实际最终轴向还要以当前规划final_body_heading与停车动作核对。'),
('14.3 N011方向规则','在N011，第一次完成确认的地图北侧来车与西侧来车都走11→12→23→31→32→48→49→61→62→63，到N063居中后按原停车动作竖停。地图东侧来车仍先走10→20→37→38→39→40；南侧来车经12→13→3→4→5→6→7再接搜索到东侧藏点。\n\n首次方向完成确认后锁存，不被后续侧后方回波反复替换；到目标藏点后解除旧锁并等待新证据。当前行进前方真正被连续确认拦截时仍保留实际路径回退例外，因此“锁定”并非无视前方所有碰撞。'),
('14.4 其他藏点','N038竖停时，地图东西侧威胁优先经N021往北部通道，北侧威胁经N039、N040向东，南侧沿原向东分流。N063北侧威胁优先南下经N078、N077、N076、N075，再择N011或N038；仅西侧威胁可先去N078临时等待。\n\nN078的新北侧威胁走N077、N076、N075后再藏；新西侧威胁走N063、N052去上部。该点没有N078→N087直接道路。首次到N026或N035等非固定情形仍由原风险搜索和可行出口约束决定，不能把初始规则误套到后续所有藏点。')])
table(['触发位置与方向','已确认的通道','终点或约束'],[['首次取宝','35→26→17→7→6→5→4→3→2→11','N011朝西横停'],['N011 北或西','12→23→31→32→48→49→61→62→63','N063竖停'],['N011 东','10→20→37→38→39→40','再去东侧藏点'],['N011 南','12→13→3→4→5→6→7','再搜索东侧藏点'],['N038 东或西','21及上部通道','随后由风险搜索选择'],['N038 北','39→40','东向逃离'],['N063 北','78→77→76→75','选择N011或N038'],['N063 仅西','78','等待新证据']],[1.3,3.5,2.1])

chapter('15 实际路径记录与遇阻回退',[
('15.1 保存已经走过的边','ThiefRouteTrail保存实际确认的道路边、运动模式、灰度、YAW与轮程，不只保存规划路线。走到路口先暂存，完成直行或转弯接线后再提交，避免把尚未真正到过的节点写入返回历史。\n\n固定容量避免不可控内存增长。历史最多保存的边数、结构体字节数及可用回程空间应以当前头文件和测试报告核对；项目历史记录为47条已执行边加1条待确认记录，约876字节工作结构。'),
('15.2 半路拦截','行进前方三帧稳定近回波确认后，先制动并确认四轮停稳，再构造当前部分路段的反向和已执行历史逆序。倒车时前方可能是物理后探头，测试必须按TravelFrame选择通道，不能把旧路线的CH4写死成永远的行进前方。\n\n当前部分边要考虑折返前已经走过的里程及制动滑移。灰度仍决定真正返回路口的时刻，轮程只排除明显残留，不把里程上界直接变成永久锁死。路线连接不成立时不能编造“原路返回成功”。'),
('15.3 返回执行','返回中直行路口继续通过，最近转弯点完成相应转向后沿旧路回到起始藏点。回程侧后方回波不反复改计划；若回程行进前方又被新障碍确认阻挡，则退出该次历史回放并进入原回撤重规划。\n\n回到原藏点要恢复其原始停车姿态。实际历史没有独立绝对定位能力：如果出程误认路口，返回记录仍会错，因此必须保留传感器证据，并在重复纹理区谨慎解释“记录了实际路”。')])

chapter('16 取宝反光与短时失线处理',[
('16.1 干扰模型','用户描述铁块约硬币大小，前板反光时后板仍在线上。该条件支持短时采用真实后板线索，但不支持前后灰度全部关闭后长距离盲开。当前实现针对无线分类，不把所有反光形式都当成同一故障。\n\n若反光使前板读成全黑、边缘多簇、I2C错误或产生相反的误差，现有无线分支未必启用。实测时应先看原始RAW12，再决定是否需要另一类处理。不要仅凭“铁块会反光”就将整片地图的灰度证据屏蔽。'),
('16.2 原速后板辅助','Route_TreasureRearBridge仅在初始1600路线N006→N004、正向FOLLOW、尚未确认到点时工作。每次都要求前板为无线、后板为真实紧凑有效线。基速读取FollowSmoothing当前平滑值，后板P修正最多±20RPM，左右平均目标仍等于入口基速。\n\n辅助从首次无线处记录里程，最多50mm；前板恢复交还原巡线，后板也失线或超范围进入原恢复。5ms快速检测与20ms控制共用判定，防止快速入口提前截走；轮速更新仍按20ms。没有给前板伪造一条黑线，也没有因此提交N004。'),
('16.3 限制与复测','后板单独的横向位置无法完整确定车身倾角，高速下长距离依靠后板可能放大姿态误差。因此50mm只是一项明确的短遮挡范围，不是更强的定位能力。用户要求原速通过已经落实为目标基速保持，实物是否平稳仍需测试。\n\n应分别记录无铁块、铁块不同位置、灯光角度、前板离地高度和不同电池状态下的数据，核对前板无线持续时间、实际经过距离及恢复后的车身方向。硬币大小不等于测量光学盲区正好等于硬币直径。')])

chapter('17 停车 故障与恢复的边界',[
('17.1 受控停车','KEY停止、持续双板全白两秒、真实I2C或陀螺仪异常、制动未停稳和电机堵转属于不同原因。串口应输出具体reason和前一状态，不能统一叫“保护代码”。当前工程中的MAP_REQUIRED_BRANCH_NOT_SEEN仍可能因为严格支路条件触发，本文不宣称所有误停已消除。\n\n制动后更新反馈是必要条件。如果只发刹车但软件缓存仍保存高速值，会一直判定没停稳；反之直接把缓存写零，则可能在车辆仍滑动时就开始转向。离线测试应保留真实判定函数并只模拟物理轮速响应。'),
('17.2 无线与通信错误','无线是一个有效的传感器结果，通信错误是没有获得可靠结果。两者不能共用同一“延时忽略”来掩盖。短无线可根据后板、道路上下文和历史误差纠偏；通信失败则要先确认总线和数据新鲜度。\n\n两秒双白用于比赛中持续失去道路线索的情况，但不是取代所有停机原因。主控任务时间、局部动作超时、传感器帧超时也各有用途，不能简单把所有超时都理解为从出发到比赛结束。'),
('17.3 定位不确定时','当前定位观察器仍是诊断，未实现经过实车验证的多候选移动重定位。曾讨论的LR1保留候选、连续路口消歧属于独立试验，不在当前运行版本。若要重新采用，应单独验证地图游标冻结、物理出口选择和重新接回策略的条件。\n\n继续循线比原地长期停止更符合逃跑目标，但需要真实线索授权运动，尤其在T形路尽头和文字干扰区。不能把“尽量不要停”落实为双白也按地图猜测直冲。')])

chapter('18 串口遥测与日志设计',[
('18.1 遥测类型','主控输出BOOT与固件身份、LASER四方向测距、LOC节点位置与姿态、MAP_NODE动作、路线锁存、TRACE历史提交和回退，以及MT3等紧凑事件。文本字段能直接用于串口助手查看，结构化事件则支持网页轨迹和计时。\n\n测距状态RAW、STALE、BAD_DATA、TOO_FAR、POSE_OFF等反映数据资格或姿态条件，不都是传感器断线。网页展示有效距离和策略真正允许触发是两层逻辑，应在报告和界面中分开说明。'),
('18.2 时间与追溯','日志至少存在主控毫秒时基、电脑接收时间和显示刷新时间。主控T=99068表示该时基99.068秒，若缺少KEY2启动事件，不能直接说本轮出发用了99.068秒。缓存导出还可能将较早采样在停车后集中打印。\n\n解析器应该容忍一行被分成多个串口块、多个事件合在一块，以及历史文本混杂。不能认为每次串口read就是一整帧。原始字节保存是修复网页解析的重要依据。'),
('18.3 控制与日志隔离','实时控制应优先，日志缓存和上报需要有界。日志过量会增加UART队列压力、丢字和主循环迟到，从而让“更多调试输出”反过来改变车辆行为。\n\n本项目保留TW等轮速快照以便在停车后导出，并记录丢帧、队列或最大控制延迟等诊断。验证时既看打印内容，也看输出是否打断20ms控制，不能只用一段漂亮串口证明实时性。')])

chapter('19 Windows一体助手与网页监控',[
('19.1 统一数据源','DesktopSerialMonitor使用Windows桌面程序与WebView2，嵌入串口调试和本地监控。后台只打开一次COM端口，同一份接收数据供文本显示、激光卡片和地图轨迹使用，避免网页与串口助手争抢端口。\n\n手动开启后台不等于连接串口；连接COM不等于收到数据；公网同步进程运行不等于远端已经更新。这些状态需要独立显示，并保留手动连接、断开、停止同步等明确按钮。'),
('19.2 本地与公网','本地监控依赖电脑上后台和页面资源。127.0.0.1只代表当前电脑，不是其他网络都能访问的公网地址。公网页面还依赖上传服务、鉴权、网络和托管平台，不能只把本地URL发给另一台设备。\n\n一体助手README记录公网访问曾遇网络拦截，程序状态不能替代实际网络验证。浏览器或WebView的显示副本与主控控制代码应解耦，刷新页面不能被误当成重新启动实物车辆。'),
('19.3 显示与计时','用户要求同屏串口、四路距离与车辆轨迹，地图节点编号常显，接收框可拖动高度，地图可滚轮缩放，并从出发到停车计时。现有资源可支持这些交互，但EXE可能捆绑旧资源，文档应同时检查源码、构建与实际运行入口。\n\n暂停显示应只暂停渲染，不停止串口接收与原始日志。轨迹属于主控上报位置及网页插值，不是独立视觉定位。显示旧坐标时应淡化并提示数据年龄，不能继续把动画当成实时位置。')])

chapter('20 资源预算与实时约束',[
('20.1 固定内存','主控RAM限制使算法必须采用固定上界。704状态工作区、有限威胁记录、路线缓冲和实际路径栈占据主要算法存储；还要保留串口缓存、库函数、全局变量和栈空间，不能只统计搜索数组。\n\n最近映像RAM44896字节、剩余4256字节，已包含预留栈。剩余空间不算宽裕，增加每节点多个候选、全历史轨迹或大文本缓存时应重新查看map文件。避免把PC上的Python矩阵和对象图直接搬上主控。'),
('20.2 栈和CPU','链接器已知最大栈920字节低于预留3072字节，但静态调用图存在中断、未知函数和库边界，不能代替实测栈水位。CPU时间也应通过周期迟到和算法计算时间记录，而不是通过PC运行几毫秒推算STM32性能。\n\n风险搜索只在车体停稳的规划时机运行，普通行驶保持轻量控制。若未来搜索规模增大，可以分阶段预算或提前预计算拓扑，但应先测当前瓶颈，避免无收益地重构基础巡线。')])

chapter('21 离线 ARM 验证方法',[
('21.1 测试技术','测试使用pyelftools读取AXF中的ELF符号和DWARF类型，再由Unicorn执行实际ARM Thumb代码。通过明确的硬件钩子替换灰度读取、编码器增量、YAW帧、按键和电机物理响应，检查真正编译后的控制逻辑。\n\n这比只重写一个Python策略模型更接近固件执行，但仍不是电机、I2C时序和地面摩擦的数字孪生。若钩子直接把“停稳”“找到线”“到达节点”强制设成功，会掩盖真实bug，因此关键测试保留真实灰度分类与停稳判定。'),
('21.2 当前覆盖','当前16组套件覆盖藏点出口策略、N011首次方向锁、初始路线和停车姿态、转弯续接、激光边界、监控协议、实际路径回退、短回撤、后板接线与硬币短遮挡。每份报告记录映像SHA256，输入清单记录源码和夹具散列。\n\n硬币专项用120、220、260RPM三种入口目标与多种后板偏差检查平均速度保持、正向输出、50mm边界和双白停车。它证明软件目标计算，不保证实物在上述每个速度都能稳定通过金属反光。'),
('21.3 回归纪律','改策略要更新与用户新路线相关的预期，但不能直接删除失败断言；因路线改变而改变“前方”的测试，应按实际行进端重新选择探头。夹具自身错误需要与生产缺陷区分，并保留初次失败、修正内容和重跑结果。\n\n当前代码修改后应重新编译再跑测试；一份旧报告即使文件名相同，也不能证明新固件。源码、AXF、HEX、日志和验证JSON应作为同一批证据归档。')])
table(['套件','退出码','本稿用途'],[[s['test'],s['exit_code'],'以当前映像绑定报告为准'] for s in validation['suites']], [3.5,.6,2.8])

chapter('22 实车标定与性能试验计划',[
('22.1 基础标定','优先逐轮检查正负方向与编码器，再测直线有效轮周长，记录不同电池电压和负载下速度跟踪。随后单独测试前后灰度位序、离地高度、黑线宽度、中心误差和边缘形态。\n\n四路激光应分别做已知距离对照，至少记录目标材质、角度、原始值、有效率、均值和波动。频率统计应数新鲜完整帧，不按网页刷新的次数。标定结果应注明测量工具和误差，而不是只说“八成准确”。'),
('22.2 动作试验','普通转弯、停车90°、短回撤、路口直通、宝藏文字和硬币反光应分组，先固定一项条件再改变速度。对每次试验记录固件ID、源散列、电池状态、起点姿态、期望节点、实际终点、故障时刻和视频。\n\n可统计每类成功次数、失败类型和用时分布；无实测前不填成功率99%、定位误差几毫米或最短逃离时间。车身中心是否在路口、前后板是否压线，应由照片或明确现场记录支持。'),
('22.3 竞赛综合试验','在各藏点分别从地图四方向放置追车，验证首次触发、边缘外物体过滤与途中拦截。重复运行应改变敌车到达时机，尤其在入中心、制动和转弯结束附近放置障碍。\n\n最终报告应区分软件逻辑通过率和对抗成功率。两车速度、起始距离、对手策略与地图条件不同，不能只根据一次逃离成功就声称算法优于所有其他方法。')])

chapter('23 GitHub项目与开源技术来源',[
('23.1 用户自己的仓库','已通过关联GitHub身份确认账号为aaayolo-ui，并公开核对到Robot_Treasure仓库。它是本项目的公开来源入口，仓库页面描述为2026江苏省大学生机器人大赛密室夺宝项目。[W1]\n\n公开README仍包含单激光阶段的历史描述，与本地四路和多次策略修改不同。本稿不声称本地TOP7_SPEED已经推送到GitHub，也不把公开仓库的旧阶段说明覆盖当前代码。未看到明确仓库许可时，“公开可浏览”不能直接写成“任意复制商用的开源授权”。'),
('23.2 实际使用的基础软件','STM32CubeF1/HAL及CMSIS构成固件底层基础。上游STM32CubeF1许可清单区分HAL和CMSIS等组件，不能把整个软件包统称为同一种许可证；实际分发应保留对应版本文件头与LICENSE。[W2][W3]\n\nUnicorn和pyelftools用于电脑侧离线测试，不在主控上运行；它们的职责分别是执行ARM指令和解析ELF/DWARF。若公开测试工具或打包依赖，应按各仓库实际许可证保存通知，不能把测试工具许可等同于整车应用许可。[W4][W5]'),
('23.3 可研究但未移植的资料','PythonRobotics提供机器人算法示例，可用于理解搜索和控制的对照实验；zhm-real/PathPlanning提供路径规划动画与示例。它们主要是PC上的算法学习资源，没有证明其硬件、实时性和内存与本车吻合，当前工程未直接移植这些仓库。[W6][W7]\n\nRed Blob Games的路径搜索教程可作为BFS、Dijkstra、A星概念的解释来源。[W8] 参考资料有助于阐释算法，但不能把本地无启发式优先队列的有界搜索包装成已经实现完整A星、SIPP、SLAM或MPC。')])
refs=[('W1','用户公开项目仓库','https://github.com/aaayolo-ui/Robot_Treasure','项目来源与历史硬件说明  与本地版本分别记录'),('W2','STM32CubeF1 上游','https://github.com/STMicroelectronics/STM32CubeF1','HAL CMSIS等组件包  实际采用基础'),('W3','CMSIS 5 上游','https://github.com/ARM-software/CMSIS_5','Cortex M基础接口  实际工程基础'),('W4','Unicorn 上游','https://github.com/unicorn-engine/unicorn','PC侧ARM离线执行  非车载运行'),('W5','pyelftools 上游','https://github.com/eliben/pyelftools','PC侧ELF DWARF解析'),('W6','PythonRobotics','https://github.com/AtsushiSakai/PythonRobotics','可参考 未直接采用'),('W7','PathPlanning','https://github.com/zhm-real/PathPlanning','可参考 未直接采用'),('W8','A星及图搜索教程','https://www.redblobgames.com/pathfinding/a-star/introduction.html','理论说明 未作为固件库直接导入'),('W9','STM32CubeF1许可清单','https://github.com/STMicroelectronics/STM32CubeF1/blob/master/LICENSE.md','逐组件许可核对入口')]
for ident,title,url,note in refs:p(f'[{ident}] {title}。{note}。核对日期2026年10月30日。{url}')

chapter('24 可扩展方向与未采用方案',[
('24.1 定位和策略改进','若要解决真实节点错位，可以先围绕最近可靠节点、实际出口和下一路口特征保留少量候选，而不是立即引入大规模概率滤波。任何候选算法都要避免将重复采样当成经过多个路口，并在资源内核对最坏工作量。\n\n风险搜索未来可在PC离线比较BFS、Dijkstra或A星作基线，但对本车的收益需要实测：更短路径未必更少转弯、更低风险或更快逃离。不能为了“用高级算法”而扩大控制层改动。'),
('24.2 可观测性和工程化','优先补充灰度板和激光几何尺寸、轮周长、转弯耗时分布、串口周期迟到以及电池变化数据。把关键参数抽成有单位的集中配置，能减少注释与真实宏值不一致。\n\n监控端可加入固件ID与日志绑定、当前COM占用状态、遥测年龄、地图估计有效性和事件定位跳转。所有显示优化都应保留原始串口，不把页面状态控制误当成车辆控制。'),
('24.3 正式技术报告的补充','待有学校或竞赛模板，可把本稿的硬件、算法、策略与实验章节映射为摘要、总体设计、硬件设计、软件设计、实验结果、创新点和参考文献。当前未验证的目标值保持“待测”，不得为凑版面虚构曲线。\n\n建议补充整车照片、最终接线图、机械图、队员分工、版本日期、原理图、功耗和BOM价格。只有获得权利和明确许可后，才把第三方图纸、资料图片和代码大量复制到公开版报告。')])

h('附录A 当前关键参数原文')
p('重要差异：本次实际读取THIEF_LASER_TRIGGER_MM为900U，THIEF_LASER_MAX_MM仍为1000U。FrontLaserGate_Observe主要使用THIEF_LASER_CHANNEL_MAX_MM(direction)，本次检查的门控调用并未直接使用TRIGGER宏。因此单独把TRIGGER改900不能作为前后实际900mm触发已经生效的证据；最近验证报告仍属于改动前映像。本文仅记录并解释该差异，没有修改工程或重新编译。')
p('以下由当前本地文件直接提取。宏名中的X10/X100/X1000是缩放单位；复杂表达式、历史注释或未启用宏不等于当前运行一定使用。实际分支仍应查调用链。修改参数后需重新构建和验证。')
param_files=['Control/LineFollowPID.h','Algorithm/ThiefStrategy/ThiefStrategyConfig.h','Hardware/Encoder.h','Algorithm/ThiefStrategy/ThiefRouteTrail.h','User/App_ThiefStrategy.c','Control/FollowSmoothing.c']
for n in param_files:
    h(n,2);rows=[]
    for i,line in enumerate(read(n).splitlines(),1):
        m=re.match(r'\s*#define\s+([A-Za-z_][A-Za-z_0-9]*)(?:\s+)(.*)',line)
        if not m:continue
        value=m[2].split('/*')[0].strip()
        if len(value)>110:value=value[:107]+'…'
        rows.append((m[1],value,i))
    table(['宏定义','当前值或表达式','行'],rows,[3.6,2.8,.5])

h('附录B 地图节点坐标')
p('坐标由x_half和y_half乘150mm得到；节点类型数字沿用代码，不在此擅自重命名。N005为内部标记，类型0；道路数量以源表为准。')
table(['节点','X mm','Y mm','类型','度数'],[(f'N{n:03}',x*150,y*150,k,d) for n,x,y,k,d in nodes],[1.2,1.5,1.5,1.3,1.4])
h('附录C 地图道路与方向')
p('方向0北、1东、2南、3西；表中方向为起点到终点，反向行驶方向需加2再模4。没有列出的边不能凭地图图形接近就当成直接连通。')
table(['道路','起点','终点','长度 mm','方向'],[(f'S{n:03}',f'N{a:03}',f'N{b:03}',l,['北','东','南','西'][h]) for n,a,b,l,h in edges],[1,1.2,1.2,1.8,1.7])

h('附录D 源码模块与入口索引')
desc={'LineFollowPID':'循线外环位置PID与航向P','FollowSmoothing':'速度与差速目标平滑','TravelFrame':'前后行进端统一映射','RestartLineSearch':'有界找线与后板回退恢复','ThiefRiskSearch':'有限状态风险搜索和藏点评分','ThiefMapPlanner':'路线规则与动作段构造','ThiefRouteTrail':'实际执行历史与逆序返回','ThiefThreatModel':'回波威胁记忆与到达余量','FrontLaserGate':'距离资格和触发门控','ThiefLocalization':'地图节点观察诊断','MapRouteNavigator':'道路合法性和路口推进','MapLaserMask':'地图方向与场外过滤','GrayPosition':'十二路位图的位置特征','GrayCoordinate':'板坐标到车辆误差','WheelSpeedPi':'轮速内环调节','App_ThiefStrategy':'小偷任务与行驶状态机','MapRoutePosition':'轮程与毫米进度','Encoder':'编码器计数和轮速换算','LaserRx':'四路激光链路入口','JY61P':'陀螺仪数据接口','PowerSoundLight':'PA3持续声光','TreasureWheelLog':'受控轮速快照与停车后导出','RoadHeadingControl':'停车朝向控制','ThiefInterceptPolicy':'藏点轴向与出口规则'}
for folder in ['User','Control','Algorithm','Hardware','System']:
    h(folder,2);rows=[]
    for path in sorted((P/folder).rglob('*.c')):
        txt=path.read_text(encoding='utf-8',errors='replace');names=re.findall(r'^\s*(?:static\s+)?(?:void|uint8_t|uint16_t|uint32_t|int16_t|int32_t)\s+([A-Za-z_]\w*)\s*\(',txt,re.M)
        rel=path.relative_to(P).as_posix();rows.append((rel,desc.get(path.stem,'工程辅助模块  以实际调用链确认'),', '.join(names[:3])))
    table(['文件','职责','函数示例'],rows,[2.6,1.9,2.4])

h('附录E 关键实现摘录')
for name,begin,end in [('状态编码','Algorithm/ThiefStrategy/ThiefRiskSearch.c','static uint16_t State('),('原速短遮挡','User/App_ThiefStrategy.c','static uint8_t Route_TreasureRearBridge('),('首次路线','Algorithm/ThiefStrategy/ThiefMapPlanner.c','static const uint16_t initial_treasure_path[]')]:
    h(name,2);txt=read(begin);i=txt.index(end);lines=txt[i:].splitlines();limit=9 if name=='状态编码' else (36 if name=='原速短遮挡' else 30);code('\n'.join(lines[:limit]));p('出处 '+begin)

h('附录F 构建与复现实验清单')
p('构建入口：MDK-ARM/Thief16V_UARTCh4_RouteOnly_Test.uvprojx。当前验证入口Tests/run_speed.py与Tests/validate_speed.py。旧run_route6、run_top7等脚本保留历史，不可把其旧身份断言作为当前总验收。')
table(['步骤','应记录的证据'],[['备份','只针对指定E盘工程 建立源文件和旧映像散列'],['全量构建','ARMCC5日志 错误数 警告数 map和htm'],['输入绑定','source_hashes test_source_hashes AXF SHA256'],['离线回归','每组退出码 实际病例与失败内容'],['主控烧录','人工确认工程 HEX和BOOT身份'],['基础实测','逐轮方向 灰度位序 YAW符号及帧年龄'],['路线实测','N035到N007 经过宝藏 N002入N011 横停'],['对抗实测','各藏点四方向触发 途中前方拦截 旧证据清除'],['归档','日志 视频 电池状态 修改原因与是否允许回退']], [1.1,5.8])
p('当前验证清单仅证明已记录的编译与离线案例，未自动执行实物烧录。正式报告的实验结果表应在人工实测后填写。')
h('附录G 需要补齐的数据')
table(['待补数据','用途或原因'],[['前后灰度中心间距和安装偏移','几何航向模型与居中距离标定'],['各探头中心偏移和光轴高度','回波落点与场外过滤'],['有效轮周长和编码器每圈实测','毫米进度与速度一致性'],['不同电压下实际转弯耗时','风险模型时间标定'],['各方向测距误差和有效帧率','阈值与新鲜度选择'],['黑线宽度与金属反光RAW序列','分类器和短遮挡范围评估'],['完整规则与复位条件','确认策略适用边界'],['重复试验次数与失败率','支撑性能评价'],['用户GitHub当前提交号与LICENSE','对齐公开版本及许可'],['硬件最终BOM与图纸','把历史选型变成可复现硬件']], [2.6,4.3])
p('文档结束。后续可按正式模板删减或重排，所有未实测指标继续保留待测状态。')

output=OUT/'密室夺宝机器人技术文档_内容扩展草稿_D1.docx'
doc.core_properties.title='密室夺宝机器人系统技术文档';doc.core_properties.subject='控制算法 路径规划 逃跑策略 遥测与验证'
doc.core_properties.author='Robot Treasure 项目技术整理';doc.core_properties.keywords='STM32 NCHD12 PID 风险搜索 四路激光'
doc.save(output)
alltext='\n'.join(p.text for p in doc.paragraphs)+'\n'+'\n'.join(c.text for t in doc.tables for row in t.rows for c in row.cells)
(QA/'文档文本.txt').write_text(alltext,encoding='utf-8')
meta=dict(output=str(output),characters=len(alltext),paragraphs=len(doc.paragraphs),tables=len(doc.tables),nodes=len(nodes),edges=len(edges),firmware=validation['firmware'],source_mismatches=diff,axf_sha256=validation['axf_sha256'],references=refs)
(QA/'文档生成记录.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(meta,ensure_ascii=False,indent=2))

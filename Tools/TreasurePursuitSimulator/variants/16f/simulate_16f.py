"""16F: unchanged 16E strategies with continuous car pose/body contact.

No STM32 files or earlier replay versions are written by this module.
The new arm_adapter compiles the existing 16E C sources into local build/.
"""
from pathlib import Path
import copy, importlib.util, json, math, sys

HERE = Path(__file__).resolve().parent
SIM = HERE.parents[1]
sys.path.insert(0,str(SIM))
sys.path.insert(0,str(HERE))
import simulate as base
from arm_adapter import build
from body_geometry import Body, separation, rectangle, contact_point, first_contact
from body_motion import BodyRobot

spec = importlib.util.spec_from_file_location('firmware_baseline_16e',HERE.with_name('16e')/'simulate_16e.py')
firmware = importlib.util.module_from_spec(spec)
spec.loader.exec_module(firmware)
NODE, ANNOTATIONS, parameter = firmware.NODE, firmware.ANNOTATIONS, firmware.parameter


def load_config():
    model=json.loads((HERE/'body_model.json').read_text(encoding='utf-8'))
    cfg=json.loads((SIM/'config.json').read_text(encoding='utf-8'))
    cfg.update(thief_speed_m_s=parameter('THIEF_FORWARD_MM_S')/1000,
        police_speed_m_s=parameter('THIEF_FORWARD_MM_S')/1000,
        turn90_s=parameter('THIEF_TURN_EXTRA_MS')/1000,
        uturn_s=parameter('THIEF_SWITCH_TO_REV_MS')/1000,
        frame_period_s=model['frame_period_s'])
    cfg['measured_motion'].update(reverse_speed_ratio=parameter('THIEF_REVERSE_MM_S')/parameter('THIEF_FORWARD_MM_S'),
        forward_to_reverse_s=parameter('THIEF_SWITCH_TO_REV_MS')/1000,
        reverse_to_forward_s=parameter('THIEF_SWITCH_TO_FWD_MS')/1000)
    if not math.isfinite(model['frame_period_s']) or model['frame_period_s'] <= 0:
        raise ValueError('Invalid replay period')
    return cfg,model


class BodyGame(firmware.FirmwareGame):
    def __init__(self,track,config,elf,model,stage=None,seed=None):
        super().__init__(track,config,elf,stage,seed)
        self.model=model; self.collision=None
        self.bodies={role:Body(**values) for role,values in model['bodies'].items()}
        self.robots={role:BodyRobot(**vars(robot)).configure(self.bodies[role],model['turn'])
                     for role,robot in self.robots.items()}

    def contact_delay(self,maximum_dt):
        p,b=self.robots['P'],self.robots['B']
        return first_contact(self.bodies['P'],lambda t:p.pose_at(self.track,t),p.corner_speed_bound(self.track),
            self.bodies['B'],lambda t:b.pose_at(self.track,t),b.corner_speed_bound(self.track),
            maximum_dt,self.model['collision'])

    def record(self):
        super().record()
        p,b=self.robots['P'],self.robots['B']
        pp,bp=p.pose_at(self.track),b.pose_at(self.track)
        f=self.frames[-1]
        # Preserve existing frame indices 0..32, but keep body geometry more
        # precise than the earlier 0.0001-grid rounded centre coordinates.
        f[0]=round(self.now,8)
        f[2:4]=[round(pp.x/self.track.spacing,9),round(pp.y/self.track.spacing,9)]
        f[9:11]=[round(bp.x/self.track.spacing,9),round(bp.y/self.track.spacing,9)]
        f.extend([round(pp.angle,11),round(bp.angle,11),p.stage(),b.stage(),
            round(separation(self.bodies['P'],pp,self.bodies['B'],bp),10)])

    def capture(self,delay):
        for actor in self.robots.values():actor.advance(self.track,delay)
        self.now+=delay
        p,b=self.robots['P'].pose_at(self.track),self.robots['B'].pose_at(self.track)
        gap=separation(self.bodies['P'],p,self.bodies['B'],b)
        assert gap <= self.model['collision']['contact_tolerance_m']+1e-10,gap
        self.collision={'time_s':self.now,'geometry':'oriented_rectangles',
            'contact_point_m':contact_point(self.bodies['P'],p,self.bodies['B'],b),
            'separation_m':gap,'tolerance_m':self.model['collision']['contact_tolerance_m'],
            'police_polygon_m':rectangle(self.bodies['P'],p),
            'thief_polygon_m':rectangle(self.bodies['B'],b),
            'body_dimensions_status':self.model['dimensions_status']}
        self.winner='police'
        self.end_reason='capture_before_pickup' if self.owner is None else 'capture_after_pickup'
        if self.owner=='B':self.owner='P'
        self.events.append({'t':self.now,'event':'capture','geometry':'oriented_rectangles',
            'police_xy':self.robots['P'].xy(self.track),'thief_xy':self.robots['B'].xy(self.track),
            'contact_point_m':self.collision['contact_point_m'],'separation_m':gap})

    def run(self):
        # Same settle/ownership/decision/sensor order as base.Game.run.
        # Only the referee's continuous point-distance calls are replaced.
        self.observe()
        next_sensor,next_frame=self.config['sensor_period_s'],0.
        while self.winner is None and self.now < self.config['time_limit_s']-base.EPS:
            if self.contact_delay(0) is not None:self.capture(0);break
            self.settle()
            if self.winner:break
            self.command_ready();self.settle()
            if self.winner:break
            if any(r.phase=='ready' for r in self.robots.values()):continue
            if self.now >= next_frame-base.EPS:
                self.record();next_frame+=self.config['frame_period_s']
            pending=[r.remaining for r in self.robots.values() if r.phase!='ready' and r.remaining>base.EPS]
            dt=min([self.config['time_limit_s']-self.now,max(base.EPS,next_sensor-self.now)]+pending)
            delay=self.contact_delay(dt)
            if delay is not None:self.capture(delay);break
            for actor in self.robots.values():actor.advance(self.track,dt)
            self.now+=dt
            if self.now >= next_sensor-base.EPS:
                self.observe();next_sensor+=self.config['sensor_period_s']
        self.settle()
        if self.winner is None:
            self.winner,self.end_reason=('thief','survived_timeout') if self.owner=='B' else ('draw','no_treasure_timeout')
        self.record()
        result={k:getattr(self,k) for k in ('police_policy','thief_policy','escape_mode','vision','winner',
            'end_reason','pickup_times','hide_decisions','escape_plans','response_events',
            'navigation_events','search_sweeps','frames','events','planner_events')}
        result.update(sensor_range_m=self.config['sensor_range_m'],
            police_sensor_range_m=self.config['police_sensor_range_m'],
            police_speed_m_s=self.config['police_speed_m_s'],thief_speed_m_s=self.config['thief_speed_m_s'],
            duration_s=self.now,traveled_m={r:round(a.traveled_m,5) for r,a in self.robots.items()},
            max_escape_straight_m=round(self.robots['B'].max_escape_straight_m,5),
            deadend_checks=self.deadend_checked,collision=self.collision)
        return result


class TurningFixture(BodyGame):
    """A static near-car, hit by a turning corner; not a strategy match."""
    def __init__(self,track,config,elf,model):
        super().__init__(track,config,elf,model,(53,'N',54,'W'))
        self.planner_events=[];self.events=[];self.escape_plans=[];self.escape_plan_index=-1
        self.active_escape_route=None;self.firmware_hiding=False
        p,b=self.robots['P'],self.robots['B']
        p.phase='move';p.target=b.node;p.progress=(.3-.228)/.3
        p.move_speed=0.;p.remaining=3.
        b.command(track,self.routing,NODE[54]['original_node'],'relocate')

    def observe(self):pass


def main():
    cfg,model=load_config();elf,hashes=build();track=base.TrackMap()
    cases=[('16F full / equal speed',1,None),('16F full / police +20%',1.2,None),
        ('16F N023 close approach',1,(23,'S',31,'N')),
        ('16F N059 side approach',1,(59,'W',75,'N')),
        ('Geometry only / rotating corner contact',1,'turn_demo')]
    labels=['完整开局 · 双方同速 · 车身碰撞','完整开局 · 警方快20% · 车身碰撞',
        '藏点专项 · N023近距来车 · 车身碰撞','藏点专项 · N059侧方来车 · 车身碰撞',
        '仅模型验证 · 原地转向扫到旁车']
    replays=[];summary=[]
    for case,label in zip(cases,labels):
        name,factor,stage=case;config=copy.deepcopy(cfg);config['police_speed_m_s']*=factor
        game=(TurningFixture(track,config,elf,model) if stage=='turn_demo'
              else BodyGame(track,config,elf,model,stage))
        result=game.run();result.update(label=label,fixture=stage,geometry_demo=stage=='turn_demo')
        result['metrics']=base.motion_metrics(result)
        result['motion_parameters']=config['measured_motion']|{'turn90_s':config['turn90_s']}
        replays.append(result)
        if stage!='turn_demo':
            summary.append({'scenario':label,'police_speed_m_s':config['police_speed_m_s'],
                'thief_speed_m_s':config['thief_speed_m_s'],'runs':1,
                'police_wins':int(result['winner']=='police'),'thief_wins':int(result['winner']=='thief'),
                'mean_duration_s':result['duration_s'],'mean_max_escape_straight_m':result['max_escape_straight_m']})
        print(name,result['winner'],round(result['duration_s'],6),'body_contact',bool(result['collision']),flush=True)
    report={'map':track.data,'config':cfg,'annotations':ANNOTATIONS,'replays':replays,'summary':summary,
        'source_hashes':hashes,'firmware':'Thief16E_VisibilityEscape_FourLaser_Test',
        'body_model':model,'frame_schema_extra':{'33':'police_angle_rad','34':'thief_angle_rad',
            '35':'police_motion_stage','36':'thief_motion_stage','37':'body_separation_m'},
        'scope':'Same 16E C strategy and limited sensors; continuous body rotation and rectangle contact referee. '
                'Fixed turn centre; no measured re-line displacement, inertial dynamics or tyre slip.'}
    (HERE/'report.json').write_text(json.dumps(report,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
    (HERE/'map_annotations.json').write_text(json.dumps(ANNOTATIONS,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    template=(HERE/'replay.template.html').read_text(encoding='utf-8')
    payload=json.dumps(report,ensure_ascii=False,separators=(',',':')).replace('</','<\\/')
    html=template.replace('__REPORT_DATA__',payload).replace('__ANNOTATION_SCRIPT__',(HERE/'map_annotations.js').read_text(encoding='utf-8'))
    (HERE/'replay.html').write_text(html,encoding='utf-8')


if __name__=='__main__':
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    main()

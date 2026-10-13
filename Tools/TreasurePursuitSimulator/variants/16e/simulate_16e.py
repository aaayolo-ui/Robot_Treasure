"""16E C-planner replay using the existing road physics and police policy."""
from pathlib import Path
import copy,hashlib,json,math,re,sys
HERE=Path(__file__).resolve().parent
SIM=HERE.parents[1]
sys.path.insert(0,str(SIM))
import simulate as base
from arm_adapter import Planner,build,FIRMWARE

ANNOTATIONS=json.loads((SIM/'map_annotations.json').read_text(encoding='utf-8'))
NODE={int(n['id'][1:]):n for n in ANNOTATIONS['nodes']}
ID={n['original_node']:i for i,n in NODE.items()}
SEGMENTS=ANNOTATIONS['segments']
CHANNELS=('front','left','right','rear')

def parameter(name):
    s=(FIRMWARE/'Algorithm/ThiefStrategy/ThiefStrategyConfig.h').read_text(encoding='utf-8')
    return int(re.search(r'#define\s+'+name+r'\s+(\d+)',s).group(1))

def expand(plan):
    nodes=[NODE[plan['from'][0]]['original_node']]
    for a,b,h,length in zip(plan['from'],plan['to'],plan['step_heading'],plan['length']):
        current=a;total=0
        for _ in range(106):
            if current==b:break
            found=None
            for seg in SEGMENTS:
                start,end=int(seg['start'][1:]),int(seg['end'][1:])
                if start==current and seg['direction_from_start']==base.DIRECTIONS[h]:
                    found=(end,seg['original_nodes'],seg['length_m']);break
                if end==current and seg['direction_from_end']==base.DIRECTIONS[h]:
                    found=(start,list(reversed(seg['original_nodes'])),seg['length_m']);break
            if found is None:raise ValueError(('Missing C segment',a,b,h,current))
            current,raw,mm=found;nodes.extend(raw[1:]);total+=round(mm*1000)
        assert current==b and total==length,(a,b,length,total)
    return nodes

class FirmwareGame(base.Game):
    def __init__(self,track,config,elf,stage=None,seed=None):
        super().__init__(track,config,thief_policy='refuge',seed=seed)
        self.c=Planner(elf)
        initial=self.c.initial()
        self.queue=[];self.firmware_hiding=False;self.firmware_fault=False
        self.settled_at=None;self.last_hide_id=0;self.previous_hide=0
        self.planner_events=[];self.laser_enabled=False
        self.laser_samples=[5000]*4;self.laser_status=[0]*4
        self.stage=stage
        if stage:
            bn,bh,pn,ph=stage
            b=self.robots['B'];p=self.robots['P']
            b.node=NODE[bn]['original_node'];b.facing=bh;b.body_facing=bh
            p.node=NODE[pn]['original_node'];p.facing=ph;p.body_facing=ph
            self.owner='B';self.hide_goal=b.node;self.hide_facing=bh
            self.firmware_hiding=True;self.settled_at=-.08;self.last_hide_id=bn
            b.phase='hide';b.remaining=.02;b.reason='hide'
            self.c.clear_gate()
            self.hide_decisions.append({'t':0,'node':b.node,'node_id':f'N{bn:03d}',
                'escape_exits':[],'source':'stage_fixture'})
            self.events.append({'t':0,'event':'fixture','role':'B','node':b.node})
        else:
            self.install(initial,'initial')

    def install(self,plan,kind):
        path=expand(plan)
        b=self.robots['B'];assert path[0]==b.node,(path[0],b.node)
        self.queue=path[1:];self.hide_goal=NODE[plan['goal']]['original_node']
        self.hide_facing=base.DIRECTIONS[(plan['step_heading'][-1]+2*plan['step_mode'][-1])%4]
        # The firmware starts from rest; its first reverse leg does not incur
        # an additional moving forward-to-reverse delay.
        body=b.body_facing or b.facing
        b.facing=base.DIRECTIONS[plan['heading']];b.body_facing=body
        b.phase='wait';b.remaining=parameter('THIEF_STARTUP_MS')/1000
        b.reason='seek_hide' if kind=='initial' else 'relocate'
        b.target=None
        self.firmware_hiding=False;self.settled_at=None
        self.c.clear_gate()
        self.active_escape_route=path;self.escape_plan_index=len(self.escape_plans)
        self.escape_plans.append({'t':round(self.now,4),'mode':'relocate','nodes':path,
            'topology':{'branch_count':sum(len(self.track.graph[n])>=3 for n in set(path))},
            'firmware_decision':plan,'source':'16E ARM C'})
        self.hide_decisions.append({'t':round(self.now,4),'node':self.hide_goal,
            'node_id':f"N{plan['goal']:03d}",'source':'16E ARM C','escape_exits':[]})
        self.route_purpose='hide';self.route_goal=self.hide_goal
        self.planner_events.append({'t':round(self.now,4),'kind':kind,'source':'16E ARM C',
            'goal':f"N{plan['goal']:03d}",'plan':plan,'original_route':path})

    def decide(self,role):
        if role=='P':return super().decide(role)
        b=self.robots['B']
        if self.firmware_fault:return None,'hold',b.node
        if self.queue:
            neighbor=self.queue.pop(0)
            return neighbor,'treasure' if self.owner!='B' else 'relocate',self.hide_goal
        if not self.firmware_hiding:
            self.firmware_hiding=True;self.settled_at=self.now
            self.previous_hide=self.last_hide_id;self.last_hide_id=ID[b.node]
            self.c.clear_gate()
            self.events.append({'t':round(self.now,4),'event':'firmware_hide','role':'B','node':b.node})
        return None,'hide',self.hide_goal

    def observe(self):
        p,b=self.robots['P'],self.robots['B']
        belief=self.beliefs['P']
        # Preserve the existing police limited-observation policy. Only this
        # simulated sensor may use world truth; the C planner never receives it.
        belief.observe(p,b.xy(self.track),self.now,self.vision,self.routing,b.speed,self.rngs['P'])
        self.update_search_coverage()
        if belief.seen:
            if not self.threat_active['P']:
                self.events.append({'t':round(self.now,4),'event':'detected','role':'P',
                    'measured_xy':belief.last_xy,'phase':p.phase})
                self.pending_response['P']=self.now
            self.threat_active['P']=True;self.plans.pop('P',None)
            self.remember_police_trail(p,belief);p.reason='chase'
            if p.phase in ('hide','wait','pickup'):p.phase='ready';p.remaining=0;p.target=None
        elif self.now-belief.last_time>self.config['pursuit_memory_s']:
            self.threat_active['P']=False
        self.laser_enabled=(self.firmware_hiding and not self.firmware_fault
                            and self.settled_at is not None and self.now-self.settled_at>=.08-base.EPS)
        self.beliefs['B'].seen=False;self.beliefs['B'].sensor=None
        if not self.laser_enabled:return
        distances=[5000]*4;conf=[20]*4;fresh=15
        other=p.xy(self.track)
        if base.detectable(b.xy(self.track),other,self.config,self.track.spacing,'B'):
            channel=CHANNELS.index(base.sensor_channel(b,other,self.track))
            if self.rngs['B'].random()>=self.config['miss_probability']:
                distances[channel]=round(math.dist(b.xy(self.track),other)*self.track.spacing*1000)
            else:fresh&=~(1<<channel)
        self.laser_samples=distances
        result=self.c.observe(ID[b.node],base.DIRECTIONS.index(b.body_facing or b.facing),
            round(self.now*1000),distances,conf,fresh,self.previous_hide)
        self.laser_status=result['status']
        self.beliefs['B'].seen=bool(result['mask'])
        if result['mask']:
            accepted=next((i for i,s in enumerate(result['status']) if s==7),0)
            self.beliefs['B'].sensor=CHANNELS[accepted]
        if not result['requested']:return
        self.events.append({'t':round(self.now,4),'event':'detected','role':'B',
            'phase':b.phase,'mask':result['mask'],'distances_mm':distances,'source':'16E C gate'})
        if result['ok']:
            self.install(result['plan'],'escape')
        else:
            self.firmware_fault=True;self.laser_enabled=False
            b.phase='wait';b.remaining=.02;b.reason='hold'
            self.events.append({'t':round(self.now,4),'event':'firmware_fault','role':'B',
                'code':'ESCAPE_PLAN_FAILED','mask':result['mask']})
            self.planner_events.append({'t':round(self.now,4),'kind':'fault','code':'ESCAPE_PLAN_FAILED',
                'mask':result['mask'],'source':'16E ARM C'})

    def record(self):
        super().record()
        self.frames[-1].extend([int(self.firmware_fault),int(self.laser_enabled),
                               len(self.planner_events)-1,self.laser_samples[:],self.laser_status[:]])

def main():
    elf,hashes=build()
    cfg=json.loads((SIM/'config.json').read_text(encoding='utf-8'))
    cfg.update(thief_speed_m_s=parameter('THIEF_FORWARD_MM_S')/1000,
        police_speed_m_s=parameter('THIEF_FORWARD_MM_S')/1000,
        turn90_s=parameter('THIEF_TURN_EXTRA_MS')/1000,
        uturn_s=parameter('THIEF_SWITCH_TO_REV_MS')/1000)
    cfg['measured_motion'].update(reverse_speed_ratio=parameter('THIEF_REVERSE_MM_S')/parameter('THIEF_FORWARD_MM_S'),
        forward_to_reverse_s=parameter('THIEF_SWITCH_TO_REV_MS')/1000,
        reverse_to_forward_s=parameter('THIEF_SWITCH_TO_FWD_MS')/1000)
    cases=[('16E完整开局 · 双方同速',1,None),('16E完整开局 · 警方快20%',1.2,None),
           ('藏点专项 · N023近距来车',1,(23,'S',31,'N')),
           ('藏点专项 · N059侧方来车',1,(59,'W',75,'N'))]
    track=base.TrackMap();replays=[];summary=[]
    for label,factor,stage in cases:
        config=copy.deepcopy(cfg);config['police_speed_m_s']*=factor
        game=FirmwareGame(track,config,elf,stage)
        result=game.run();result['label']=label;result['planner_events']=game.planner_events
        result['metrics']=base.motion_metrics(result)
        result['fixture']=stage;result['motion_parameters']=config['measured_motion']|{'turn90_s':config['turn90_s']}
        replays.append(result)
        summary.append({'scenario':label,'police_speed_m_s':config['police_speed_m_s'],
            'thief_speed_m_s':config['thief_speed_m_s'],'runs':1,
            'police_wins':int(result['winner']=='police'),'thief_wins':int(result['winner']=='thief'),
            'mean_duration_s':result['duration_s'],'mean_max_escape_straight_m':result['max_escape_straight_m']})
        print(label,result['winner'],result['end_reason'],result['duration_s'],
              'decisions',[(x['kind'],x.get('goal',x.get('code'))) for x in game.planner_events])
    report={'map':track.data,'config':cfg,'annotations':ANNOTATIONS,'replays':replays,'summary':summary,
        'source_hashes':hashes,'firmware':'Thief16E_VisibilityEscape_FourLaser_Test',
        'scope':'Actual C planner and gate; ideal road physics/localization; police uses existing Python policy.'}
    report['comparison']=json.loads((HERE/'heldout_comparison.json').read_text(encoding='utf-8'))
    (HERE/'report.json').write_text(json.dumps(report,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
    (HERE/'map_annotations.json').write_text(json.dumps(ANNOTATIONS,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    template=(HERE/'replay.template.html').read_text(encoding='utf-8')
    payload=json.dumps(report,ensure_ascii=False,separators=(',',':')).replace('</','<\\/')
    html=template.replace('__REPORT_DATA__',payload).replace('__ANNOTATION_SCRIPT__',(SIM/'map_annotations.js').read_text(encoding='utf-8'))
    (HERE/'replay.html').write_text(html,encoding='utf-8')
if __name__=='__main__':
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    main()

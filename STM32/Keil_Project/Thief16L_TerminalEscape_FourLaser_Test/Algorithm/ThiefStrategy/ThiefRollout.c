#include "ThiefRollout.h"
#include "ThiefMapPlanner.h"
#include "ThiefThreatModel.h"
#include "ThiefStrategyConfig.h"
#include "ThiefMapTopology.h"
#include "ThiefRolloutData.h"
#include "ThiefRefugePolicy.h"
#include "ThiefTerminalForecast.h"
#include <stddef.h>
#include <string.h>

/* Bounded maximin over loop-free alternatives and finite-observation pursuit
 * models. These are hypotheses from a measured point, never world truth.
 * Tables are immutable Flash; runtime uses integer coordinates and milliseconds. */
typedef ThiefTerminalActor_t Actor;
typedef ThiefTerminalBox_t Box;
typedef struct {uint16_t contact,travel;int32_t escape,gap;uint8_t clear;} Score;
typedef struct {Actor own,police;uint8_t visits[88],valid;} Arrival;
typedef struct {
  uint16_t index,time,contact,runway,rank[THIEF_ROLLOUT_MODELS];
  int32_t escape,gap;
  uint8_t clear,cool,coverage,refuge;
} Candidate;
typedef struct {
  int32_t reserve;
  uint32_t steps;
  uint16_t wait;
  uint8_t known,clear,qualified,checked;
} Terminal;
static uint32_t exposed_at[88];
static uint8_t exposed[88];
static uint32_t model_steps;
static uint16_t model_count;
/* Main-loop stopped planning is sequential. Keep bounded temporary records
 * in RAM rather than nesting them on the unchanged 1KB Cortex-M3 stack.
 * Select's entry guard prevents recursive reuse of these scratch objects. */
static uint8_t select_active;
static Actor run_thief,run_police;
static Box run_b0,run_p0,run_b1,run_p1;
static uint8_t run_seen_count[88];
static Candidate normal,conditional,current;
static Terminal normal_terminal,conditional_terminal,selected_terminal;
static uint16_t best_rank[THIEF_ROLLOUT_MODELS];
static struct {
  uint16_t a,z,i,best,evaluated,max_shelter_delay,best_runway,best_contact,best_time,terminal_candidates;
  int16_t px,py;
  uint8_t best_clear,best_cool,best_coverage,best_refuge;
  int32_t best_escape,best_gap;
  uint32_t terminal_steps;
  uint16_t rank[THIEF_ROLLOUT_MODELS],bound[THIEF_ROLLOUT_MODELS];
  Score score;
} selector;
static Terminal terminal_result;
static Score terminal_transfer;
static ThiefTerminalScore_t terminal_score;
typedef struct {uint32_t last_clear,first_hit;uint16_t own,node;uint8_t clear_count,valid;} EntryEvidence;
static EntryEvidence entry[4];
static uint16_t entry_age;
static uint8_t entry_heading=4U;

/* Sorted, conservative contact-time bands. Once the earliest danger band
 * ties, preserve more complete-transfer hypotheses and more time before the
 * next mandatory corner. Coverage is not a measured probability of winning. */
static int8_t RiskOrder(const uint16_t *a,const uint16_t *b,uint8_t ac,uint8_t bc,
                        uint16_t ar,uint16_t br) {
  uint8_t k;
  if(THIEF_ROLLOUT_RISK_TIERS) {
    if(a[0]!=b[0]) return a[0]>b[0]?1:-1;
    if(ac!=bc) return ac>bc?1:-1;
    if(ar!=br) return ar>br?1:-1;
  }
  for(k=0U;k<THIEF_ROLLOUT_MODELS;k++) if(a[k]!=b[k]) return a[k]>b[k]?1:-1;
  return 0;
}

static int32_t Abs(int32_t x) {return x<0L?-x:x;}
static uint16_t S(uint16_t n,uint8_t h,uint8_t mode) {return (uint16_t)((n-1U)*8U+h*2U+mode);}
static uint32_t MoveMs(uint16_t mm,uint8_t mode,uint32_t speed) {
  if(mode) speed=speed*THIEF_REVERSE_MM_S/THIEF_FORWARD_MM_S;
  return ((uint32_t)mm*1000UL+speed-1UL)/speed;
}
static Box Pose(const Actor *a) {
  Box b;
  uint8_t body=(uint8_t)((a->h+2U*a->mode)%4U);
  b.x=map_route_nodes[a->node-1U].x_half*150L;
  b.y=map_route_nodes[a->node-1U].y_half*150L;
  if(a->phase==2U) {
    int32_t d=(int32_t)((uint32_t)a->length*a->elapsed/a->total);
    if(a->h==0U) b.y-=d;else if(a->h==1U) b.x+=d;
    else if(a->h==2U) b.y+=d;else b.x-=d;
  }
  b.hx=(body%2U)?95:99;b.hy=(body%2U)?99:95;
  if(a->phase==1U && a->turning) b.hx=b.hy=138; /* ceil half diagonal */
  return b;
}
static uint8_t Visible(const Box *a,const Box *b) {
  int32_t dx=Abs(a->x-b->x),dy=Abs(a->y-b->y),major=dx>dy?dx:dy,minor=dx>dy?dy:dx;
  return (uint8_t)(dx*dx+dy*dy<=(int32_t)(THIEF_LASER_MAX_MM*THIEF_LASER_MAX_MM) && minor*1000L<=major*177L);
}
static uint8_t Slab(int32_t d,int32_t v,int32_t r,int32_t *lo,int32_t *hi) {
  int32_t a,b,l,h;
  if(!v) return (uint8_t)(Abs(d)<=r);
  a=(-r-d)*1024L/v;b=(r-d)*1024L/v;l=(a<b?a:b)-1L;h=(a>b?a:b)+1L;
  if(l>*lo) *lo=l;if(h<*hi) *hi=h;
  return (uint8_t)(*lo<=*hi);
}
static int32_t ContactFraction(const Box *a0,const Box *a1,const Box *b0,const Box *b1) {
  int32_t lo=0L,hi=1024L;
  int32_t rx=(a0->hx>a1->hx?a0->hx:a1->hx)+(b0->hx>b1->hx?b0->hx:b1->hx)+2L;
  int32_t ry=(a0->hy>a1->hy?a0->hy:a1->hy)+(b0->hy>b1->hy?b0->hy:b1->hy)+2L;
  int32_t dx=a0->x-b0->x,dy=a0->y-b0->y;
  if(Slab(dx,(a1->x-b1->x)-dx,rx,&lo,&hi) &&
      Slab(dy,(a1->y-b1->y)-dy,ry,&lo,&hi)) return lo<0L?0L:lo;
  return -1L;
}
static int32_t FloorDiv(int32_t a,int32_t positive) {
  return a>=0L?a/positive:-((-a+positive-1L)/positive);
}
/* Feasible fraction 0..256 of a linear half-plane c+v*t >= 0. */
static uint8_t HalfPlane(int32_t c,int32_t v,int32_t *lo,int32_t *hi) {
  int32_t t;
  if(!v) return (uint8_t)(c>=0L);
  if(v>0L) {t=-FloorDiv(c*256L,v);if(t>*lo) *lo=t;}
  else {t=FloorDiv(c*256L,-v);if(t<*hi) *hi=t;}
  return (uint8_t)(*lo<=*hi);
}
static uint8_t InRange(int32_t x,int32_t y,int32_t vx,int32_t vy,int32_t t) {
  x+=vx*t/256L;y+=vy*t/256L;
  return (uint8_t)(x*x+y*y<=(int32_t)(THIEF_LASER_MAX_MM*THIEF_LASER_MAX_MM));
}
/* A committed move cannot change heading before its next phase endpoint.
 * Check the complete relative-motion interval for any cardinal laser view,
 * including a short glimpse that falls between coarse replay frames. */
static uint8_t VisibleDuring(const Box *a0,const Box *a1,const Box *b0,const Box *b1) {
  int32_t x=b0->x-a0->x,y=b0->y-a0->y;
  int32_t vx=(b1->x-a1->x)-x,vy=(b1->y-a1->y)-y;
  uint8_t h;
  if(Visible(a0,b0) || Visible(a1,b1)) return 1U;
  if((x>(int32_t)THIEF_LASER_MAX_MM && x+vx>(int32_t)THIEF_LASER_MAX_MM) ||
     (x<-(int32_t)THIEF_LASER_MAX_MM && x+vx<-(int32_t)THIEF_LASER_MAX_MM) ||
     (y>(int32_t)THIEF_LASER_MAX_MM && y+vy>(int32_t)THIEF_LASER_MAX_MM) ||
     (y<-(int32_t)THIEF_LASER_MAX_MM && y+vy<-(int32_t)THIEF_LASER_MAX_MM)) return 0U;
  for(h=0U;h<4U;h++) {
    int32_t u=h%2U?x:y,v=h%2U?y:x;
    int32_t du=h%2U?vx:vy,dv=h%2U?vy:vx,lo=0L,hi=256L,t;
    if(h==0U || h==3U) {u=-u;du=-du;}
    if(!HalfPlane(u,du,&lo,&hi) ||
       !HalfPlane(u*177L-v*1000L,du*177L-dv*1000L,&lo,&hi) ||
       !HalfPlane(u*177L+v*1000L,du*177L+dv*1000L,&lo,&hi)) continue;
    t=lo;
    if(vx || vy) {
      int64_t dot=(int64_t)x*vx+(int64_t)y*vy;
      int64_t squared=(int64_t)vx*vx+(int64_t)vy*vy;
      t=(int32_t)(-dot*256L/squared);if(t<lo)t=lo;if(t>hi)t=hi;
    }
    if(InRange(x,y,vx,vy,t) || (t<hi && InRange(x,y,vx,vy,t+1L))) return 1U;
  }
  return 0U;
}
static void Command(Actor *a,uint8_t out,uint32_t speed,uint32_t turn) {
  uint8_t d=(uint8_t)((out+4U-a->h)%4U);
  uint32_t wait=0UL;
  a->turning=(uint8_t)(d%2U);
  if(d==2U) a->mode^=1U;
  if(a->first==1U) {wait=0UL;a->first=0U;a->turning=0U;}
  else if(a->first==2U) {
    wait=d%2U?turn:d==2U?(a->mode?THIEF_SWITCH_TO_REV_MS:THIEF_SWITCH_TO_FWD_MS):0UL;
    wait=wait>entry_age?wait-entry_age:0UL;a->first=0U;
  }
  else if(d%2U) wait=turn;
  else if(d==2U) wait=a->mode?THIEF_SWITCH_TO_REV_MS:THIEF_SWITCH_TO_FWD_MS;
  a->h=out;a->to=rollout_next[a->node-1U][out];
  a->length=rollout_length[a->node-1U][out];a->elapsed=0U;
  a->phase=(uint8_t)(wait?1U:2U);
  a->total=(uint16_t)(wait?wait:MoveMs(a->length,a->mode,speed));
}
static void Advance(Actor *a,uint16_t dt,uint32_t speed) {
  if(a->phase==3U) return;
  a->elapsed=(uint16_t)(a->elapsed+dt);
  if(a->elapsed<a->total) return;
  a->elapsed=0U;
  if(a->phase==1U) {a->phase=2U;a->total=(uint16_t)MoveMs(a->length,a->mode,speed);}
  else if(a->phase==2U) {a->last=a->node;a->node=a->to;a->to=0U;a->phase=0U;}
}
static uint8_t Direction(uint16_t from,uint16_t to) {
  uint8_t h;
  for(h=0U;h<4U;h++) if(rollout_next[from-1U][h]==to) return h;
  return 4U;
}
static uint8_t BestPolice(uint16_t node,uint8_t heading,uint8_t mode,uint16_t goal,uint32_t speed) {
  uint32_t best=0xFFFFFFFFUL;uint8_t h,result=4U;
  for(h=0U;h<4U;h++) {
    uint16_t next=rollout_next[node-1U][h];uint8_t d=(uint8_t)((h+4U-heading)%4U),m;
    uint32_t cost;
    if(!next || (map_route_nodes[node-1U].kind==0U && d)) continue;
    m=(uint8_t)(mode^(d==2U));
    cost=MoveMs(rollout_length[node-1U][h],m,speed)+
      (d%2U?THIEF_ROLLOUT_POLICE_TURN_MS:d==2U?(m?THIEF_SWITCH_TO_REV_MS:THIEF_SWITCH_TO_FWD_MS):0UL)+
      rollout_police_ms[goal-1U][S(next,h,m)];
    if(cost<best) {best=cost;result=h;}
  }
  return result;
}
static uint8_t Seed(Actor *p,int16_t x,int16_t y,Box own,uint32_t speed,uint8_t bias) {
  uint16_t n;
  for(n=1U;n<=88U;n++) if(map_route_nodes[n-1U].x_half*150==x && map_route_nodes[n-1U].y_half*150==y) {
    memset(p,0,sizeof(*p));p->node=n;p->first=1U;
    p->h=(uint8_t)(Abs(own.x-x)>Abs(own.y-y)?(own.x>x?1U:3U):(own.y>y?2U:0U));
    if(entry_heading<4U) {
      uint8_t h,choices[2],count=0U;
      for(h=0U;h<4U;h++) if(h%2U!=entry_heading%2U && rollout_next[n-1U][h])
        choices[count++]=(uint8_t)((h+2U)%4U);
      if(count) {p->h=choices[bias%count];p->first=2U;}
    }
    p->last_seen_h=4U;return 1U;
  }
  for(n=0U;n<106U;n++) {
    const MapRouteSegment_t *e=&map_route_segments[n];
    int32_t ax=map_route_nodes[e->start-1U].x_half*150L,ay=map_route_nodes[e->start-1U].y_half*150L;
    int32_t bx=map_route_nodes[e->end-1U].x_half*150L,by=map_route_nodes[e->end-1U].y_half*150L;
    uint16_t progress=(uint16_t)(Abs(x-ax)+Abs(y-ay));uint8_t out=e->heading;
    if((ax==bx && x==ax && y>=(ay<by?ay:by) && y<=(ay>by?ay:by)) ||
       (ay==by && y==ay && x>=(ax<bx?ax:bx) && x<=(ax>bx?ax:bx))) {
      memset(p,0,sizeof(*p));p->first=0U;p->last_seen_h=4U;
      if(Abs(own.x-ax)+Abs(own.y-ay)<=Abs(own.x-bx)+Abs(own.y-by)) {
        p->node=e->end;out=(uint8_t)((out+2U)%4U);progress=(uint16_t)(e->length_mm-progress);
      } else p->node=e->start;
      p->h=out;p->to=rollout_next[p->node-1U][out];p->length=e->length_mm;p->phase=2U;
      p->total=(uint16_t)MoveMs(p->length,0U,speed);
      p->elapsed=(uint16_t)((uint32_t)progress*p->total/p->length);return 1U;
    }
  }
  return 0U;
}
static int32_t Reach(const Actor *p,uint16_t goal,uint32_t speed) {
  int32_t value=rollout_police_ms[goal-1U][S(p->node,p->h,p->mode)];
  if(p->phase==1U) value=(int32_t)(p->total-p->elapsed)+
      (int32_t)MoveMs(p->length,p->mode,speed)+rollout_police_ms[goal-1U][S(p->to,p->h,p->mode)];
  else if(p->phase==2U) value=(int32_t)(p->total-p->elapsed)+rollout_police_ms[goal-1U][S(p->to,p->h,p->mode)];
  return value;
}
static void ObservedGoal(Actor *p,const Actor *b,uint8_t branch_bias) {
  p->goal=b->phase==2U?b->to:b->node;
  if(b->phase==2U) p->last_seen_h=b->h;
  if(branch_bias>=2U && p->last_seen_h<4U) {
    uint16_t at=b->phase==2U?b->to:b->node;
    uint16_t ahead=rollout_next[at-1U][p->last_seen_h];
    p->goal=ahead?ahead:at;
  }
}
static Score Run(const RolloutRoute_t *r,uint8_t body,int16_t px,int16_t py,uint32_t speed,uint8_t branch_bias,Arrival *arrival) {
  Score score;
  uint32_t now=0UL,limit=(uint32_t)r->time_ms+THIEF_ROLLOUT_HOLD_MS;
  const uint8_t *path=&rollout_nodes[r->offset];
  memset(run_seen_count,0,sizeof(run_seen_count));
  memset(&run_thief,0,sizeof(run_thief));memset(&score,0,sizeof(score));run_thief.node=path[0];run_thief.h=body;run_thief.index=1U;
  if(arrival) arrival->valid=0U;
  run_thief.first=1U;score.travel=r->time_ms;score.contact=(uint16_t)limit;score.clear=1U;
  if(!Seed(&run_police,px,py,Pose(&run_thief),speed,branch_bias)) {score.clear=0U;score.contact=0U;score.escape=-6000L;return score;}
  while(now<limit && now<THIEF_ROLLOUT_MAX_MS) {
    uint16_t dt;uint8_t out;int32_t contact;
    if(run_thief.phase==0U) {
      if(run_thief.index>=r->count) {
        run_thief.phase=3U;
        if(arrival && !arrival->valid) {
          arrival->own=run_thief;arrival->police=run_police;
          memcpy(arrival->visits,run_seen_count,sizeof(run_seen_count));arrival->valid=1U;
        }
      }
      else {
        uint8_t initial=run_thief.first;
        out=Direction(run_thief.node,path[run_thief.index++]);
        Command(&run_thief,out,THIEF_FORWARD_MM_S,THIEF_TURN_EXTRA_MS);
        if(initial) {run_thief.phase=1U;run_thief.total=THIEF_STARTUP_MS;run_thief.turning=0U;}
      }
    }
    run_b0=Pose(&run_thief);run_p0=Pose(&run_police);
    if(Visible(&run_p0,&run_b0)) {
      /* Project only the observed movement direction. Reading the route's
       * next chosen turn here would give this hypothesis hidden intent. */
      ObservedGoal(&run_police,&run_thief,branch_bias);
    }
    if(run_police.phase==0U) {
      if(!run_police.goal) run_police.goal=run_thief.node; /* Initial sighting exists, including the root. */
      if(run_police.node==run_police.goal) {
        uint8_t best_visits=255U,k;
        out=4U;
        for(k=0U;k<4U;k++) {
          uint8_t h=(uint8_t)(((run_police.last_seen_h<4U?run_police.last_seen_h:run_police.h)+(branch_bias?4U-k:k))%4U);
          uint16_t next=rollout_next[run_police.node-1U][h];
          if(!next || next==run_police.last || (map_route_nodes[run_police.node-1U].kind==0U && h!=run_police.h)) continue;
          if(run_seen_count[next-1U]<best_visits) {best_visits=run_seen_count[next-1U];out=h;}
        }
        if(out==4U) out=(uint8_t)((run_police.h+2U)%4U);
      } else out=BestPolice(run_police.node,run_police.h,run_police.mode,run_police.goal,speed);
      if(out>3U || !rollout_next[run_police.node-1U][out]) break;
      if(run_seen_count[run_police.node-1U]<254U) run_seen_count[run_police.node-1U]++;
      Command(&run_police,out,speed,THIEF_ROLLOUT_POLICE_TURN_MS);
    }
    dt=(uint16_t)(limit-now);
    if(run_thief.phase!=3U && run_thief.total-run_thief.elapsed<dt) dt=(uint16_t)(run_thief.total-run_thief.elapsed);
    if(run_police.total-run_police.elapsed<dt) dt=(uint16_t)(run_police.total-run_police.elapsed);
    if(limit-now<dt) dt=(uint16_t)(limit-now);
    if(!dt) break;
    /* Endpoint pose before changing phases: the entire interval has constant
     * velocity, or stationary centre plus a conservative rotating-body hull. */
    run_thief.elapsed=(uint16_t)(run_thief.elapsed+dt);run_police.elapsed=(uint16_t)(run_police.elapsed+dt);
    run_b1=Pose(&run_thief);run_p1=Pose(&run_police);
    run_thief.elapsed=(uint16_t)(run_thief.elapsed-dt);run_police.elapsed=(uint16_t)(run_police.elapsed-dt);
    if(VisibleDuring(&run_p0,&run_p1,&run_b0,&run_b1)) ObservedGoal(&run_police,&run_thief,branch_bias);
    model_steps++;
    contact=ContactFraction(&run_b0,&run_b1,&run_p0,&run_p1);
    if(contact>=0L) {score.clear=0U;score.contact=(uint16_t)(now+(uint32_t)dt*contact/1024UL);break;}
    Advance(&run_thief,dt,THIEF_FORWARD_MM_S);Advance(&run_police,dt,speed);
    now+=dt;
  }
  if(now<limit && score.clear) {score.clear=0U;score.contact=(uint16_t)now;}
  score.escape=-6000L;
  {
    uint16_t goal=path[r->count-1U];uint8_t i;
    for(i=0U;i<2U;i++) {
      uint8_t h=(uint8_t)((r->body+2U*i)%4U);uint16_t branch=rollout_branch[goal-1U][h];
      int32_t value;
      if(!branch) continue;
      value=Reach(&run_police,branch,speed)-(int32_t)thief_exit_branch_ms[goal-1U][h]-
        (int32_t)(THIEF_CONTACT_MM*1000UL/speed)-(int32_t)THIEF_TIMING_GUARD_MS;
      if(value>score.escape) score.escape=value;
    }
  }
  run_b1=Pose(&run_thief);run_p1=Pose(&run_police);
  score.gap=Abs(run_b1.x-run_p1.x)+Abs(run_b1.y-run_p1.y);
  return score;
}
/* Callbacks share the original immutable map and committed-motion math.
 * The terminal module owns no second copy of the large travel-time table. */
static Box T_Pose(void *c,const Actor *a) {(void)c;return Pose(a);}
static uint8_t T_Visible(void *c,const Box *a,const Box *b) {(void)c;return Visible(a,b);}
static uint8_t T_View(void *c,const Box *a,const Box *b,const Box *p,const Box *q) {(void)c;return VisibleDuring(a,b,p,q);}
static int32_t T_Contact(void *c,const Box *a,const Box *b,const Box *p,const Box *q) {(void)c;return ContactFraction(a,b,p,q);}
static void T_Command(void *c,Actor *a,uint8_t h,uint32_t s,uint32_t t) {(void)c;Command(a,h,s,t);}
static void T_Advance(void *c,Actor *a,uint16_t dt,uint32_t s) {(void)c;Advance(a,dt,s);}
static uint8_t T_Guide(void *c,uint16_t n,uint8_t h,uint8_t m,uint16_t g,uint32_t s) {(void)c;return BestPolice(n,h,m,g,s);}
static uint16_t T_Next(void *c,uint16_t n,uint8_t h) {(void)c;return n && n<=88U && h<4U?rollout_next[n-1U][h]:0U;}
static uint16_t T_Length(void *c,uint16_t n,uint8_t h) {(void)c;return n && n<=88U && h<4U?rollout_length[n-1U][h]:0U;}
static uint16_t T_Branch(void *c,uint16_t n,uint8_t h) {(void)c;return n && n<=88U && h<4U?thief_external_branch[n-1U][h]:0U;}
static int32_t T_Reach(void *c,const Actor *a,uint16_t n,uint32_t s) {(void)c;return Reach(a,n,s);}
static uint8_t T_Kind(void *c,uint16_t n) {(void)c;return n && n<=88U?map_route_nodes[n-1U].kind:0U;}
static const ThiefTerminalOps_t terminal_ops={NULL,T_Pose,T_Visible,T_View,T_Contact,
    T_Command,T_Advance,T_Guide,T_Next,T_Length,T_Branch,T_Reach,T_Kind};
static const ThiefTerminalConfig_t terminal_config={THIEF_FORWARD_MM_S,THIEF_TURN_EXTRA_MS,
    THIEF_HIDE_REACTION_MS,THIEF_TIMING_GUARD_MS,THIEF_CONTACT_MM,
    THIEF_TERMINAL_WAIT_MS,THIEF_TERMINAL_EXIT_MS,THIEF_TERMINAL_ENDPOINTS,THIEF_TERMINAL_PATH_EDGES};
static Arrival terminal_arrival; /* Sequential stopped planning, never ISR/reentrant. */

static Terminal CheckTerminal(uint16_t index,uint8_t body,int16_t px,int16_t py) {
  uint8_t model;
  const RolloutRoute_t *r=&rollout_routes[index];
  memset(&terminal_result,0,sizeof(terminal_result));terminal_result.reserve=0x7FFFFFFFL;terminal_result.wait=65535U;terminal_result.checked=1U;
  for(model=0U;model<THIEF_ROLLOUT_MODELS;model++) {
    uint8_t branches=(uint8_t)(THIEF_ROLLOUT_MODELS/2U),scenario=(uint8_t)(THIEF_ROLLOUT_MODELS-1U-model),e;
    uint32_t speed=scenario<branches?THIEF_FORWARD_MM_S:THIEF_POLICE_MAX_MM_S;
    uint8_t bias=(uint8_t)(scenario%branches),known=0U,clear=0U;
    int32_t best=-0x7FFFFFFFL;
    terminal_transfer=Run(r,body,px,py,speed,bias,&terminal_arrival);
    if(!terminal_transfer.clear || !terminal_arrival.valid) continue;
    for(e=0U;e<2U;e++) {
      uint8_t out=(uint8_t)((r->body+2U*e)%4U);
      terminal_score=ThiefTerminalForecast_Run(&terminal_config,&terminal_ops,
          &terminal_arrival.own,&terminal_arrival.police,terminal_arrival.visits,speed,bias,out);
      terminal_result.steps+=terminal_score.steps;model_steps+=terminal_score.steps;
      if(terminal_score.sensed && terminal_score.wait_ms<terminal_result.wait) terminal_result.wait=terminal_score.wait_ms;
      if(terminal_score.known) {
        known++;
        if(terminal_score.clear && terminal_score.reserve_ms>(int32_t)THIEF_TIMING_GUARD_MS) {
          clear=1U;if(terminal_score.reserve_ms>best) best=terminal_score.reserve_ms;
        }
      }
    }
    /* One demonstrated exit suffices for this finite hypothesis. If no exit
     * passes, both must be known before labeling the hypothesis unsafe. */
    if(clear || known==2U) terminal_result.known++;
    if(clear) {terminal_result.clear++;if(best<terminal_result.reserve) terminal_result.reserve=best;}
  }
  if(terminal_result.wait==65535U) terminal_result.wait=0U;
  terminal_result.qualified=(uint8_t)(terminal_result.clear==THIEF_ROLLOUT_MODELS);
  if(!terminal_result.qualified) terminal_result.reserve=0L;
  return terminal_result;
}

static uint8_t BetterClear(const Candidate *a,const Candidate *b) {
  int8_t order;
  if(b->index==65535U) return 1U;
  order=ThiefRefugePolicy_CompareSlack(a->escape,a->refuge,b->escape,b->refuge);
  return (uint8_t)(order>0 || (order==0 && (a->cool<b->cool || (a->cool==b->cool &&
      (a->gap>b->gap || (a->gap==b->gap && a->time<b->time))))));
}
void ThiefRollout_ResetEvidence(void) {memset(entry,0,sizeof(entry));entry_heading=4U;entry_age=0U;}
void ThiefRollout_Init(void) {memset(exposed,0,sizeof(exposed));model_steps=0UL;model_count=0U;ThiefRollout_ResetEvidence();}
void ThiefRollout_Evidence(uint16_t own,uint8_t h,uint16_t distance,uint8_t confidence,uint32_t tick) {
  EntryEvidence *e;uint16_t n,mm,total=0U;uint8_t steps=0U;
  if(!own || own>88U || h>3U) return;
  e=&entry[h];
  if(e->own!=own) {memset(e,0,sizeof(*e));e->own=own;}
  if(confidence<20U || !distance) {e->clear_count=0U;e->valid=0U;return;}
  if(distance>THIEF_LASER_MAX_MM) {
    if(tick-e->last_clear>80UL) e->clear_count=0U;
    if(e->clear_count<255U) e->clear_count++;e->last_clear=tick;e->valid=0U;return;
  }
  if(e->valid) return;
  if(e->clear_count<4U || tick-e->last_clear>80UL) {e->clear_count=0U;return;}
  e->clear_count=0U;n=own;
  while(steps++<88U && total<=THIEF_LASER_MAX_MM) {
    uint16_t next=rollout_next[n-1U][h];
    if(!next) break;mm=rollout_length[n-1U][h];total=(uint16_t)(total+mm);n=next;
    /* The ray was clear, then a car appeared at its ending junction. There
     * is no road beyond it, so under the reference motion model the car
     * arrived through a side arm and has to change travel direction. */
    if(Abs((int32_t)distance-total)<=2L && !rollout_next[n-1U][h] &&
        map_route_nodes[n-1U].degree>=2U) {
      e->node=n;e->first_hit=tick;e->valid=1U;break;
    }
  }
}
void ThiefRollout_MarkExposed(uint16_t n,uint32_t now) {
  if(!n || n>88U) return;exposed[n-1U]=1U;exposed_at[n-1U]=now;
}
static uint8_t Enumerate(uint16_t node,uint8_t body,uint8_t mask,uint16_t previous,uint32_t now) {
  memset(&selector,0,sizeof(selector));
  selector.best=0xFFFFU;selector.best_cool=1U;selector.best_time=65535U;
  selector.best_escape=-0x7FFFFFFFL;selector.best_gap=-1L;
  memset(best_rank,0,sizeof(best_rank));
  memset(&normal,0,sizeof(normal));memset(&conditional,0,sizeof(conditional));
  normal.index=conditional.index=65535U;
  memset(&selected_terminal,0,sizeof(selected_terminal));
  if(!node || node>88U || body>3U) return 0U;
  if(!ThiefThreatModel_Snapshot(now,&selector.px,&selector.py)) return 0U;
  entry_heading=4U;entry_age=0U;
  {
    uint8_t h;
    for(h=0U;h<4U;h++) if(THIEF_ENTRY_EVIDENCE && entry[h].valid && entry[h].own==node && now-entry[h].first_hit<=250UL &&
        selector.px==map_route_nodes[entry[h].node-1U].x_half*150 && selector.py==map_route_nodes[entry[h].node-1U].y_half*150) {
      entry_heading=h;entry_age=(uint16_t)(now-entry[h].first_hit);break;
    }
  }
  selector.a=rollout_start[(node-1U)*4U+body];selector.z=rollout_start[(node-1U)*4U+body+1U];
  for(selector.i=1U;selector.i<=88U;selector.i++) {
    uint16_t delay=ThiefRefugePolicy_PatrolDetectionMs(selector.i);
    if(ThiefRefugePolicy_FastExits(selector.i)>=4U && delay>selector.max_shelter_delay) selector.max_shelter_delay=delay;
  }
  model_steps=0UL;model_count=0U;
  for(selector.i=selector.a;selector.i<selector.z;selector.i++) {
    const RolloutRoute_t *r=&rollout_routes[selector.i];uint16_t contact=65535U;uint8_t clear=1U,model,cool,pruned=0U;
    uint16_t goal=rollout_nodes[r->offset+r->count-1U];
    uint8_t refuge=ThiefRefugePolicy_Class(goal);
    uint8_t role=ThiefRefugePolicy_Role(goal);
    int32_t escape=0x7FFFFFFFL,gap=0x7FFFFFFFL;
    int8_t rank_order=0,slack_order;uint8_t coverage=0U;
    uint16_t runway=thief_straight_runway_mm[goal-1U][r->body];
    uint16_t back=thief_straight_runway_mm[goal-1U][(r->body+2U)%4U];
    if(!THIEF_REFUGE_RETURN_CYCLES && rollout_nodes[r->offset]==goal) continue;
    if(role==2U && !THIEF_TERMINAL_ESCAPE_ENABLED) continue;
    if(rollout_nodes[r->offset]==goal &&
        (ThiefRefugePolicy_FastExits(goal)<4U ||
         ThiefRefugePolicy_PatrolDetectionMs(goal)<selector.max_shelter_delay)) continue;
    if(back<runway) runway=back;
    if(mask&(1U<<r->first)) continue;
    cool=(uint8_t)(goal==previous || (exposed[goal-1U] && now-exposed_at[goal-1U]<THIEF_HIDE_COOLDOWN_MS));
    for(model=0U;model<THIEF_ROLLOUT_MODELS;model++) {
      uint8_t branches=(uint8_t)(THIEF_ROLLOUT_MODELS/2U);
      /* Fastest interception first. Unknown futures optimistically remain
       * clear; if even that vector loses, completing this candidate cannot
       * change the maximin/leximin result. This is an exact pruning rule. */
      uint8_t scenario=(uint8_t)(THIEF_ROLLOUT_MODELS-1U-model),k;
      selector.score=Run(r,body,selector.px,selector.py,scenario<branches?THIEF_FORWARD_MM_S:THIEF_POLICE_MAX_MM_S,(uint8_t)(scenario%branches),NULL);
      model_count++;if(!selector.score.clear) clear=0U;
      selector.rank[model]=selector.score.clear?20001U:THIEF_ROLLOUT_RISK_TIERS?
          (uint16_t)(selector.score.contact/THIEF_ROLLOUT_TIME_BUCKET_MS*THIEF_ROLLOUT_TIME_BUCKET_MS):selector.score.contact;
      if(selector.score.clear) coverage++;
      if(selector.score.contact<contact) contact=selector.score.contact;
      if(selector.score.escape<escape) escape=selector.score.escape;if(selector.score.gap<gap) gap=selector.score.gap;
      if(THIEF_ROLLOUT_PRUNE && selector.best!=0xFFFFU) {
        for(k=0U;k<THIEF_ROLLOUT_MODELS;k++) selector.bound[k]=k<=model?selector.rank[k]:20001U;
        for(k=1U;k<THIEF_ROLLOUT_MODELS;k++) {
          uint16_t value=selector.bound[k];uint8_t j=k;
          while(j && selector.bound[j-1U]>value) {selector.bound[j]=selector.bound[j-1U];j--;}
          selector.bound[j]=value;
        }
        if((selector.best_clear && !clear) || (!selector.best_clear && RiskOrder(selector.bound,best_rank,
            (uint8_t)(coverage+THIEF_ROLLOUT_MODELS-1U-model),selector.best_coverage,
            runway,selector.best_runway)<0)) {pruned=1U;break;}
      }
    }
    selector.evaluated++;
    if(pruned) continue;
    /* Leximin retains the second/third worst futures when the first risky
     * corner is common to many routes. Goal value after selector.a modeled collision
     * must not be used as if the car had reached that goal. */
    for(model=1U;model<THIEF_ROLLOUT_MODELS;model++) {
      uint16_t value=selector.rank[model];uint8_t j=model;
      while(j && selector.rank[j-1U]>value) {selector.rank[j]=selector.rank[j-1U];j--;}
      selector.rank[j]=value;
    }
    rank_order=RiskOrder(selector.rank,best_rank,coverage,selector.best_coverage,runway,selector.best_runway);
    if(!clear) escape=0L;
    if(rollout_nodes[r->offset]==goal &&
        (!clear || escape<=(int32_t)(THIEF_HIDE_REACTION_MS+THIEF_TIMING_GUARD_MS))) continue;
    slack_order=ThiefRefugePolicy_CompareSlack(escape,refuge,selector.best_escape,selector.best_refuge);
    if(THIEF_TERMINAL_ESCAPE_ENABLED && clear && escape>0L && role) {
      Candidate *target=role==2U?&conditional:&normal;
      current.index=selector.i;current.time=r->time_ms;current.contact=contact;current.runway=runway;
      current.escape=escape;current.gap=gap;current.clear=clear;current.cool=cool;
      current.coverage=coverage;current.refuge=refuge;memcpy(current.rank,selector.rank,sizeof(selector.rank));
      if(BetterClear(&current,target)) *target=current;
    }
    /* Conditional N026 is never selector.a legacy/fallback parking result. It must
     * demonstrate the wait/detect/first-corner exit below before admission. */
    if(role==2U) continue;
    /* Clear transfers preserve larger dynamic terminal-slack bands; refuge
     * class only breaks positive same-band ties.
     * Only degraded decisions use the bounded uncertainty/coverage hierarchy.
     * Long straight distance alone must never displace selector.a clear transfer. */
    if(selector.best==0xFFFFU || clear>selector.best_clear || (clear==selector.best_clear &&
       ((!clear && rank_order>0) || ((clear || rank_order==0) &&
       (slack_order>0 || (slack_order==0 &&
       (cool<selector.best_cool || (cool==selector.best_cool &&
       (gap>selector.best_gap || (gap==selector.best_gap && r->time_ms<selector.best_time)))))))))) {
      selector.best=selector.i;selector.best_coverage=coverage;selector.best_runway=runway;selector.best_clear=clear;selector.best_contact=contact;selector.best_escape=escape;selector.best_cool=cool;selector.best_gap=gap;selector.best_time=r->time_ms;
      selector.best_refuge=refuge;
      memcpy(best_rank,selector.rank,sizeof(best_rank));
    }
  }
  return (uint8_t)(selector.best!=0xFFFFU);
}
/* Enumerate has returned before any callback-heavy terminal forecast starts.
 * All finalist records survive in guarded RAM; the enumeration stack does not. */
static uint8_t Finalize(uint8_t body,uint16_t *path,uint16_t *count,uint8_t *mode,ThiefDecision_t *d) {
  if(THIEF_TERMINAL_ESCAPE_ENABLED) {
    Candidate *pick=NULL;
    memset(&normal_terminal,0,sizeof(normal_terminal));
    memset(&conditional_terminal,0,sizeof(conditional_terminal));
    if(normal.index!=65535U) {
      normal_terminal=CheckTerminal(normal.index,body,selector.px,selector.py);selector.terminal_candidates++;
      selector.terminal_steps+=normal_terminal.steps;
      if(normal_terminal.qualified) {pick=&normal;selected_terminal=normal_terminal;}
    }
    if(conditional.index!=65535U) {
      conditional_terminal=CheckTerminal(conditional.index,body,selector.px,selector.py);selector.terminal_candidates++;
      selector.terminal_steps+=conditional_terminal.steps;
      if(conditional_terminal.qualified && (pick==NULL ||
          ThiefRefugePolicy_CompareSlack(conditional_terminal.reserve,conditional.refuge,
              selected_terminal.reserve,pick->refuge)>0)) {
        pick=&conditional;selected_terminal=conditional_terminal;
      }
    }
    if(pick) {
      selector.best=pick->index;selector.best_time=pick->time;selector.best_contact=pick->contact;selector.best_runway=pick->runway;
      selector.best_escape=pick->escape;selector.best_gap=pick->gap;selector.best_clear=pick->clear;selector.best_cool=pick->cool;
      selector.best_coverage=pick->coverage;selector.best_refuge=pick->refuge;memcpy(best_rank,pick->rank,sizeof(best_rank));
    } else {
      /* Unknown or failed alternatives do not invent safe parking. Retain
       * the original transfer winner; role0 is an explicit emergency stop. */
      if(selector.best==normal.index) selected_terminal=normal_terminal;
    }
  }
  {
    const RolloutRoute_t *r=&rollout_routes[selector.best];uint16_t goal=rollout_nodes[r->offset+r->count-1U];
    for(selector.i=0U;selector.i<r->count;selector.i++) path[selector.i]=rollout_nodes[r->offset+selector.i];
    *count=r->count;*mode=r->mode;memset(d,0,sizeof(*d));
    d->goal_node=goal;d->travel_ms=r->time_ms;d->states_expanded=selector.evaluated;d->threat_count=1U;
    d->degraded=(uint8_t)(!selector.best_clear || selector.best_cool || selector.best_escape<=0L);d->reused_hide=selector.best_cool;
    d->emergency_hide=(uint8_t)(ThiefRefugePolicy_Role(goal)==0U);
    d->terminal_checked=selected_terminal.checked;
    d->terminal_known_models=selected_terminal.known;d->terminal_clear_models=selected_terminal.clear;
    d->terminal_wait_ms=selected_terminal.wait;d->terminal_reserve_ms=selected_terminal.reserve;
    d->terminal_candidates=selector.terminal_candidates;d->terminal_steps=selector.terminal_steps;
    if(d->emergency_hide || (d->terminal_checked && !selected_terminal.qualified)) d->degraded=1U;
    d->escape_exits=thief_exit_count[goal-1U];d->parking_heading=r->body;
    d->exit_branch_ms=thief_exit_branch_ms[goal-1U][r->body];
    d->worst_margin_ms=selector.best_clear?selector.best_escape:(int32_t)selector.best_contact-(int32_t)r->time_ms;
    d->first_pause_risk_ms=selector.best_clear?20001U:selector.best_contact;
    d->candidates_evaluated=selector.evaluated;d->model_rollouts=model_count;d->forecast_contact_ms=selector.best_contact;
    d->terminal_escape_ms=selector.best_escape;d->planner_kind=1U;
    d->model_steps=model_steps;d->forecast_clear=selector.best_clear;
    d->entry_inferred=(uint8_t)(entry_heading<4U);
    d->surviving_models=selector.best_coverage;d->risk_band_ms=best_rank[0];
    d->recovery_runway_ms=(uint16_t)((uint32_t)selector.best_runway*1000UL/
        (THIEF_FORWARD_MM_S>THIEF_REVERSE_MM_S?THIEF_FORWARD_MM_S:THIEF_REVERSE_MM_S));
    d->refuge_class=ThiefRefugePolicy_Class(goal);
    d->fast_escape_exits=ThiefRefugePolicy_FastExits(goal);
    d->patrol_detection_ms=ThiefRefugePolicy_PatrolDetectionMs(goal);
  }
  return 1U;
}
uint8_t ThiefRollout_Select(uint16_t node,uint8_t body,uint8_t mask,uint16_t previous,uint32_t now,
    uint16_t *path,uint16_t *count,uint8_t *mode,ThiefDecision_t *d) {
  uint8_t ok;
  if(select_active || !path || !count || !mode || !d) return 0U;
  select_active=1U;
  ok=Enumerate(node,body,mask,previous,now);
  if(ok) ok=Finalize(body,path,count,mode,d);
  select_active=0U;
  return ok;
}
uint32_t ThiefRollout_WorkBytes(void) {return (uint32_t)(sizeof(exposed_at)+sizeof(exposed)+sizeof(model_steps)+sizeof(model_count)+sizeof(entry)+sizeof(entry_age)+sizeof(entry_heading)+sizeof(terminal_arrival)+sizeof(select_active)+sizeof(run_thief)+sizeof(run_police)+sizeof(run_b0)+sizeof(run_p0)+sizeof(run_b1)+sizeof(run_p1)+sizeof(run_seen_count)+sizeof(normal)+sizeof(conditional)+sizeof(current)+sizeof(normal_terminal)+sizeof(conditional_terminal)+sizeof(selected_terminal)+sizeof(best_rank)+sizeof(selector)+sizeof(terminal_result)+sizeof(terminal_transfer)+sizeof(terminal_score))+ThiefTerminalForecast_WorkBytes();}
#ifdef THIEF_ROLLOUT_TEST
int8_t ThiefRollout_TestOrder(const uint16_t *a,const uint16_t *b,uint8_t ac,uint8_t bc,uint16_t ar,uint16_t br) {
  return RiskOrder(a,b,ac,bc,ar,br);
}
uint8_t ThiefRollout_TestView(int32_t x0,int32_t y0,int32_t x1,int32_t y1) {
  Box a={0,0,95,99},b0={x0,y0,95,99},b1={x1,y1,95,99};
  return VisibleDuring(&a,&a,&b0,&b1);
}
int32_t ThiefRollout_TestSweep(int32_t x0,int32_t y0,int32_t x1,int32_t y1,uint8_t rotating) {
  Box a0={x0,y0,95,99},a1={x1,y1,95,99},b={0,0,95,99};
  if(rotating) b.hx=b.hy=138;
  return ContactFraction(&a0,&a1,&b,&b);
}
#endif

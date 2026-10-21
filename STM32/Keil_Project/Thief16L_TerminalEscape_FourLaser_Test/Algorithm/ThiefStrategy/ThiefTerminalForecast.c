#include "ThiefTerminalForecast.h"
#include <stddef.h>
#include <string.h>

typedef ThiefTerminalActor_t Actor;
typedef ThiefTerminalBox_t Box;
typedef ThiefTerminalOps_t Ops;
typedef ThiefTerminalConfig_t Config;
typedef ThiefTerminalScore_t Score;
static struct {
  Actor thief,pursuer,projection,path_actor,guide;
  Box own,b0,p0,b1,p1,view_start,threat,destination;
  Score result;
  uint8_t count,index,body;
  uint16_t nominal;
  uint32_t now,started;
  struct {
    uint8_t k,chosen,h;
    uint16_t goal,next,branch;
    int32_t best,best_gap,own_cost,police_cost,reserve,gap;
  } turn;
  uint8_t seen[88],path[13];
} workspace;
static uint8_t active;

static uint8_t Node(uint16_t n) {return (uint8_t)(n && n<=THIEF_TERMINAL_NODE_COUNT);}
static uint8_t ActorValid(const Actor *a)
{
  if(a==NULL || !Node(a->node) || a->h>3U || a->mode>1U || a->phase>3U) return 0U;
  if(a->phase==1U || a->phase==2U)
    return (uint8_t)(Node(a->to) && a->length && a->total && a->elapsed<a->total);
  return 1U;
}
static uint8_t Inputs(const Config *c,const Ops *o)
{
  return (uint8_t)(c!=NULL && o!=NULL && c->forward_speed_mm_s && c->turn_ms &&
    c->reaction_ms && c->wait_limit_ms && c->wait_limit_ms<=THIEF_TERMINAL_WAIT_MAX_MS &&
    c->exit_limit_ms && c->exit_limit_ms<=THIEF_TERMINAL_EXIT_MAX_MS &&
    c->reaction_ms<=c->exit_limit_ms && c->max_edges && c->max_edges<=THIEF_TERMINAL_PATH_MAX_EDGES &&
    c->endpoint_budget && c->endpoint_budget<=THIEF_TERMINAL_MAX_INTERVALS &&
    o->pose && o->visible && o->visible_during && o->contact_fraction &&
    o->command && o->advance && o->best_police && o->next && o->length &&
    o->branch && o->reach && o->node_kind);
}
static uint8_t Step(const Config *c,Score *s)
{
  if(s->steps>=c->endpoint_budget) return 0U;
  s->steps++;return 1U;
}
static Box PoseAt(const Ops *o,const Actor *a,uint16_t dt)
{
  workspace.projection=*a;
  if(workspace.projection.phase==1U || workspace.projection.phase==2U)
    workspace.projection.elapsed=(uint16_t)(workspace.projection.elapsed+dt);
  return o->pose(o->context,&workspace.projection);
}
static uint16_t Remaining(const Actor *a,uint16_t cap)
{
  if(a->phase==1U || a->phase==2U) {
    uint16_t left=(uint16_t)(a->total-a->elapsed);
    if(left<cap) cap=left;
  }
  return cap;
}
static uint8_t Command(const Config *c,const Ops *o,Actor *a,uint8_t out,
                       uint32_t speed,uint32_t turn)
{
  uint16_t node=a->node,next;
  (void)c;
  if(out>3U || !Node(node)) return 0U;
  next=o->next(o->context,node,out);
  if(!Node(next) || !o->length(o->context,node,out)) return 0U;
  o->command(o->context,a,out,speed,turn);
  return (uint8_t)(a->node==node && a->to==next && a->h==out &&
    (a->phase==1U || a->phase==2U) && a->elapsed==0U && ActorValid(a));
}
static uint8_t Advance(const Ops *o,Actor *a,uint16_t dt,uint32_t speed)
{
  if(a->phase==3U) return 1U; /* Never wake a stationary pursuer hypothesis. */
  o->advance(o->context,a,dt,speed);
  return ActorValid(a);
}
static void ObservedGoal(Actor *p,const Actor *b,uint8_t bias)
{
  p->goal=b->phase==2U?b->to:b->node;
  if(b->phase==2U) p->last_seen_h=b->h;
  (void)bias;
}
static uint8_t PoliceReady(const Config *c,const Ops *o,Actor *p,const Actor *b,
                          uint32_t speed,uint8_t bias,uint8_t seen[88])
{
  uint8_t out;
  workspace.b0=o->pose(o->context,p);workspace.p0=o->pose(o->context,b);
  if(o->visible(o->context,&workspace.b0,&workspace.p0)) {
    ObservedGoal(p,b,bias);
    if(bias>=2U && p->last_seen_h<4U) {
      uint16_t at=b->phase==2U?b->to:b->node;
      uint16_t ahead=o->next(o->context,at,p->last_seen_h);
      p->goal=Node(ahead)?ahead:at;
    }
  }
  if(p->phase!=0U) return 1U;
  if(!Node(p->goal)) p->goal=b->node;
  if(p->node==p->goal) {
    uint8_t visits=255U,k;
    out=4U;
    for(k=0U;k<4U;k++) {
      uint8_t h=(uint8_t)(((p->last_seen_h<4U?p->last_seen_h:p->h)+(bias?4U-k:k))%4U);
      uint16_t next=o->next(o->context,p->node,h);
      if(!Node(next) || next==p->last || (o->node_kind(o->context,p->node)==0U && h!=p->h)) continue;
      if(seen[next-1U]<visits) {visits=seen[next-1U];out=h;}
    }
    if(out==4U) out=(uint8_t)((p->h+2U)%4U);
  } else out=o->best_police(o->context,p->node,p->h,p->mode,p->goal,speed);
  if(seen[p->node-1U]<254U) seen[p->node-1U]++;
  return Command(c,o,p,out,speed,c->turn_ms);
}
static uint16_t FirstView(const Ops *o,const Box *own,const Actor *p,uint16_t dt)
{
  uint16_t lo=0U,hi=dt;
  workspace.view_start=o->pose(o->context,p);
  if(o->visible(o->context,own,&workspace.view_start)) return 0U;
  workspace.p1=PoseAt(o,p,dt);
  if(!o->visible_during(o->context,own,own,&workspace.view_start,&workspace.p1)) return 65535U;
  /* Existence of any view in the prefix is monotone, even for a brief pass. */
  while((uint16_t)(hi-lo)>1U) {
    uint16_t mid=(uint16_t)(lo+(hi-lo)/2U);
    workspace.p1=PoseAt(o,p,mid);
    if(o->visible_during(o->context,own,own,&workspace.view_start,&workspace.p1)) hi=mid;
    else lo=mid;
  }
  return hi;
}
static uint8_t InPath(const uint8_t *path,uint8_t count,uint16_t node)
{
  uint8_t k;
  for(k=0U;k<count;k++) if(path[k]==node) return 1U;
  return 0U;
}
static int32_t Abs(int32_t x) {return x<0L?-x:x;}
static uint8_t TurnArm(const Config *c,const Ops *o,const Actor *a,const Actor *observed,
    uint32_t police_speed,const uint8_t *path,uint8_t count,uint8_t *heading,uint16_t *target)
{
  memset(&workspace.turn,0,sizeof(workspace.turn));
  workspace.turn.chosen=4U;workspace.turn.best=-0x7FFFFFFFL;workspace.turn.best_gap=-1L;
  workspace.threat=o->pose(o->context,observed);
  if(o->node_kind(o->context,a->node)==0U) return 0U;
  for(workspace.turn.k=0U;workspace.turn.k<2U;workspace.turn.k++) {
    workspace.turn.h=(uint8_t)((a->h+(workspace.turn.k?3U:1U))%4U);
    workspace.turn.next=o->next(o->context,a->node,workspace.turn.h);workspace.turn.branch=o->branch(o->context,a->node,workspace.turn.h);
    workspace.guide=*a;
    if(!Node(workspace.turn.next) || !Node(workspace.turn.branch) || workspace.turn.branch==a->node || InPath(path,count,workspace.turn.next) || InPath(path,count,workspace.turn.branch)) continue;
    workspace.guide.h=workspace.turn.h;workspace.guide.phase=0U;workspace.guide.to=0U;workspace.guide.elapsed=0U;
    workspace.turn.own_cost=o->reach(o->context,&workspace.guide,workspace.turn.branch,c->forward_speed_mm_s);
    workspace.turn.police_cost=o->reach(o->context,observed,workspace.turn.branch,police_speed);
    if(workspace.turn.own_cost<0L || workspace.turn.police_cost<0L) continue;
    workspace.turn.reserve=workspace.turn.police_cost-workspace.turn.own_cost;
    workspace.guide.node=workspace.turn.branch;workspace.guide.phase=3U;workspace.destination=o->pose(o->context,&workspace.guide);
    workspace.turn.gap=Abs(workspace.destination.x-workspace.threat.x)+Abs(workspace.destination.y-workspace.threat.y);
    /* The arm is chosen from a first observed hypothesis snapshot. A cheap
     * turn toward the detected car must not automatically exclude the safe
     * opposite continuation. This ranking is followed by actual contact tests. */
    if(workspace.turn.reserve>workspace.turn.best || (workspace.turn.reserve==workspace.turn.best && workspace.turn.gap>workspace.turn.best_gap)) {
      workspace.turn.best=workspace.turn.reserve;workspace.turn.best_gap=workspace.turn.gap;workspace.turn.chosen=workspace.turn.h;workspace.turn.goal=workspace.turn.branch;
    }
  }
  if(workspace.turn.chosen>3U) return 0U;
  *heading=workspace.turn.chosen;*target=workspace.turn.goal;return 1U;
}
static uint8_t ExitPath(const Config *c,const Ops *o,uint16_t goal,uint8_t body,
    uint8_t out,const Actor *observed,uint32_t police_speed,
    uint8_t path[13],uint8_t *count,uint16_t *duration,Score *s)
{
  Actor *b=&workspace.path_actor;
  uint8_t n=1U,turned=0U;
  uint16_t target=0U;
  uint32_t total=0UL;
  memset(b,0,sizeof(*b));b->node=goal;b->h=body;b->first=1U;path[0]=(uint8_t)goal;
  while((uint8_t)(n-1U)<c->max_edges) {
    uint16_t expected;
    if(!Step(c,s) || !Command(c,o,b,out,c->forward_speed_mm_s,c->turn_ms)) return 0U;
    expected=b->to;
    if(InPath(path,n,expected)) return 0U;
    if(n==1U) {b->phase=1U;b->total=c->reaction_ms;b->elapsed=0U;b->turning=0U;}
    if(b->phase==1U) {
      uint16_t wait=b->total;
      total+=wait;
      if(!Advance(o,b,wait,c->forward_speed_mm_s) || b->phase!=2U || b->elapsed!=0U) return 0U;
    }
    total+=b->total;
    if(total>c->exit_limit_ms || !Advance(o,b,b->total,c->forward_speed_mm_s) ||
        b->phase!=0U || b->node!=expected) return 0U;
    path[n++]=(uint8_t)b->node;
    if(turned && b->node==target) {*count=n;*duration=(uint16_t)total;return 1U;}
    if(!turned) {
      uint16_t forward=o->next(o->context,b->node,b->h);
      /* Do not follow a straight arm into a dead/pocket endpoint just to find
       * a physical corner. Turn at its last viable junction instead. */
      if(Node(forward) && Node(o->branch(o->context,b->node,b->h))) out=b->h;
      else {
        if(!TurnArm(c,o,b,observed,police_speed,path,n,&out,&target)) return 0U;
        turned=1U;
      }
    } else out=o->best_police(o->context,b->node,b->h,b->mode,target,c->forward_speed_mm_s);
  }
  return 0U; /* No route THROUGH a first corner within the fixed edge budget. */
}

static Score Run(const Config *c,const Ops *o,
    const Actor *stopped,const Actor *pursuer,const uint8_t visits[88],
    uint32_t speed,uint8_t bias,uint8_t out)
{
  Actor *b=&workspace.thief,*p=&workspace.pursuer;
  uint8_t *seen=workspace.seen,*path=workspace.path;
  workspace.now=0UL;
  memset(&workspace.result,0,sizeof(workspace.result));workspace.result.exit_heading=out;
  if(!Inputs(c,o) || !ActorValid(stopped) || !ActorValid(pursuer) || !visits ||
      !speed || (stopped->phase!=0U && stopped->phase!=3U)) return workspace.result;
  *b=*stopped;*p=*pursuer;workspace.body=(uint8_t)((b->h+2U*b->mode)%4U);
  if(out>3U || out%2U!=workspace.body%2U) return workspace.result;
  b->phase=3U;b->to=0U;b->elapsed=0U;b->turning=0U;
  {
    workspace.own=o->pose(o->context,b);workspace.p0=o->pose(o->context,p);
    if(o->contact_fraction(o->context,&workspace.own,&workspace.own,&workspace.p0,&workspace.p0)>=0L) {
      workspace.result.steps=1U;workspace.result.known=1U;workspace.result.reserve_ms=-(int32_t)c->reaction_ms;return workspace.result;
    }
  }
  memcpy(seen,visits,88U);
  while(workspace.now<c->wait_limit_ms) {
    uint16_t dt,view;
    int32_t fraction;
    if(!Step(c,&workspace.result) || !PoliceReady(c,o,p,b,speed,bias,seen)) return workspace.result;
    dt=Remaining(p,(uint16_t)(c->wait_limit_ms-workspace.now));
    if(!dt) return workspace.result;
    workspace.own=o->pose(o->context,b);view=FirstView(o,&workspace.own,p,dt);
    if(view!=65535U && view<dt) dt=view;
    workspace.p0=o->pose(o->context,p);workspace.p1=PoseAt(o,p,dt);
    fraction=o->contact_fraction(o->context,&workspace.own,&workspace.own,&workspace.p0,&workspace.p1);
    if(fraction>=0L) {
      if(fraction>1024L) return workspace.result;
      workspace.result.wait_ms=(uint16_t)(workspace.now+(uint32_t)dt*(uint32_t)fraction/1024UL);
      workspace.result.known=1U;workspace.result.reserve_ms=-(int32_t)c->reaction_ms;return workspace.result;
    }
    if(dt && !Advance(o,p,dt,speed)) return workspace.result;
    workspace.now+=dt;
    if(view!=65535U) {workspace.result.sensed=1U;break;}
  }
  workspace.result.wait_ms=(uint16_t)workspace.now;
  if(!workspace.result.sensed) return workspace.result;
  /* Choose the future turn only after the first virtual sensor view. The
   * immutable observed actor is a finite hypothesis, not an omniscient pose. */
  if(!ExitPath(c,o,b->node,workspace.body,out,p,speed,path,&workspace.count,&workspace.nominal,&workspace.result)) return workspace.result;
  workspace.started=workspace.now;memset(b,0,sizeof(*b));b->node=path[0];b->h=workspace.body;b->first=1U;workspace.index=1U;
  while(workspace.now-workspace.started<=c->exit_limit_ms) {
    uint16_t dt;
    int32_t fraction;
    if(b->phase==0U && workspace.index>=workspace.count) {
      int32_t reserve=o->reach(o->context,p,b->node,speed)-
        (int32_t)((uint32_t)c->contact_mm*1000UL/speed)-(int32_t)c->timing_guard_ms;
      workspace.result.known=1U;workspace.result.clear=1U;workspace.result.escape_ms=(uint16_t)(workspace.now-workspace.started);
      workspace.result.reserve_ms=reserve>0L?reserve:0L;return workspace.result;
    }
    if(!Step(c,&workspace.result)) return workspace.result;
    if(b->phase==0U) {
      uint8_t h;
      for(h=0U;h<4U;h++) if(o->next(o->context,b->node,h)==path[workspace.index]) break;
      workspace.index++;
      if(!Command(c,o,b,h,c->forward_speed_mm_s,c->turn_ms)) return workspace.result;
      if(workspace.index==2U) {b->phase=1U;b->total=c->reaction_ms;b->elapsed=0U;b->turning=0U;}
    }
    if(!PoliceReady(c,o,p,b,speed,bias,seen)) return workspace.result;
    dt=Remaining(b,(uint16_t)(c->exit_limit_ms-(workspace.now-workspace.started)));
    dt=Remaining(p,dt);
    if(!dt) return workspace.result;
    workspace.b0=o->pose(o->context,b);workspace.p0=o->pose(o->context,p);
    workspace.b1=PoseAt(o,b,dt);workspace.p1=PoseAt(o,p,dt);
    if(o->visible_during(o->context,&workspace.p0,&workspace.p1,&workspace.b0,&workspace.b1)) {
      ObservedGoal(p,b,bias);
      if(bias>=2U && p->last_seen_h<4U) {
        uint16_t at=b->phase==2U?b->to:b->node,ahead=o->next(o->context,at,p->last_seen_h);
        p->goal=Node(ahead)?ahead:at;
      }
    }
    fraction=o->contact_fraction(o->context,&workspace.b0,&workspace.b1,&workspace.p0,&workspace.p1);
    if(fraction>=0L) {
      uint32_t hit;
      if(fraction>1024L) return workspace.result;
      hit=workspace.now-workspace.started+(uint32_t)dt*(uint32_t)fraction/1024UL;
      workspace.result.known=1U;workspace.result.escape_ms=(uint16_t)hit;
      workspace.result.reserve_ms=(int32_t)hit-(int32_t)workspace.nominal;return workspace.result;
    }
    if(!Advance(o,b,dt,c->forward_speed_mm_s) || !Advance(o,p,dt,speed)) return workspace.result;
    workspace.now+=dt;
  }
  return workspace.result;
}
ThiefTerminalScore_t ThiefTerminalForecast_Run(const Config *c,const Ops *o,
    const Actor *stopped,const Actor *pursuer,const uint8_t visits[88],
    uint32_t speed,uint8_t bias,uint8_t out)
{
  Score s;
  memset(&s,0,sizeof(s));s.exit_heading=out;
  if(active) return s;
  active=1U;memset(&workspace,0,sizeof(workspace));
  s=Run(c,o,stopped,pursuer,visits,speed,bias,out);
  active=0U;return s;
}
uint32_t ThiefTerminalForecast_WorkBytes(void)
{
  return (uint32_t)(sizeof(workspace)+sizeof(active));
}

/* Pure fixed-graph algorithm fixtures. No HAL, simulated peripherals or motors. */
#include "ThiefTerminalForecast.h"
#include <string.h>

volatile unsigned test_checks,test_failures,test_done,test_first_line;
volatile unsigned test_known_cases,test_unknown_cases;
volatile unsigned test_view_clear,test_view_escape_ms,test_work_bytes;
typedef ThiefTerminalActor_t Actor;
typedef ThiefTerminalBox_t Box;
typedef ThiefTerminalScore_t Score;
typedef struct {
  uint16_t next[12][4],length[12][4],branch[12][4];
  int16_t x[12],y[12];
  unsigned command_calls,held_command_calls,quarter_commands;
  unsigned first_mode[32],first_body[32],first_count;
  unsigned arrivals[12];
  unsigned guarded_calls;
  uint16_t reverse_speed;
  uint8_t invalid_command,invalid_advance,try_reentrant;
} Graph;
static Graph graph;
static ThiefTerminalConfig_t cfg;
static ThiefTerminalOps_t ops;
static uint8_t visits[88];

static void checked(unsigned ok,unsigned line)
{
  test_checks++;
  if(!ok) {if(!test_failures)test_first_line=line;test_failures++;}
}
#define check(ok) checked((ok),__LINE__)
static int32_t abs32(int32_t x) {return x<0L?-x:x;}
static uint16_t next(void *p,uint16_t n,uint8_t h)
{Graph *g=p;return n<12U && h<4U?g->next[n][h]:0U;}
static uint16_t length(void *p,uint16_t n,uint8_t h)
{Graph *g=p;return n<12U && h<4U?g->length[n][h]:0U;}
static uint16_t branch(void *p,uint16_t n,uint8_t h)
{Graph *g=p;return n<12U && h<4U?g->branch[n][h]:0U;}
static uint8_t kind(void *p,uint16_t n) {(void)p;(void)n;return 2U;}
static Box pose(void *p,const Actor *a)
{
  Graph *g=p;Box b;int32_t d=0L;
  b.x=g->x[a->node];b.y=g->y[a->node];b.hx=b.hy=40;
  if(a->phase==2U) d=(int32_t)((uint32_t)a->length*a->elapsed/a->total);
  if(a->h==0U)b.y-=d;else if(a->h==1U)b.x+=d;else if(a->h==2U)b.y+=d;else b.x-=d;
  if(a->phase==1U && a->turning)b.hx=b.hy=57;
  return b;
}
static uint8_t visible(void *p,const Box *a,const Box *b)
{
  Graph *g=p;
  int32_t x=abs32(a->x-b->x),y=abs32(a->y-b->y),hi=x>y?x:y,lo=x>y?y:x;
  if(g->try_reentrant) {
    Actor same;Score nested;
    g->try_reentrant=0U;memset(&same,0,sizeof(same));same.node=1U;same.h=1U;same.phase=3U;
    nested=ThiefTerminalForecast_Run(&cfg,&ops,&same,&same,visits,1000UL,0U,3U);
    if(!nested.known && !nested.clear && !nested.reserve_ms)g->guarded_calls++;
  }
  return (uint8_t)(x*x+y*y<=360000L && lo*1000L<=hi*177L);
}
static uint8_t visible_during(void *p,const Box *a0,const Box *a1,const Box *b0,const Box *b1)
{
  unsigned t;
  for(t=0U;t<=256U;t++) {
    Box a=*a0,b=*b0;
    a.x+=(a1->x-a0->x)*(int32_t)t/256L;a.y+=(a1->y-a0->y)*(int32_t)t/256L;
    b.x+=(b1->x-b0->x)*(int32_t)t/256L;b.y+=(b1->y-b0->y)*(int32_t)t/256L;
    if(visible(p,&a,&b))return 1U;
  }
  return 0U;
}
static uint8_t slab(int32_t d,int32_t v,int32_t r,int32_t *lo,int32_t *hi)
{
  int32_t a,b,l,h;
  if(!v)return (uint8_t)(abs32(d)<=r);
  a=(-r-d)*1024L/v;b=(r-d)*1024L/v;l=(a<b?a:b)-1L;h=(a>b?a:b)+1L;
  if(l>*lo)*lo=l;if(h<*hi)*hi=h;return (uint8_t)(*lo<=*hi);
}
static int32_t contact(void *p,const Box *a0,const Box *a1,const Box *b0,const Box *b1)
{
  int32_t lo=0L,hi=1024L,dx=a0->x-b0->x,dy=a0->y-b0->y;
  int32_t rx=(a0->hx>a1->hx?a0->hx:a1->hx)+(b0->hx>b1->hx?b0->hx:b1->hx);
  int32_t ry=(a0->hy>a1->hy?a0->hy:a1->hy)+(b0->hy>b1->hy?b0->hy:b1->hy);
  (void)p;
  if(slab(dx,(a1->x-b1->x)-dx,rx,&lo,&hi) && slab(dy,(a1->y-b1->y)-dy,ry,&lo,&hi))
    return lo<0L?0L:lo;
  return -1L;
}
static uint16_t move_ms(Graph *g,const Actor *a,uint32_t speed)
{
  if(a->mode && speed==1000UL)speed=g->reverse_speed;
  return (uint16_t)(((uint32_t)a->length*1000UL+speed-1UL)/speed);
}
static void command(void *p,Actor *a,uint8_t out,uint32_t speed,uint32_t turn)
{
  Graph *g=p;uint8_t delta=(uint8_t)((out+4U-a->h)%4U),first=a->first;
  g->command_calls++;
  if(a->phase==3U)g->held_command_calls++;
  if(delta==2U)a->mode^=1U;
  a->turning=(uint8_t)(delta%2U);
  if(a->turning)g->quarter_commands++;
  a->h=out;a->to=next(p,a->node,out);a->length=length(p,a->node,out);a->elapsed=0U;
  a->phase=a->turning?1U:2U;a->total=(uint16_t)(a->turning?turn:move_ms(g,a,speed));
  if(first) {
    a->first=0U;a->turning=0U;a->phase=2U;a->total=move_ms(g,a,speed);
    if(g->first_count<32U) {
      g->first_mode[g->first_count]=a->mode;
      g->first_body[g->first_count]=(a->h+2U*a->mode)%4U;g->first_count++;
    }
  }
  if(g->invalid_command)a->mode=3U;
}
static void advance(void *p,Actor *a,uint16_t dt,uint32_t speed)
{
  Graph *g=p;
  if(a->phase==3U)return;
  a->elapsed=(uint16_t)(a->elapsed+dt);
  if(a->elapsed>=a->total) {
    a->elapsed=0U;
    if(a->phase==1U) {a->phase=2U;a->total=move_ms(g,a,speed);}
    else {a->last=a->node;a->node=a->to;a->to=0U;a->phase=0U;g->arrivals[a->node]++;}
  }
  if(g->invalid_advance)a->phase=4U;
}
static uint8_t best_police(void *p,uint16_t node,uint8_t h,uint8_t mode,
                           uint16_t goal,uint32_t speed)
{
  Graph *g=p;uint8_t out,result=4U;int32_t best=0x7FFFFFFFL;
  (void)h;(void)mode;(void)speed;
  for(out=0U;out<4U;out++) {
    uint16_t n=next(p,node,out);int32_t value;
    if(!n)continue;
    value=abs32(g->x[n]-g->x[goal])+abs32(g->y[n]-g->y[goal]);
    if(value<best) {best=value;result=out;}
  }
  return result;
}
static int32_t reach(void *p,const Actor *a,uint16_t goal,uint32_t speed)
{
  Graph *g=p;Box b=pose(p,a);
  return (abs32(g->x[goal]-b.x)+abs32(g->y[goal]-b.y))*1000L/(int32_t)speed;
}
static void edge(unsigned a,unsigned b,unsigned h)
{
  graph.next[a][h]=(uint16_t)b;graph.next[b][(h+2U)%4U]=(uint16_t)a;
  graph.length[a][h]=graph.length[b][(h+2U)%4U]=(uint16_t)
    (abs32(graph.x[a]-graph.x[b])+abs32(graph.y[a]-graph.y[b]));
}
static void reset(void)
{
  memset(&graph,0,sizeof(graph));memset(visits,0,sizeof(visits));
  graph.x[2]=300;graph.x[3]=600;graph.x[4]=600;graph.y[4]=300;
  graph.x[5]=600;graph.y[5]=600;graph.x[6]=-300;graph.x[7]=-600;
  graph.x[8]=-600;graph.y[8]=300;graph.x[9]=900;graph.y[10]=2000;
  edge(1,2,1);edge(2,3,1);edge(3,4,2);edge(4,5,2);
  edge(1,6,3);edge(6,7,3);edge(7,8,2);edge(3,9,1);edge(1,10,2);
  /* Forward corridor leads THROUGH a future corner, not just to immediateT. */
  graph.branch[2][1]=4;graph.branch[3][2]=4;
  graph.branch[6][3]=8;graph.branch[7][2]=8;
  graph.reverse_speed=1000U;
  memset(&cfg,0,sizeof(cfg));cfg.forward_speed_mm_s=1000UL;cfg.turn_ms=100U;
  cfg.reaction_ms=340U;cfg.timing_guard_ms=120U;cfg.contact_mm=80U;
  cfg.wait_limit_ms=6000U;cfg.exit_limit_ms=6000U;cfg.endpoint_budget=96U;cfg.max_edges=12U;
  ops.context=&graph;ops.pose=pose;ops.visible=visible;ops.visible_during=visible_during;
  ops.contact_fraction=contact;ops.command=command;ops.advance=advance;ops.best_police=best_police;
  ops.next=next;ops.length=length;ops.branch=branch;ops.reach=reach;ops.node_kind=kind;
}
static Actor held(unsigned node,unsigned heading)
{
  Actor a;memset(&a,0,sizeof(a));a.node=(uint16_t)node;a.h=(uint8_t)heading;
  a.phase=3U;a.last_seen_h=4U;return a;
}
static Score run(Actor *b,Actor *p,unsigned out)
{
  Score s=ThiefTerminalForecast_Run(&cfg,&ops,b,p,visits,1000UL,0U,(uint8_t)out);
  check(s.steps<=cfg.endpoint_budget);
  if(s.known)test_known_cases++;else test_unknown_cases++;
  if(!s.known)check(!s.clear && s.reserve_ms==0L);
  return s;
}

void test_main(void)
{
  Actor b,p,saved_b,saved_p;Score s,forward,backward;
  unsigned i;
  reset();b=held(1,1);p=held(10,0);saved_b=b;saved_p=p;
  s=run(&b,&p,3);
  check(!s.known && !s.sensed && s.wait_ms==6000U);
  check(graph.held_command_calls==0U);
  check(memcmp(&b,&saved_b,sizeof(b))==0 && memcmp(&p,&saved_p,sizeof(p))==0);
  for(i=0U;i<88U;i++)check(visits[i]==0U);

  reset();b=held(1,1);p=held(1,3);s=run(&b,&p,3);
  check(s.known && !s.clear && s.reserve_ms<0L && s.wait_ms==0U && s.escape_ms==0U);
  check(graph.command_calls==0U); /* Immediate contact is known before planning. */

  reset();b=held(1,1);p=held(2,3);graph.reverse_speed=800U;
  backward=run(&b,&p,3);
  check(backward.known && backward.clear && backward.sensed);
  check(backward.escape_ms==1565U); /* 340 reaction+100 turn+3*375 reverse. */
  for(i=0U;i<graph.first_count;i++)check(graph.first_body[i]==1U && graph.first_mode[i]==1U);
  check(graph.quarter_commands>=2U); /* Path forecast and executed forced turn. */
  reset();b=held(1,3);p=held(2,3);graph.reverse_speed=800U;
  forward=run(&b,&p,3);
  check(forward.known && forward.clear && forward.sensed);
  check(forward.escape_ms==1340U); /* Same geographic path, physical forward. */
  check(backward.escape_ms-forward.escape_ms==225U);
  for(i=0U;i<graph.first_count;i++)check(graph.first_body[i]==3U && graph.first_mode[i]==0U);

  reset();b=held(1,1);p=held(9,3);p.phase=2U;p.to=3;p.length=300U;p.total=300U;
  p.elapsed=100U; /* Starts at800mm, earliest view at600mm after200ms. */
  s=run(&b,&p,3);
  check(s.known && s.sensed && s.wait_ms>=199U && s.wait_ms<=201U);
  test_view_clear=s.clear;test_view_escape_ms=s.escape_ms;
  check(!s.clear && s.escape_ms>=1040U && s.escape_ms<1340U);
  /* 100 ms pivot leaves exactly160mm separation: the rectangular tails meet
   * after rotation when the new transverse leg begins. This must be caught. */

  reset();b=held(1,1);p=held(9,3);p.phase=2U;p.to=3;p.length=300U;p.total=300U;
  p.elapsed=100U;cfg.turn_ms=50U;s=run(&b,&p,3);
  check(s.known && s.clear && s.sensed && s.escape_ms==1290U);

  reset();b=held(1,1);p=held(9,3);p.phase=2U;p.to=3;p.length=300U;p.total=300U;
  cfg.turn_ms=500U;s=run(&b,&p,3);
  check(s.known && s.sensed && !s.clear && s.reserve_ms<=0L);
  check(s.escape_ms>=940U && s.escape_ms<1440U); /* Collision in FIRST forced corner. */
  check(s.wait_ms>=299U && s.wait_ms<=301U);

  reset();b=held(1,1);p=held(9,3);p.phase=2U;p.to=3;p.length=300U;p.total=300U;
  s=run(&b,&p,1);
  check(s.known && s.sensed && !s.clear && s.reserve_ms<0L);
  check(s.escape_ms<940U); /* Oncoming conflict before reaching the corner. */

  reset();b=held(1,1);p=held(2,3);cfg.endpoint_budget=3U;s=run(&b,&p,3);
  check(!s.known && s.steps==3U);
  reset();b=held(1,1);p=held(2,3);cfg.max_edges=2U;s=run(&b,&p,3);
  check(!s.known); /* ImmediateT/straight endpoint alone cannot certify an exit. */
  reset();b=held(1,1);p=held(2,3);graph.branch[7][2]=0U;s=run(&b,&p,3);
  check(!s.known); /* The post-corner arm is dead/pocket: retain fallback. */

  reset();b=held(1,1);p=held(2,3);graph.invalid_command=1U;s=run(&b,&p,3);
  check(!s.known);
  reset();b=held(1,1);p=held(2,3);graph.invalid_advance=1U;s=run(&b,&p,3);
  check(!s.known);
  reset();b=held(1,1);p=held(2,3);s=run(&b,&p,0);
  check(!s.known && s.steps==0U); /* Fore/aft only; no invented immediate sideways turn. */
  cfg.wait_limit_ms=6001U;s=run(&b,&p,3);check(!s.known);
  cfg.wait_limit_ms=6000U;cfg.endpoint_budget=97U;s=run(&b,&p,3);check(!s.known);
  /* N026-like geometry: escape south while first observed threat is west.
   * The west corner arm is cheaper/equal for the thief but approaches police;
   * choose the legal east continuation, then actually pass its corner. */
  reset();
  memset(graph.next,0,sizeof(graph.next));memset(graph.length,0,sizeof(graph.length));
  memset(graph.branch,0,sizeof(graph.branch));
  graph.x[1]=0;graph.y[1]=0;graph.x[2]=0;graph.y[2]=300;
  graph.x[3]=0;graph.y[3]=600;graph.x[4]=-300;graph.y[4]=600;
  graph.x[11]=300;graph.y[11]=600;graph.x[6]=-300;graph.y[6]=0;
  edge(1,2,2);edge(2,3,2);edge(3,4,3);edge(3,11,1);edge(1,6,3);
  graph.branch[2][2]=11;graph.branch[3][3]=4;graph.branch[3][1]=11;
  b=held(1,2);p=held(6,1);s=run(&b,&p,2);
  check(s.known && s.clear && s.sensed);
  check(graph.arrivals[11]>=2U && graph.arrivals[4]==0U);
  check(graph.quarter_commands>=2U);
  reset();b=held(1,1);p=held(2,3);graph.try_reentrant=1U;s=run(&b,&p,3);
  check(s.known && s.clear && graph.guarded_calls==1U);
  test_work_bytes=ThiefTerminalForecast_WorkBytes();
  check(test_work_bytes>100U && test_work_bytes<=512U);
  check(test_known_cases>=6U && test_unknown_cases>=8U);
  test_done=1U;while(1){}
}

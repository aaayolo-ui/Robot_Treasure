#include "FrontLaserGate.h"
#include "ThiefMapPlanner.h"
volatile unsigned test_failures, test_checks, test_done;
static ThiefRoutePlan_t plan;
static FrontLaserGate_t gate[4];
static ThiefMapPose_t pose;
static void check(unsigned ok) { test_checks++; if(!ok) test_failures++; }
static unsigned observe(unsigned direction,unsigned distance,unsigned confidence,unsigned hiding)
{
  static const unsigned offsets[4]={0,3,1,2};
  ThiefMapPose_t ray=pose;
  ray.body_heading=(unsigned char)((pose.body_heading+offsets[direction])%4);
  return FrontLaserGate_Observe(&gate[direction],&ray,(unsigned char)hiding,
                                (unsigned short)distance,(unsigned char)confidence,123);
}
void test_main(void)
{
  unsigned n,h,d,i,mask,ok,next,length,ports;
  unsigned short neighbor,len;
  /* Check every direction transform on every node and orientation against map edges. */
  for(n=1;n<=MAP_ROUTE_NODE_COUNT;n++) for(h=0;h<4;h++) for(d=0;d<4;d++) {
    static const unsigned offsets[4]={0,3,1,2};
    unsigned world=(h+offsets[d])%4;
    ThiefMapPlanner_PoseAtNode((unsigned short)n,(unsigned char)h,&pose);
    FrontLaserGate_Init(&gate[d]);
    ok=ThiefMapPlanner_HasRoad((unsigned short)n,(unsigned char)world,&neighbor,&len);
    for(i=0;i<4;i++) check(observe(d,50,10,1)==0);
    check(observe(d,50,10,1)==ok);
    if(ok) {
      check(gate[d].hit_x_mm==pose.x_mm+(world==1?50:world==3?-50:0));
      check(gate[d].hit_y_mm==pose.y_mm+(world==2?50:world==0?-50:0));
    }
  }
  ThiefMapPlanner_PoseAtNode(59,0,&pose);
  for(d=0;d<4;d++) FrontLaserGate_Init(&gate[d]);
  /* Samples from different channels must not combine into one 3/5 vote. */
  for(d=0;d<4;d++) { observe(d,100,20,1); observe(d,100,20,1); }
  for(d=0;d<4;d++) check(gate[d].history_count==2);
  /* Choose an actual road direction and check quality/range/stability/reset. */
  for(h=0;h<4;h++) if(ThiefMapPlanner_HasRoad(59,h,&neighbor,&len)) break;
  pose.body_heading=h;
  FrontLaserGate_Init(&gate[0]);
  for(i=0;i<5;i++) check(!observe(0,49,20,1));
  for(i=0;i<5;i++) check(!observe(0,601,20,1));
  for(i=0;i<5;i++) check(!observe(0,100,9,1));
  for(i=0;i<5;i++) check(!observe(0,100,20,0));
  FrontLaserGate_ResetEvidence(&gate[0]);
  for(i=0;i<4;i++) check(!observe(0,100,20,1));
  check(observe(0,100,20,1));
  FrontLaserGate_ResetEvidence(&gate[0]);
  check(!observe(0,100,20,1));
  /* Every accepted plan must depart on physical body axis, never toward a detected threat. */
  for(n=1;n<=MAP_ROUTE_NODE_COUNT;n++) if(n==59 || n==53 || n==38) for(h=0;h<4;h++) for(mask=1;mask<16;mask++) {
    ok=ThiefMapPlanner_PlanEscape(n,h,mask,0,&plan);
    if(!ok) continue;
    check(plan.valid && plan.route.count>0);
    check(!(mask & (1U<<plan.steps[0].heading)));
    check((plan.steps[0].heading+2U*plan.route.drive_mode)%4==h);
    for(i=0;i<plan.route.count;i++) {
      const MapRouteStep_t *s=&plan.steps[i];
      next=s->from_node; length=0; ports=0;
      while(next!=s->to_node && ports++<MAP_ROUTE_NODE_COUNT) {
        if(!ThiefMapPlanner_HasRoad(next,s->heading,&neighbor,&len)) break;
        next=neighbor; length+=len;
      }
      check(next==s->to_node && length==s->length_mm);
    }
  }
  check(ThiefMapPlanner_PlanEscape(59,0,15,0,&plan)==0);
  check(ThiefMapPlanner_PlanInitial(&plan));
  check(plan.route.start_node==54 && plan.steps[0].to_node==53);
  test_done=1;
  while(1) {}
}

#include "FrontLaserGate.h"
#include "ThiefMapPlanner.h"
#include "ThiefThreatModel.h"
#include "ThiefRiskSearch.h"
#include "MapRouteNavigator.h"
#include "ThiefStrategyConfig.h"
#include "ThiefMapTopology.h"
volatile unsigned test_failures, test_checks, test_done;
volatile unsigned test_initial_goal, test_initial_steps, test_workspace_bytes;
volatile unsigned test_routes_accepted, test_max_states;
volatile unsigned test_initial_nodes[49];
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
  ThiefMapPlanner_Init();
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
  for(i=0;i<4;i++) check(!observe(0,100,10,1));
  check(observe(0,100,10,1));
  FrontLaserGate_ResetEvidence(&gate[0]);
  check(!observe(0,100,20,1));
#if THIEF_FAST_NEAR_CONFIRM
  check(observe(0,100,20,1));
  FrontLaserGate_ResetEvidence(&gate[0]);
  check(!observe(0,100,20,1));
  check(!observe(0,100,9,1));
  check(!observe(0,100,20,1));
  check(observe(0,100,20,1));
  FrontLaserGate_ResetEvidence(&gate[0]);
  check(!observe(0,100,20,1));
  check(!observe(0,200,20,1)); /* noisy >80 mm two-frame jump */
  FrontLaserGate_ResetEvidence(&gate[0]);
  check(!observe(0,100,20,1));
  check(!observe(0,100,20,0)); /* motion does not confirm an object */
#endif
  /* Every accepted plan must depart on physical body axis, never toward a detected threat. */
  for(n=1;n<=MAP_ROUTE_NODE_COUNT;n++) if(n==59 || n==53 || n==38) for(h=0;h<4;h++) for(mask=1;mask<16;mask++) {
    ok=ThiefMapPlanner_PlanEscape(n,h,mask,0,10000UL,&plan);
    if(!ok) continue;
    test_routes_accepted++;
    if(ThiefMapPlanner_Decision()->states_expanded>test_max_states)
      test_max_states=ThiefMapPlanner_Decision()->states_expanded;
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
      if(i>0) check(plan.steps[i].to_node!=plan.steps[i-1].from_node);
    }
  }
  check(ThiefMapPlanner_PlanEscape(59,0,15,0,10000UL,&plan)==0);
  check(ThiefMapPlanner_PlanInitial(&plan));
  test_initial_goal=plan.goal_node;test_initial_steps=plan.route.count;
  test_workspace_bytes=ThiefMapPlanner_WorkBytes();
  test_initial_nodes[0]=54;
  for(i=0;i<plan.route.count;i++) test_initial_nodes[i+1]=plan.steps[i].to_node;
  check(test_routes_accepted>20);
  check(plan.route.start_node==54 && plan.steps[0].to_node==53);
  check(ThiefRiskSearch_LegTime(300,0,1,0)>ThiefRiskSearch_LegTime(300,0,0,0));
  check(ThiefRiskSearch_LegTime(300,1,0,1)>ThiefRiskSearch_LegTime(300,1,0,0));
  check(ThiefMapPlanner_WorkBytes()<18000UL);
  check(!ThiefRiskSearch_HideSafe(8) && !ThiefRiskSearch_HideSafe(9));
  check(!ThiefRiskSearch_HideSafe(18) && !ThiefRiskSearch_HideSafe(19));
  check(!ThiefRiskSearch_HideSafe(27) && !ThiefRiskSearch_HideSafe(28));
  check(!ThiefRiskSearch_HideSafe(15) && !ThiefRiskSearch_HideSafe(49));
  check(!ThiefRiskSearch_HideSafe(44)); /* two real exits and one dead arm */
  check(!ThiefRiskSearch_HideSafe(76)); /* every parking axis is long */
  check(ThiefRiskSearch_HideSafe(38));
  check(thief_exit_count[37]==4 && (thief_parking_axis_mask[37]&10)==10);
  for(n=1;n<=MAP_ROUTE_NODE_COUNT;n++) if(ThiefRiskSearch_HideSafe(n)) {
    check(thief_exit_count[n-1]>=3);
    check(ThiefMapPlanner_Node(n)->degree>=3);
    for(h=0;h<4;h++) if(thief_parking_axis_mask[n-1]&(1U<<h)) {
      check(ThiefMapPlanner_HasRoad(n,h,&neighbor,&len));
      check(ThiefMapPlanner_HasRoad(n,(h+2)%4,&neighbor,&len));
      check(thief_exit_branch_ms[n-1][h]<65535);
      check(thief_exit_branch_ms[n-1][(h+2)%4]<65535);
    }
  }
  check(ThiefMapPlanner_Decision()->parking_heading==plan.final_body_heading);
  check(ThiefMapPlanner_Decision()->escape_exits>=3);
  {
    static const unsigned targets[]={53,44,36,35,34,33,25,15,6,4};
    static const int actions[]={1,0,-1,0,0,1,0,0,-1,-1};
    static MapRouteNavigator_t nav;
    check(plan.route.count>10);
    for(i=0;i<10;i++) {
      check(plan.steps[i].to_node==targets[i]);
      /* N004 departure may depend on the chosen hide; treasure prefix is fixed. */
      if(i<9) check(plan.steps[i].action==actions[i]);
    }
    check(ThiefRiskSearch_HideSafe(plan.goal_node));
    MapRouteNavigator_StartRuntime(&nav,&plan.route,plan.steps);
    check(!nav.fault);
  }
  /* Actual coordinates/distance affect the reachable set; no observations of
   * the unseen simulator opponent are fed to this model. */
  {
    const MapRouteNode_t *n59=ThiefMapPlanner_Node(59);
    unsigned when=10000;
    int m0,m1;
    ThiefMapPlanner_Init();
    ThiefMapPlanner_ObserveThreat(59,3,n59->x_half*150-600,n59->y_half*150,when);
    check(ThiefThreatModel_Prepare(when)==1);
    m0=ThiefThreatModel_Margin(59,59,0,0);
    check(ThiefThreatModel_Prepare(when+100)==1);
    m1=ThiefThreatModel_Margin(59,59,0,0);
    check(m1<m0);
    check(ThiefMapPlanner_PlanEscape(59,3,8,0,when,&plan));
    check(plan.steps[0].heading==1);
    check(plan.route.drive_mode==1);
    check(plan.goal_node!=59 && ThiefRiskSearch_HideSafe(plan.goal_node));
    check(ThiefMapPlanner_Decision()->threat_count==1);
    check(ThiefMapPlanner_Decision()->states_expanded<=THIEF_STATE_COUNT);
    check(ThiefThreatModel_Prepare(when+THIEF_THREAT_MEMORY_MS+1)==0);
#if THIEF_EMERGENCY_ESCAPE
    /* A negative conservative margin must not stop a feasible retreat. */
    ThiefMapPlanner_Init();
    ThiefMapPlanner_ObserveThreat(59,3,n59->x_half*150-300,n59->y_half*150,when);
    check(ThiefMapPlanner_PlanEscape(59,3,8,0,when,&plan));
    check(plan.valid && plan.steps[0].heading==1 && plan.route.drive_mode==1);
    check(ThiefMapPlanner_Decision()->degraded);
#endif
    ThiefMapPlanner_Init();
    /* Off-road hit is rejected as a belief seed, even if a caller bypasses gate. */
    ThiefMapPlanner_ObserveThreat(59,0,123,456,when);
    check(!ThiefThreatModel_Prepare(when));
    ThiefMapPlanner_Init();
    ThiefMapPlanner_ObserveThreat(59,3,n59->x_half*150-50,n59->y_half*150,when);
#if THIEF_EMERGENCY_ESCAPE
    /* Range to a surface is not a calibrated centre-to-centre capture signal.
     * A close return must not reinstate the old risk-induced parking bug. */
    check(ThiefMapPlanner_PlanEscape(59,3,8,0,when,&plan));
    check(plan.valid && plan.steps[0].heading==1 && plan.route.drive_mode==1);
#else
    check(!ThiefMapPlanner_PlanEscape(59,3,8,0,when,&plan));
    check(!plan.valid);
#endif
    /* A timer wrap must not turn a fresh observation into a stale one. */
    ThiefMapPlanner_Init();
    ThiefMapPlanner_ObserveThreat(59,3,n59->x_half*150-600,n59->y_half*150,0xFFFFFFF0U);
    check(ThiefThreatModel_Prepare(20U)==1);
  }
#if THIEF_EMERGENCY_ESCAPE
  {
    ThiefMapPlanner_Init();
    for(i=1;i<=MAP_ROUTE_NODE_COUNT;i++)
      if(ThiefRiskSearch_HideSafe(i)) ThiefRiskSearch_MarkExposed(i,10000UL);
    check(ThiefMapPlanner_PlanEscape(59,3,8,39,10000UL,&plan));
    check(plan.steps[0].heading==1);
    check(ThiefMapPlanner_Decision()->reused_hide);
    check(ThiefMapPlanner_Decision()->degraded);
    /* All exits observed remains a hard constraint; do not drive blind. */
    check(!ThiefMapPlanner_PlanEscape(59,3,15,39,10000UL,&plan));
  }
#endif
  test_done=1;
  while(1) {}
}

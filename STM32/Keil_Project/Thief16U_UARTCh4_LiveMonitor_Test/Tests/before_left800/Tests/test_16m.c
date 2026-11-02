#include "ThiefMapPlanner.h"
#include "ThiefInterceptPolicy.h"
#include "ThiefThreatModel.h"
#include "ThiefStrategyConfig.h"
#include "FrontLaserGate.h"
#include "MapLaserMask.h"
#include "MapRouteNavigator.h"
#include "ThiefLocalization.h"
#include <stddef.h>
volatile unsigned test_checks,test_failures,test_first_line,test_done,test_case;
static ThiefRoutePlan_t plan;
static MapRouteNavigator_t nav;
#define CHECK(x) do {test_checks++;if(!(x)){test_failures++;if(!test_first_line)test_first_line=__LINE__;}} while(0)
static uint16_t Gray(const MapRouteStep_t *s)
{
  uint16_t raw=0x0060U;
  if(ThiefMapPlanner_HasRoad(s->to_node,(uint8_t)((s->heading+3U)%4U),NULL,NULL)) raw|=0x0C00U;
  if(ThiefMapPlanner_HasRoad(s->to_node,(uint8_t)((s->heading+1U)%4U),NULL,NULL)) raw|=3U;
  return s->target_kind==4U?0x0FFFU:raw;
}
static void CheckPlan(uint16_t start,uint8_t body)
{
  uint16_t i;
  CHECK(plan.valid && plan.route.start_node==start);
  CHECK(ThiefIntercept_HideAxis(plan.goal_node)&(1U<<plan.final_body_heading));
  MapRouteNavigator_StartRuntime(&nav,&plan.route,plan.steps);
  CHECK(!nav.fault);
  {
    int32_t minimum=(int32_t)plan.steps[0].length_mm*4L/10L-50L;
    CHECK(MapRouteNavigator_InArrivalWindow(&nav,minimum-1L)); /* Runtime: actual gray takes priority. */
    CHECK(MapRouteNavigator_InArrivalWindow(&nav,minimum));
    CHECK(MapRouteNavigator_InArrivalWindow(&nav,plan.steps[0].length_mm));
    CHECK(MapRouteNavigator_InArrivalWindow(&nav,(int32_t)plan.steps[0].length_mm*16L/10L+101L));
  }
  for(i=0U;i<plan.route.count;i++) {
    CHECK(plan.steps[i].length_mm>0U);
    CHECK(ThiefMapPlanner_Node(plan.steps[i].to_node)->kind!=0U);
  }
  /* A root departure can be same-body forward/reverse or a real quarter turn. */
  CHECK((plan.steps[0].heading+2U*plan.steps[0].drive_mode+4U-body)%4U!=2U);
}
static void TestSensors(void)
{
  FrontLaserGate_t g;
  ThiefMapPose_t p;
  uint8_t h,i;
  uint16_t clearance[4]={500U,500U,500U,500U};
  CHECK(THIEF_LASER_CHANNEL_MASK==0x0FU);
  CHECK(THIEF_LASER_MAX_MM==1000U);
  CHECK(THIEF_LASER_TRIGGER_MM==1000U);
  for(h=0U;h<4U;h++) {
    uint8_t back=(uint8_t)((h+2U)%4U);
    CHECK(ThiefIntercept_ShouldReverse(h,(uint8_t)(1U<<h),clearance,0U));
    CHECK(ThiefIntercept_ShouldReverse(h,(uint8_t)(1U<<((h+1U)%4U)),clearance,0U));
    CHECK(ThiefIntercept_ShouldReverse(h,(uint8_t)(1U<<((h+3U)%4U)),clearance,0U));
    CHECK(!ThiefIntercept_ShouldReverse(h,(uint8_t)(1U<<back),clearance,0U));
    CHECK(ThiefIntercept_ShouldReverse(h,15U,clearance,0U));
    CHECK(!ThiefIntercept_ShouldReverse(h,(uint8_t)(1U<<h),clearance,1U));
    ThiefMapPlanner_PoseAtNode(1U,h,&p);
    for(i=0U;i<4U;i++) {
      static const uint8_t offsets[4]={0U,3U,1U,2U};
      uint8_t world=(uint8_t)((h+offsets[i])%4U);
      CHECK(((MapLaserMask_DetectionMask(&p)>>i)&1U)==(world==1U || world==2U));
    }
  }
  ThiefMapPlanner_PoseAtNode(1U,1U,&p);
  FrontLaserGate_Init(&g);
  CHECK(FrontLaserGate_Observe(&g,&p,1U,1000U,30U,1U,0U));
  CHECK(FrontLaserGate_Observe(&g,&p,1U,1000U,30U,1U,0U));
  CHECK(g.status==FRONT_LASER_OPPONENT);
  CHECK(g.hit_x_mm==1150 && g.hit_y_mm==0 && g.distance_mm==1000U);
  CHECK(!FrontLaserGate_Observe(&g,&p,1U,1001U,30U,1U,0U));
  CHECK(g.status==FRONT_LASER_TOO_FAR);
  FrontLaserGate_ResetEvidence(&g);
  CHECK(!FrontLaserGate_Observe(&g,&p,1U,999U,5U,1U,0U));
  CHECK(!FrontLaserGate_Observe(&g,&p,1U,0U,30U,1U,0U));
  p.body_heading=3U;
  CHECK(!FrontLaserGate_Observe(&g,&p,1U,800U,30U,1U,0U));
  /* Every probe: out-of-effective-range samples must never trigger, including the slow
   * five-frame path; crossing out of the trigger range clears near evidence. */
  for(i=0U;i<4U;i++) {
    static const uint16_t threshold[4]={1000U,1000U,900U,1000U};
    uint8_t frame;
    ThiefMapPlanner_PoseAtNode(1U,1U,&p);FrontLaserGate_Init(&g);
    for(frame=0U;frame<8U;frame++) {
      CHECK(!FrontLaserGate_Observe(&g,&p,1U,1001U,30U,1U,i));
      CHECK(g.status==FRONT_LASER_TOO_FAR);
    }
    FrontLaserGate_ResetEvidence(&g);
    CHECK(FrontLaserGate_Observe(&g,&p,1U,threshold[i],30U,1U,i));
    CHECK(g.hit_x_mm==((i==0U || i==3U)?1150:(i==2U?1000:1100)));
    CHECK(!FrontLaserGate_Observe(&g,&p,1U,threshold[i]+1U,30U,1U,i));
    CHECK(g.status==FRONT_LASER_TOO_FAR);
    CHECK(FrontLaserGate_Observe(&g,&p,1U,threshold[i],30U,1U,i));
    /* A clearly off-field hit must not survive five otherwise good frames. */
    p.x_mm=100;p.y_mm=0;p.body_heading=3U;FrontLaserGate_Init(&g);
    for(frame=0U;frame<8U;frame++) {
      CHECK(!FrontLaserGate_Observe(&g,&p,1U,400U,30U,1U,i));
      CHECK(g.status==FRONT_LASER_OUTSIDE_FIELD);
    }
    /* Nearby west edge, but looking inward along its road: keep detection. */
    p.body_heading=1U;FrontLaserGate_ResetEvidence(&g);
    CHECK(FrontLaserGate_Observe(&g,&p,1U,400U,30U,1U,i));
  }
  ThiefMapPlanner_PoseAtNode(1U,1U,&p);FrontLaserGate_Init(&g);
  CHECK(FrontLaserGate_Observe(&g,&p,1U,380U,60U,1U,0U));
  CHECK(!FrontLaserGate_HideConfirmed(&g));
  CHECK(!FrontLaserGate_Observe(&g,&p,1U,422U,0U,1U,0U));
  CHECK(!FrontLaserGate_HideConfirmed(&g));
  for(i=0U;i<3U;i++) {
    CHECK(FrontLaserGate_Observe(&g,&p,1U,380U,60U,1U,0U));
    CHECK(FrontLaserGate_HideConfirmed(&g)==(i==2U));
  }
  FrontLaserGate_ResetEvidence(&g);
  CHECK(!FrontLaserGate_HideConfirmed(&g));
  CHECK(!MapLaserMask_FrameAllowed(15U,0U,100U,100U));
  CHECK(!MapLaserMask_FrameAllowed(15U,0U,99U,100U));
  CHECK(MapLaserMask_FrameAllowed(15U,0U,101U,100U));
  ThiefIntercept_Reset();ThiefIntercept_BlockLeg(4U,6U,0xFFFFFF00UL);
  CHECK(ThiefIntercept_EdgeBlocked(4U,5U,10U));
  CHECK(ThiefIntercept_EdgeBlocked(6U,5U,10U));
  CHECK(!ThiefIntercept_EdgeBlocked(4U,5U,5000U));
  CHECK(!ThiefIntercept_EdgeBlocked(1U,2U,10U));
}
static void TestReverse(void)
{
  ThiefLocalization_t l;
  ThiefLocYaw_t y={0,100U,1U};
  MapRouteStep_t s={11U,12U,300U,0U,1U,1U,3U,2U,MAP_ACTION_STOP,0U},r;
  int progress;
  for(progress=0;progress<=300;progress+=25) {
    int16_t x,z;
    CHECK(ThiefLocalization_Start(&l,11U,1U,&y,100U));
    CHECK(ThiefLocalization_BeginStep(&l,&s,&y,100U));
    CHECK(ThiefLocalization_Update(&l,progress,&y,100U));
    x=l.pose.x_mm;z=l.pose.y_mm;
    r=s;r.from_node=12U;r.to_node=11U;r.heading=3U;r.drive_mode=1U;
    r.expected_ports=4U;r.target_kind=3U;
    CHECK(ThiefLocalization_ReverseLeg(&l,&r,progress,&y,100U));
    CHECK(l.pose.x_mm==x && l.pose.y_mm==z && l.confirmed_node==11U);
    CHECK(l.pose.body_heading==1U && l.pose.drive_mode==1U);
    CHECK(ThiefLocalization_Update(&l,300-progress,&y,100U));
    /* A second interception returns along the same partial leg continuously. */
    CHECK(ThiefLocalization_ReverseLeg(&l,&s,300-progress,&y,100U));
    CHECK(l.pose.x_mm==x && l.pose.y_mm==z && l.confirmed_node==11U);
    CHECK(ThiefLocalization_ReverseLeg(&l,&r,progress,&y,100U));
    CHECK(ThiefLocalization_Arrive(&l,&r,Gray(&r),300,&y,100U));
    CHECK(l.confirmed_node==11U && l.phase==THIEF_LOC_AT_NODE);
  }
  CHECK(ThiefLocalization_Start(&l,11U,1U,&y,100U));
  CHECK(ThiefLocalization_BeginStep(&l,&s,&y,100U));
  CHECK(ThiefLocalization_ReverseLeg(&l,&r,581,&y,100U));
  CHECK(l.reversal_entry_mm==-281L && l.confirmed_node==11U); /* No gray fix from arithmetic. */
  /* Root quarter turn, learned CW sign, then forward departure. */
  CHECK(ThiefLocalization_Start(&l,11U,1U,&y,100U));
  s.from_node=11U;s.to_node=22U;s.heading=2U;s.target_kind=2U;s.expected_ports=3U;
  CHECK(ThiefLocalization_PrepareDeparture(&l,&s,&y,100U));
  CHECK(l.phase==THIEF_LOC_AT_JUNCTION && !l.pose.valid);
  y.yaw_x100=4500;
  CHECK(!ThiefLocalization_ExitReady(&l,&s,1U,&y,100U));
  y.yaw_x100=9000;
  CHECK(ThiefLocalization_ExitReady(&l,&s,1U,&y,100U));
  CHECK(ThiefLocalization_CommitExit(&l,&s,&y,100U));
  CHECK(ThiefLocalization_BeginStep(&l,&s,&y,100U));
  CHECK(l.pose.body_heading==2U && l.yaw_cw_sign==1);
}
/* Independent field-rectangle oracle. Latest user requirement permits
 * cross-road rays: connectivity is for planning, not sensor acceptance. */
static uint16_t OracleReach(int16_t x,int16_t y,uint8_t heading)
{
  if(heading==0U)return (uint16_t)(y+150);
  if(heading==1U)return (uint16_t)(3450-x);
  if(heading==2U)return (uint16_t)(2850-y);
  return (uint16_t)(x+150);
}
static void TestAllRoadGeometry(void)
{
  static const uint16_t ranges[]={0U,49U,50U,399U,400U,550U,551U,600U,601U,777U,778U,779U,849U,850U,851U,899U,900U,901U,999U,1000U,1001U,65535U};
  uint16_t i;
  uint8_t point,h,r,d;
  FrontLaserGate_t gate;
  ThiefMapPose_t p;
  for(i=0U;i<MAP_ROUTE_SEGMENT_COUNT;i++) {
    const MapRouteSegment_t *e=&map_route_segments[i];
    const MapRouteNode_t *a=&map_route_nodes[e->start-1U],*b=&map_route_nodes[e->end-1U];
    for(point=0U;point<3U;point++) {
      p.x_mm=(int16_t)(a->x_half*150+(b->x_half-a->x_half)*75*point);
      p.y_mm=(int16_t)(a->y_half*150+(b->y_half-a->y_half)*75*point);
      p.node_id=e->start;p.segment_progress_mm=0U;p.drive_mode=0U;p.valid=1U;
      for(h=0U;h<4U;h++) {
        uint16_t reach=OracleReach(p.x_mm,p.y_mm,h);
        p.body_heading=h;p.travel_heading=h;
        for(r=0U;r<sizeof(ranges)/sizeof(ranges[0]);r++) {
          uint16_t raw=ranges[r];
          for(d=0U;d<4U;d++) {
            uint32_t center=(uint32_t)raw+((d==0U || d==3U)?150U:100U);
            uint8_t expected=(uint8_t)(raw>=50U && raw<=(d==2U?900U:1000U) && center<=reach);
            FrontLaserGate_Init(&gate);
            CHECK(FrontLaserGate_Observe(&gate,&p,1U,raw,30U,1U,d)==expected);
            CHECK(FrontLaserGate_Observe(&gate,&p,1U,raw,30U,1U,d)==expected);
            CHECK(gate.distance_mm==raw);
          }
        }
      }
    }
  }
}
static void TestEveryReverseLeg(void)
{
  uint16_t n;
  uint8_t h,mode,k;
  ThiefLocYaw_t y={0,100U,1U};
  ThiefLocalization_t l;
  for(n=1U;n<=MAP_ROUTE_NODE_COUNT;n++) if(map_route_nodes[n-1U].kind) {
    for(h=0U;h<4U;h++) {
      uint16_t to,length,part,count=1U;
      MapRouteStep_t s,r;
      if(!ThiefMapPlanner_HasRoad(n,h,&to,&length)) continue;
      while(!map_route_nodes[to-1U].kind) {
        CHECK(ThiefMapPlanner_HasRoad(to,h,&to,&part));length+=part;count++;
      }
      s.from_node=n;s.to_node=to;s.heading=h;s.length_mm=length;
      s.segment_count=count;s.segment_offset=0U;s.action=MAP_ACTION_STOP;
      s.target_kind=map_route_nodes[to-1U].kind;s.expected_ports=map_route_nodes[to-1U].degree;
      for(mode=0U;mode<2U;mode++) for(k=0U;k<=4U;k++) {
        uint8_t body=(uint8_t)((h+2U*mode)%4U);
        int32_t progress=(int32_t)length*k/4L;
        int16_t x,z;
        s.drive_mode=mode;r=s;r.from_node=to;r.to_node=n;
        r.heading=(uint8_t)((h+2U)%4U);r.drive_mode^=1U;
        r.target_kind=map_route_nodes[n-1U].kind;r.expected_ports=map_route_nodes[n-1U].degree;
        CHECK(ThiefLocalization_Start(&l,n,body,&y,100U));
        CHECK(ThiefLocalization_BeginStep(&l,&s,&y,100U));
        CHECK(ThiefLocalization_Update(&l,progress,&y,100U));
        x=l.pose.x_mm;z=l.pose.y_mm;
        CHECK(ThiefLocalization_ReverseLeg(&l,&r,progress,&y,100U));
        CHECK(l.pose.x_mm==x && l.pose.y_mm==z && l.confirmed_node==n);
        CHECK(l.pose.body_heading==body && l.pose.drive_mode==(mode^1U));
        CHECK(ThiefLocalization_Update(&l,length,&y,100U));
        CHECK(ThiefLocalization_Arrive(&l,&r,Gray(&r),length,&y,100U));
        CHECK(l.confirmed_node==n && l.phase==THIEF_LOC_AT_NODE);
      }
    }
  }
}
/* A 76-degree quarter turn with an existing 15-degree entry offset is
 * outside the global 25-degree window, but reaches it by 90 degrees. */
static void TestTurnConvergence(void)
{
  static const int16_t origins[3]={0,17000,-17000};
  uint8_t mode,o;
  int sign,quarter;
  for(mode=0U;mode<2U;mode++) for(o=0U;o<3U;o++)
    for(sign=-1;sign<=1;sign+=2) for(quarter=-1;quarter<=1;quarter+=2) {
      ThiefLocalization_t l;
      ThiefLocYaw_t y;
      MapRouteStep_t s={12U,11U,300U,0U,1U,3U,4U,3U,0,0U},next;
      int32_t raw;
      uint8_t body=(uint8_t)((3U+2U*mode)%4U);
      y.yaw_x100=origins[o];y.stamp_ms=100U;y.valid=1U;
      s.drive_mode=mode;s.action=(int8_t)quarter;
      CHECK(ThiefLocalization_Start(&l,12U,body,&y,100U));
      l.yaw_cw_sign=(int8_t)sign;
      CHECK(ThiefLocalization_BeginStep(&l,&s,&y,100U));
      raw=(int32_t)origins[o]-sign*quarter*1500L;
      while(raw>18000L) raw-=36000L;
      while(raw< -18000L) raw+=36000L;
      y.yaw_x100=(int16_t)raw;
      CHECK(ThiefLocalization_Arrive(&l,&s,0xFFFU,300L,&y,100U));
      next=s;next.from_node=11U;next.to_node=quarter>0?2U:22U;
      next.heading=(uint8_t)((3+quarter+4)%4);next.action=MAP_ACTION_STOP;
      next.target_kind=2U;next.expected_ports=3U;
      raw=(int32_t)y.yaw_x100+sign*quarter*7600L;
      while(raw>18000L) raw-=36000L;
      while(raw< -18000L) raw+=36000L;
      y.yaw_x100=(int16_t)raw;
      CHECK(!ThiefLocalization_ExitReady(&l,&next,1U,&y,100U));
      CHECK(l.error==THIEF_LOC_OK && l.phase==THIEF_LOC_AT_JUNCTION);
      raw=(int32_t)y.yaw_x100+sign*quarter*1400L;
      while(raw>18000L) raw-=36000L;
      while(raw< -18000L) raw+=36000L;
      y.yaw_x100=(int16_t)raw;
      CHECK(!ThiefLocalization_ExitReady(&l,&next,0U,&y,100U));
      CHECK(ThiefLocalization_ExitReady(&l,&next,1U,&y,100U));
      CHECK(ThiefLocalization_CommitExit(&l,&next,&y,100U));
      CHECK(l.origin_yaw_x100==origins[o]);
      CHECK(ThiefLocalization_BeginStep(&l,&next,&y,100U));
    }
}
void test_main(void)
{
  static const uint16_t hides[5]={11U,38U,26U,35U,63U};
  uint16_t node;
  uint8_t i,h,mask;
  TestSensors();TestReverse();TestTurnConvergence();TestAllRoadGeometry();TestEveryReverseLeg();
  for(node=1U;node<=MAP_ROUTE_NODE_COUNT;node++)
    CHECK((ThiefIntercept_HideAxis(node)!=0U)==
        (node==11U || node==38U || node==26U || node==35U || node==63U));
  ThiefMapPlanner_Init();
  CHECK(ThiefMapPlanner_PlanInitial(&plan));
  CheckPlan(54U,3U);CHECK(plan.goal_node==38U);
  CHECK(plan.steps[0].from_node==54U && plan.steps[0].to_node==53U);
  for(i=0U;i<5U;i++) for(h=0U;h<4U;h++) if(ThiefIntercept_HideAxis(hides[i])&(1U<<h)) {
    const MapRouteNode_t *n=ThiefMapPlanner_Node(hides[i]);
    for(mask=0U;mask<3U;mask++) {
      test_case++;
      ThiefMapPlanner_Init();
      if(mask) ThiefThreatModel_Record(0U,n->x_half*150,n->y_half*150,100U);
      CHECK(ThiefMapPlanner_PlanEscape(hides[i],h,mask==2U?15U:(uint8_t)(1U<<h),0U,100U,&plan));
      CheckPlan(hides[i],h);
      CHECK(plan.goal_node!=hides[i]);
      if(mask==2U) CHECK(ThiefMapPlanner_Decision()->degraded);
    }
  }
  /* Recently blocked corridor is avoided when an alternative exists. */
  ThiefMapPlanner_Init();ThiefIntercept_BlockLeg(11U,12U,100U);
  CHECK(ThiefMapPlanner_PlanEscape(11U,1U,2U,0U,100U,&plan));
  CheckPlan(11U,1U);CHECK(plan.steps[0].to_node!=12U);
  /* Any confirmed junction can become a retreat endpoint. Ensure every
   * body heading has a legal least-bad escape, including all-exits-blocked. */
  for(node=1U;node<=MAP_ROUTE_NODE_COUNT;node++) if(map_route_nodes[node-1U].kind) {
    for(h=0U;h<4U;h++) {
      test_case++;
      ThiefMapPlanner_Init();
      CHECK(ThiefMapPlanner_PlanEscape(node,h,15U,0U,100U,&plan));
      CheckPlan(node,h);
    }
  }
  /* More than one hide is reachable/selected as observations move. */
  test_done=1U;
  for(;;) {}
}

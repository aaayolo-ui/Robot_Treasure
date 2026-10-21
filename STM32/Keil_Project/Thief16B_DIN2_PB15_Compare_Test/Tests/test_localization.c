#include "ThiefLocalization.h"
#include "ThiefStrategyConfig.h"

volatile unsigned test_checks,test_failures,test_done;
volatile unsigned test_routes,test_confirmed_nodes;
volatile unsigned test_first_line,test_first_error,test_first_from,test_first_to,test_first_phase;
static ThiefLocalization_t loc;
static ThiefRoutePlan_t plan;
static unsigned now;

static void checked(unsigned ok,unsigned line)
{
  test_checks++;
  if(!ok) {
    if(!test_failures) {
      test_first_line=line; test_first_error=loc.error; test_first_from=loc.step.from_node;
      test_first_to=loc.step.to_node; test_first_phase=loc.phase;
    }
    test_failures++;
  }
}
#define check(ok) checked((ok),__LINE__)
static short wrapped(int value)
{
  while(value>18000) value-=36000;
  while(value< -18000) value+=36000;
  return (short)value;
}
static ThiefLocYaw_t yaw_at(unsigned body,unsigned origin,int sign)
{
  ThiefLocYaw_t y;
  y.yaw_x100=wrapped(17800+((int)body-(int)origin)*9000*sign);
  y.stamp_ms=now; y.valid=1;
  return y;
}
static unsigned gray_for(const MapRouteStep_t *s)
{
  if(ThiefMapPlanner_HasRoad(s->to_node,(unsigned char)((s->heading+3)%4),0,0)) return 0xC60;
  if(ThiefMapPlanner_HasRoad(s->to_node,(unsigned char)((s->heading+1)%4),0,0)) return 0x063;
  return 0xFFF;
}
static void drive_plan(int sign)
{
  unsigned i,origin=(plan.steps[0].heading+2*plan.steps[0].drive_mode)%4;
  ThiefLocYaw_t y=yaw_at(origin,origin,sign);
  check(ThiefLocalization_Start(&loc,plan.route.start_node,(unsigned char)origin,&y,now));
  for(i=0;i<plan.route.count;i++) {
    const MapRouteStep_t *s=&plan.steps[i];
    unsigned body=(s->heading+2*s->drive_mode)%4;
    now+=20; y=yaw_at(body,origin,sign);
    check(ThiefLocalization_BeginStep(&loc,s,&y,now));
    check(ThiefLocalization_Update(&loc,s->length_mm+1,&y,now));
    check(loc.confirmed_node==s->from_node && loc.pose.node_id==s->from_node);
    check(loc.pose.valid && loc.pose.body_heading==body);
    check(ThiefLocalization_Arrive(&loc,s,(unsigned short)gray_for(s),s->length_mm,&y,now));
    check(loc.confirmed_node==s->to_node);
    test_confirmed_nodes++;
    if(i+1<plan.route.count) {
      const MapRouteStep_t *next=&plan.steps[i+1];
      unsigned out=(next->heading+2*next->drive_mode)%4;
      check(!loc.pose.valid);
      now+=20; y=yaw_at(out,origin,sign);
      check(!ThiefLocalization_ExitReady(&loc,next,0,&y,now));
      check(ThiefLocalization_ExitReady(&loc,next,1,&y,now));
      check(ThiefLocalization_CommitExit(&loc,next,&y,now));
      check(loc.body_heading==out && loc.confirmed_node==next->from_node);
      check(loc.pose.valid);
    } else {
      check(loc.pose.valid && loc.phase==THIEF_LOC_AT_NODE);
      check(ThiefLocalization_Update(&loc,0,&y,now));
    }
  }
  check(loc.confirmed_node==plan.goal_node && loc.body_heading==plan.final_body_heading);
  test_routes++;
}

void test_main(void)
{
  unsigned node,body,mask,i;
  int sign;
  ThiefLocYaw_t y;
  MapRouteStep_t next;
  ThiefLocalization_t saved;
  now=1000;
  for(sign=-1;sign<=1;sign+=2) {
    ThiefMapPlanner_Init(); check(ThiefMapPlanner_PlanInitial(&plan));
    check(plan.goal_node==38);
    drive_plan(sign); /* Full closed-loop event order, both sensor yaw signs/wrap. */
    for(node=11;node<=62;node++) {
      if(node!=11 && node!=12 && node!=38 && node!=62) continue;
      for(body=0;body<4;body++) for(i=0;i<2;i++) {
        mask=1U<<((body+2*i)%4);
        ThiefMapPlanner_Init();
        if(ThiefMapPlanner_PlanEscape((unsigned short)node,(unsigned char)body,
            (unsigned char)mask,0,now,&plan)) drive_plan(sign);
      }
    }
  }
  ThiefMapPlanner_Init(); check(ThiefMapPlanner_PlanInitial(&plan));
  y=yaw_at(3,3,1); y.valid=0;
  check(!ThiefLocalization_Start(&loc,54,3,&y,now));
  check(loc.error==THIEF_LOC_GYRO_STALE && !loc.pose.valid);
  y.valid=1; y.stamp_ms=now-151;
  check(!ThiefLocalization_Start(&loc,54,3,&y,now));
  y.stamp_ms=now;
  check(ThiefLocalization_Start(&loc,54,3,&y,now));
  check(loc.pose.x_mm==3300 && loc.pose.y_mm==1500);
  check(ThiefLocalization_BeginStep(&loc,&plan.steps[0],&y,now));
  saved=loc;
  check(!ThiefLocalization_Arrive(&loc,&plan.steps[0],0x060,300,&y,now));
  check(loc.error==THIEF_LOC_GRAY_MISMATCH && loc.confirmed_node==54);
  loc=saved;
  check(!ThiefLocalization_Arrive(&loc,&plan.steps[0],(unsigned short)gray_for(&plan.steps[0]),0,&y,now));
  check(loc.error==THIEF_LOC_DISTANCE && loc.confirmed_node==54);
  loc=saved;
  y.yaw_x100=wrapped(y.yaw_x100+3000);
  check(!ThiefLocalization_Update(&loc,150,&y,now));
  check(loc.error==THIEF_LOC_HEADING_MISMATCH && !loc.pose.valid);
  loc=saved; y=yaw_at(3,3,1); y.stamp_ms=now-151;
  check(!ThiefLocalization_Update(&loc,150,&y,now));
  check(loc.error==THIEF_LOC_GYRO_STALE);
  loc=saved; y.stamp_ms=now;
  y.yaw_x100=wrapped(y.yaw_x100+2000);
  check(ThiefLocalization_Arrive(&loc,&plan.steps[0],(unsigned short)gray_for(&plan.steps[0]),300,&y,now));
  y.yaw_x100=wrapped(y.yaw_x100+10000);
  check(!ThiefLocalization_ExitReady(&loc,&plan.steps[1],1,&y,now));
  check(loc.error==THIEF_LOC_HEADING_MISMATCH); /* Relative turn alone cannot mask global drift. */
  loc=saved; y=yaw_at(3,3,1);
  check(ThiefLocalization_Arrive(&loc,&plan.steps[0],(unsigned short)gray_for(&plan.steps[0]),300,&y,now));
  next=plan.steps[1];
  y.yaw_x100=wrapped(y.yaw_x100+4500);
  check(!ThiefLocalization_ExitReady(&loc,&next,1,&y,now));
  check(loc.yaw_cw_sign==0 && loc.body_heading==3);
  y=yaw_at(0,3,1); y.stamp_ms=now-151;
  check(!ThiefLocalization_ExitReady(&loc,&next,1,&y,now));
  y.stamp_ms=now;
  saved=loc;
  next.heading=2; check(!ThiefLocalization_ExitReady(&loc,&next,1,&y,now));
  check(loc.error==THIEF_LOC_STEP_MISMATCH);
  loc=saved; next=plan.steps[1];
  check(ThiefLocalization_CommitExit(&loc,&next,&y,now));
  check(loc.yaw_cw_sign==1);
  check(ThiefLocalization_BeginStep(&loc,&next,&y,now));
  check(ThiefLocalization_Arrive(&loc,&next,(unsigned short)gray_for(&next),next.length_mm,&y,now));
  check(ThiefLocalization_CommitExit(&loc,&plan.steps[2],&y,now));
  check(ThiefLocalization_BeginStep(&loc,&plan.steps[2],&y,now));
  check(ThiefLocalization_Arrive(&loc,&plan.steps[2],(unsigned short)gray_for(&plan.steps[2]),
                               plan.steps[2].length_mm,&y,now));
  /* Existing sign cannot flip to accept a later physically wrong turn. */
  y.yaw_x100=wrapped(y.yaw_x100+9000); /* N036 needs a left turn west. */
  check(!ThiefLocalization_ExitReady(&loc,&plan.steps[3],1,&y,now));
  check(loc.error==THIEF_LOC_TURN_DIRECTION);
  now=20; y.yaw_x100=100; y.stamp_ms=0xFFFFFFF0U; y.valid=1;
  check(!ThiefLocalization_Start(&loc,5,3,&y,now)); /* Treasure marker is not a gray landmark. */
  check(ThiefLocalization_Start(&loc,54,3,&y,now));
  check(loc.gyro_age_ms==36);
  saved=loc;
  next=plan.steps[0]; next.to_node=5;
  check(!ThiefLocalization_BeginStep(&loc,&next,&y,now));
  check(loc.error==THIEF_LOC_STEP_MISMATCH);
  loc=saved;
  check(ThiefLocalization_BeginStep(&loc,&plan.steps[0],&y,now));
  check(ThiefLocalization_Arrive(&loc,&plan.steps[0],(unsigned short)gray_for(&plan.steps[0]),300,&y,now));
  y.stamp_ms=now-151;
  check(!ThiefLocalization_Update(&loc,300,&y,now)); /* Stale gyro during a turn cannot be ignored. */
  check(loc.error==THIEF_LOC_GYRO_STALE && !loc.pose.valid);
  test_done=1;
  for(;;){}
}

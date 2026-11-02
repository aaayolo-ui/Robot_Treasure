#include "ThiefMapPlanner.h"
#include <stddef.h>
#include "ThiefThreatModel.h"
#include "ThiefStrategyConfig.h"
#include "ThiefInterceptPolicy.h"
#include <string.h>

#define ESCAPE_ID 1601U
#define INITIAL_ID 1600U
static uint16_t departure[MAP_ROUTE_NODE_COUNT];
static uint16_t combined[MAP_ROUTE_NODE_COUNT];
static ThiefDecision_t decision;

void ThiefMapPlanner_Init(void)
{
  ThiefRiskSearch_Init();
  ThiefThreatModel_Init();
  ThiefIntercept_Reset();
  decision.goal_node=0U;
}
void ThiefMapPlanner_ObserveThreat(uint16_t node,uint8_t heading,
    int16_t x,int16_t y,uint32_t now)
{
  ThiefThreatModel_Record(heading,x,y,now);
  ThiefRiskSearch_MarkExposed(node,now);
}
const ThiefDecision_t *ThiefMapPlanner_Decision(void) {return &decision;}
uint32_t ThiefMapPlanner_WorkBytes(void)
{
  return ThiefRiskSearch_WorkBytes()+ThiefThreatModel_WorkBytes()+
    (uint32_t)(sizeof(departure)+sizeof(combined)+sizeof(decision));
}

const MapRouteNode_t *ThiefMapPlanner_Node(uint16_t node_id)
{
  if ((node_id == 0U) || (node_id > MAP_ROUTE_NODE_COUNT)) return NULL;
  if (map_route_nodes[node_id - 1U].id != node_id) return NULL;
  return &map_route_nodes[node_id - 1U];
}

uint8_t ThiefMapPlanner_HasRoad(uint16_t node_id, uint8_t heading,
                                uint16_t *next_node, uint16_t *length_mm)
{
  uint16_t i;
  for (i = 0U; i < MAP_ROUTE_SEGMENT_COUNT; i++) {
    const MapRouteSegment_t *segment = &map_route_segments[i];
    uint8_t out_heading;
    uint16_t other;
    if (segment->start == node_id) {
      out_heading = segment->heading;
      other = segment->end;
    } else if (segment->end == node_id) {
      out_heading = (uint8_t)((segment->heading + 2U) % 4U);
      other = segment->start;
    } else continue;
    if (out_heading == heading) {
      if (next_node != NULL) *next_node = other;
      if (length_mm != NULL) *length_mm = segment->length_mm;
      return 1U;
    }
  }
  return 0U;
}

static uint8_t SegmentBetween(uint16_t from, uint16_t to,
                              uint8_t *heading, uint16_t *length)
{
  uint16_t i;
  for (i = 0U; i < MAP_ROUTE_SEGMENT_COUNT; i++) {
    const MapRouteSegment_t *segment = &map_route_segments[i];
    if ((segment->start == from) && (segment->end == to)) {
      *heading = segment->heading;
      *length = segment->length_mm;
      return 1U;
    }
    if ((segment->end == from) && (segment->start == to)) {
      *heading = (uint8_t)((segment->heading + 2U) % 4U);
      *length = segment->length_mm;
      return 1U;
    }
  }
  return 0U;
}

static uint8_t BuildRoute(const uint16_t *nodes, uint16_t node_count,
                          uint8_t start_drive_mode, uint16_t route_id,
                          ThiefRoutePlan_t *plan)
{
  uint16_t i = 0U;
  uint16_t step_count = 0U;
  uint8_t drive_mode = start_drive_mode;
  if ((plan == NULL) || (node_count < 2U)) return 0U;

  while (i + 1U < node_count) {
    uint16_t j = (uint16_t)(i + 1U);
    uint16_t length = 0U;
    uint16_t segment_count = 0U;
    uint8_t heading, part_heading;
    uint16_t part_length;
    if (SegmentBetween(nodes[i], nodes[j], &heading, &part_length) == 0U) return 0U;
    length = part_length;
    segment_count = 1U;
    while ((j + 1U < node_count) &&
           (ThiefMapPlanner_Node(nodes[j])->kind == 0U)) {
      if (SegmentBetween(nodes[j], nodes[j + 1U], &part_heading, &part_length) == 0U)
        return 0U;
      if (part_heading != heading) return 0U;
      length = (uint16_t)(length + part_length);
      segment_count++;
      j++;
    }
    if ((ThiefMapPlanner_Node(nodes[j])->kind == 0U) ||
        (step_count >= THIEF_ROUTE_MAX_STEPS)) return 0U;
    plan->steps[step_count].from_node = nodes[i];
    plan->steps[step_count].to_node = nodes[j];
    plan->steps[step_count].length_mm = length;
    plan->steps[step_count].segment_offset = 0U;
    plan->steps[step_count].segment_count = segment_count;
    plan->steps[step_count].heading = heading;
    plan->steps[step_count].expected_ports = ThiefMapPlanner_Node(nodes[j])->degree;
    plan->steps[step_count].target_kind = ThiefMapPlanner_Node(nodes[j])->kind;
    plan->steps[step_count].drive_mode = drive_mode;
    plan->steps[step_count].action = MAP_ACTION_STOP;
    step_count++;
    i = j;
  }

  for (i = 0U; i + 1U < step_count; i++) {
    uint8_t difference = (uint8_t)((plan->steps[i + 1U].heading + 4U -
                                    plan->steps[i].heading) % 4U);
    if (difference == 3U) plan->steps[i].action = MAP_ACTION_LEFT;
    else if (difference == 2U) {
      plan->steps[i].action = MAP_ACTION_REVERSE;
      drive_mode ^= 1U;
    } else plan->steps[i].action = (int8_t)difference;
    plan->steps[i + 1U].drive_mode = drive_mode;
  }

  plan->route.id = route_id;
  plan->route.offset = 0U;
  plan->route.count = step_count;
  plan->route.start_node = nodes[0];
  plan->route.heading = plan->steps[0].heading;
  plan->route.drive_mode = start_drive_mode;
  plan->goal_node = nodes[node_count - 1U];
  plan->final_body_heading = (uint8_t)((plan->steps[step_count - 1U].heading +
      (plan->steps[step_count - 1U].drive_mode != 0U ? 2U : 0U)) % 4U);
  plan->valid = 1U;
  return 1U;
}

/* User-selected outbound route. Keep every observable intermediate junction.
 * W from N054; right at N053, left at N036, right at N033,
 * left at N006. N005 is crossed without stopping toward N004. */
static const uint16_t initial_treasure_path[] = {
  54U, 53U, 44U, 36U, 35U, 34U, 33U, 25U, 15U, 6U, 5U, 4U
};

uint8_t ThiefMapPlanner_PlanInitial(ThiefRoutePlan_t *plan)
{
  uint16_t departure_count,i,count;
  uint8_t first_mode;
  const uint16_t prefix_count=(uint16_t)(sizeof(initial_treasure_path)/sizeof(initial_treasure_path[0]));
  if(plan==NULL) return 0U;
  plan->valid=0U;
  /* Preserve the exact user-selected treasure prefix, then optimize the hide. */
  if(!ThiefRiskSearch_Select(4U,3U,0U,0U,0U,5U,0U,0UL,
      departure,&departure_count,&first_mode,&decision)) return 0U;
  if(prefix_count+departure_count-1U>MAP_ROUTE_NODE_COUNT) return 0U;
  count=0U;
  for(i=0U;i<prefix_count;i++) combined[count++]=initial_treasure_path[i];
  for(i=1U;i<departure_count;i++) combined[count++]=departure[i];
  if(!BuildRoute(combined,count,0U,INITIAL_ID,plan)) return 0U;
  /* Initial telemetry reports the entire fixed-prefix + hide route, not only
   * the N004-to-hide suffix used by the hide selector. */
  decision.travel_ms=THIEF_STARTUP_MS;
  for(i=0U;i<plan->route.count;i++) {
    uint8_t turn=0U,change=0U;
    if(i>0U) {
      turn=(plan->steps[i-1U].action==MAP_ACTION_LEFT ||
            plan->steps[i-1U].action==MAP_ACTION_RIGHT)?1U:0U;
      change=(plan->steps[i-1U].action==MAP_ACTION_REVERSE)?1U:0U;
    }
    decision.travel_ms+=ThiefRiskSearch_LegTime(plan->steps[i].length_mm,
        plan->steps[i].drive_mode,turn,change);
  }
  return 1U;
}

uint8_t ThiefMapPlanner_PlanEscape(uint16_t current_node,
    uint8_t body_heading,uint8_t threat_mask,uint16_t previous_hide_node,
    uint32_t now,ThiefRoutePlan_t *plan)
{
  uint16_t count;uint8_t mode,pass,found=0U,allowed,preferred;
  if(!plan || body_heading>3U)return 0U;
  plan->valid=0U;allowed=current_node==11U?10U:15U;
  if(current_node==63U && !(threat_mask&1U) && (threat_mask&8U)) {
    static const uint16_t temporary_path[]={63U,78U};
    /* Already north-facing: reverse-follow south without a chassis U-turn. */
    mode=(uint8_t)(body_heading==0U);
    if(!BuildRoute(temporary_path,2U,mode,ESCAPE_ID,plan))return 0U;
    memset(&decision,0,sizeof(decision));
    decision.goal_node=78U;decision.parking_heading=plan->final_body_heading;
    decision.travel_ms=THIEF_STARTUP_MS+ThiefRiskSearch_LegTime(600U,mode,
        (uint8_t)(body_heading%2U),0U);
    decision.escape_exits=3U;decision.planner_kind=4U;
    return 1U;
  }
  preferred=allowed;
  if(current_node==11U) {
    if(threat_mask&3U) {allowed=8U;preferred=allowed;}
    else if(threat_mask&12U) {allowed=2U;preferred=allowed;}
  } else if(current_node==38U && threat_mask) {
    /* Vertically parked N038: either map side -> upper hide through N021;
     * north -> east N039-N040. South retains the east escape. */
    allowed=(uint8_t)((threat_mask&10U)?1U:2U);
    preferred=allowed;
  } else if(current_node==63U && (threat_mask&1U)) {
    /* North pursuer: commit the whole south/west transfer in one plan.
     * N078 is a turn, not the temporary-wait destination in this case. */
    allowed=4U;preferred=allowed;
  } else if(current_node==78U) {
    if(threat_mask&1U)allowed=8U;       /* north -> N077-N076-N075 */
    else if(threat_mask&8U)allowed=1U;  /* west -> N063-N052 -> upper hide */
    preferred=allowed;
  }
  for(pass=0U;pass<3U && !found;pass++) {
    ThiefRiskSearch_SetEmergency((uint8_t)(pass==2U));
    ThiefRiskSearch_SetDepartureMask(pass==0U?preferred:allowed);
    found=ThiefRiskSearch_Select(current_node,body_heading,0U,1U,threat_mask,
        0U,previous_hide_node,now,departure,&count,&mode,&decision);
  }
  ThiefRiskSearch_SetEmergency(0U);ThiefRiskSearch_SetDepartureMask(15U);
  return found?BuildRoute(departure,count,mode,ESCAPE_ID,plan):0U;
}

void ThiefMapPlanner_PoseAtNode(uint16_t node_id, uint8_t body_heading,
                                ThiefMapPose_t *pose)
{
  const MapRouteNode_t *node = ThiefMapPlanner_Node(node_id);
  if (pose == NULL) return;
  pose->valid = 0U;
  if (node == NULL) return;
  pose->x_mm = (int16_t)(node->x_half * 150);
  pose->y_mm = (int16_t)(node->y_half * 150);
  pose->node_id = node_id;
  pose->segment_progress_mm = 0U;
  pose->travel_heading = body_heading;
  pose->body_heading = body_heading;
  pose->drive_mode = 0U;
  pose->valid = 1U;
}

void ThiefMapPlanner_PoseOnStep(const MapRouteStep_t *step,
                                int32_t distance_mm,
                                ThiefMapPose_t *pose)
{
  const MapRouteNode_t *from;
  int32_t clamped;
  if ((pose == NULL) || (step == NULL)) return;
  pose->valid = 0U;
  from = ThiefMapPlanner_Node(step->from_node);
  if (from == NULL) return;
  clamped = distance_mm;
  if (clamped < 0L) clamped = 0L;
  if (clamped > step->length_mm) clamped = step->length_mm;
  pose->x_mm = (int16_t)(from->x_half * 150);
  pose->y_mm = (int16_t)(from->y_half * 150);
  if (step->heading == 0U) pose->y_mm = (int16_t)(pose->y_mm - clamped);
  else if (step->heading == 1U) pose->x_mm = (int16_t)(pose->x_mm + clamped);
  else if (step->heading == 2U) pose->y_mm = (int16_t)(pose->y_mm + clamped);
  else pose->x_mm = (int16_t)(pose->x_mm - clamped);
  pose->node_id = (clamped >= step->length_mm) ?
                  step->to_node : step->from_node;
  pose->segment_progress_mm = (uint16_t)clamped;
  pose->travel_heading = step->heading;
  pose->drive_mode = step->drive_mode;
  pose->body_heading = (uint8_t)((step->heading +
      (step->drive_mode != 0U ? 2U : 0U)) % 4U);
  pose->valid = 1U;
}


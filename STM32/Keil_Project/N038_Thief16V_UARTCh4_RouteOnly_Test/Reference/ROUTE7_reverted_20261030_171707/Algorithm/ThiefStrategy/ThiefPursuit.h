#ifndef THIEF_PURSUIT_H
#define THIEF_PURSUIT_H
#include "ThiefMapPlanner.h"

typedef struct {
  int16_t x,y;
  uint32_t first,last;
  uint8_t count;
} ThiefPursuitRay;
typedef struct {
  ThiefPursuitRay rays[4];
  int16_t x,y;
  uint32_t stamp;
} ThiefPursuit;
typedef struct {
  int16_t x,y;
  int32_t body_cw_x100;
  uint8_t travel_heading;
} ThiefPursuitPose;
void ThiefPursuit_Reset(ThiefPursuit *p);
void ThiefPursuit_Observe(ThiefPursuit *p,uint8_t channel,uint16_t raw,
    uint8_t confidence,const ThiefPursuitPose *pose,uint32_t tick);
uint8_t ThiefPursuit_GoalMask(const ThiefPursuit *p,const MapRouteStep_t *goal,
    uint32_t now);
uint8_t ThiefPursuit_Append(ThiefRoutePlan_t *plan,const ThiefRoutePlan_t *tail);
#endif

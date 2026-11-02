#include "TravelFrame.h"
#include "nchd12_rear.h"
#include "ChassisSpeedControl.h"

static uint8_t reverse_travel;

void TravelFrame_SetReverse(uint8_t reverse)
{
  reverse_travel = (reverse != 0U) ? 1U : 0U;
}

uint8_t TravelFrame_IsReverse(void)
{
  return reverse_travel;
}

NCHD12_Status_t TravelFrame_ReadLeading(uint16_t *raw16)
{
  return (reverse_travel != 0U) ? NCHD12Rear_ReadRaw16(raw16) :
                                 NCHD12_ReadRaw16(raw16);
}

NCHD12_Status_t TravelFrame_ReadTrailing(uint16_t *raw16)
{
  return (reverse_travel != 0U) ? NCHD12_ReadRaw16(raw16) :
                                 NCHD12Rear_ReadRaw16(raw16);
}

void TravelFrame_SetWheelTargets(int32_t fl, int32_t fr, int32_t rl, int32_t rr)
{
  if (reverse_travel != 0U)
  {
    /* Same controller output as forward travel; only rotate coordinates. */
    ChassisSpeedControl_SetWheelTargets(-rr, -rl, -fr, -fl);
  }
  else
  {
    ChassisSpeedControl_SetWheelTargets(fl, fr, rl, rr);
  }
}

int16_t TravelFrame_Yaw(int16_t chassis_yaw)
{
  int32_t yaw = chassis_yaw;
  if (reverse_travel != 0U)
  {
    yaw += 18000L;
    if (yaw > 18000L) yaw -= 36000L;
  }
  /* Relative yaw and in-place left/right PWM retain their physical signs. */
  return (int16_t)yaw;
}

uint16_t TravelFrame_TrailingToLeadingGray(uint16_t raw12)
{
  uint16_t result=0U;uint8_t i;
  for(i=0U;i<12U;i++) {result=(uint16_t)((result<<1U)|(raw12&1U));raw12>>=1U;}
  return result;
}

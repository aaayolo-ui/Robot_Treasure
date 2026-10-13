#include "ReverseRouteFrame.h"
#include "nchd12_rear.h"
#include "ChassisSpeedControl.h"

void ReverseRouteFrame_LeadingInit(void)
{
  NCHD12Rear_Init();
}

NCHD12_Status_t ReverseRouteFrame_LeadingProbe(void)
{
  uint16_t raw16;
  /* The rear driver already has a fixed verified address (0x20). */
  return NCHD12Rear_ReadRaw16(&raw16);
}

NCHD12_Status_t ReverseRouteFrame_LeadingConfigure(void)
{
  return NCHD12Rear_ConfigureInputs();
}

NCHD12_Status_t ReverseRouteFrame_LeadingRead(uint16_t *raw16)
{
  return NCHD12Rear_ReadRaw16(raw16);
}

void ReverseRouteFrame_TrailingInit(void)
{
  NCHD12_Init();
}

NCHD12_Status_t ReverseRouteFrame_TrailingConfigure(void)
{
  /* The front driver requires address discovery before register access. */
  if (NCHD12_HasSelectedDevice() == 0U)
  {
    NCHD12_Status_t status = NCHD12_ScanBus();
    if (status != NCHD12_STATUS_OK)
    {
      return status;
    }
  }
  return NCHD12_ConfigureInputs();
}

NCHD12_Status_t ReverseRouteFrame_TrailingRead(uint16_t *raw16)
{
  return NCHD12_ReadRaw16(raw16);
}

void ReverseRouteFrame_SetWheelTargets(int32_t fl, int32_t fr,
                                      int32_t rl, int32_t rr)
{
  /* virtual FL = physical RR, virtual FR = physical RL, etc.
   * Negation reverses travel; exchanging sides preserves negative feedback.
   * In-place yaw keeps its original sign under a 180-degree rotation, so
   * the baseline equal-PWM turn functions need no sign change. */
  ChassisSpeedControl_SetWheelTargets(-rr, -rl, -fr, -fl);
}

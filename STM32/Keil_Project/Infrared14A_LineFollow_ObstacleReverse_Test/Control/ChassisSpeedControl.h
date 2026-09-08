#ifndef CHASSIS_SPEED_CONTROL_H
#define CHASSIS_SPEED_CONTROL_H

#include "Chassis.h"
#include "WheelSpeedPi.h"

void ChassisSpeedControl_Init(void);
void ChassisSpeedControl_Enable(void);
void ChassisSpeedControl_Disable(void);
void ChassisSpeedControl_Stop(void);
void ChassisSpeedControl_SetWheelTargets(int32_t fl_rpm_x10,
                                          int32_t fr_rpm_x10,
                                          int32_t rl_rpm_x10,
                                          int32_t rr_rpm_x10);
void ChassisSpeedControl_SetMotionTarget(ChassisMotion_t motion,
                                         int32_t target_rpm_x10);
void ChassisSpeedControl_Update(uint32_t elapsed_ms);
const WheelSpeedPi_t *ChassisSpeedControl_GetController(ChassisWheelId_t wheel);
uint8_t ChassisSpeedControl_IsEnabled(void);
uint8_t ChassisSpeedControl_IsDisabled(void);
uint8_t ChassisSpeedControl_HasStall(void);
ChassisWheelId_t ChassisSpeedControl_GetStallWheel(void);

#endif /* CHASSIS_SPEED_CONTROL_H */

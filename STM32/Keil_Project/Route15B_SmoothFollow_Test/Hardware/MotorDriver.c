#include "MotorDriver.h"
#include "main.h"

#define MOTOR_DIRECTION_CHANGE_DELAY_MS 10U

typedef struct
{
  TIM_HandleTypeDef *timer;
  uint32_t channel;
  GPIO_TypeDef *in1_port;
  uint16_t in1_pin;
  GPIO_TypeDef *in2_port;
  uint16_t in2_pin;
} MotorConfig_t;

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

static const MotorConfig_t motor_config[MOTOR_ID_COUNT] =
{
  {&htim8, TIM_CHANNEL_1, MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin,
   MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin},
  {&htim8, TIM_CHANNEL_2, MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin,
   MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin},
  {&htim5, TIM_CHANNEL_2, MOTOR_C_IN1_GPIO_Port, MOTOR_C_IN1_Pin,
   MOTOR_C_IN2_GPIO_Port, MOTOR_C_IN2_Pin},
  {&htim5, TIM_CHANNEL_3, MOTOR_D_IN1_GPIO_Port, MOTOR_D_IN1_Pin,
   MOTOR_D_IN2_GPIO_Port, MOTOR_D_IN2_Pin}
};

static uint16_t motor_pwm[MOTOR_ID_COUNT];
static MotorDirection_t motor_direction[MOTOR_ID_COUNT];
static uint8_t motor_driver_enabled;

static uint8_t Motor_IsValid(MotorId_t motor)
{
  return ((uint32_t)motor < (uint32_t)MOTOR_ID_COUNT) ? 1U : 0U;
}

static uint16_t Motor_LimitPwm(uint16_t pwm)
{
  return (pwm > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : pwm;
}

static void Motor_SetCompare(MotorId_t motor, uint16_t pwm)
{
  __HAL_TIM_SET_COMPARE(motor_config[motor].timer,
                        motor_config[motor].channel,
                        pwm);
  motor_pwm[motor] = pwm;
}

static void Motor_SetDirectionPins(MotorId_t motor, MotorDirection_t direction)
{
  GPIO_PinState in1_state = GPIO_PIN_RESET;
  GPIO_PinState in2_state = GPIO_PIN_RESET;

  if (direction == MOTOR_DIRECTION_FORWARD)
  {
    in1_state = GPIO_PIN_SET;
  }
  else if (direction == MOTOR_DIRECTION_REVERSE)
  {
    in2_state = GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(motor_config[motor].in1_port,
                    motor_config[motor].in1_pin,
                    in1_state);
  HAL_GPIO_WritePin(motor_config[motor].in2_port,
                    motor_config[motor].in2_pin,
                    in2_state);
}

static void Motor_StopRaw(MotorId_t motor)
{
  Motor_SetCompare(motor, 0U);
  Motor_SetDirectionPins(motor, MOTOR_DIRECTION_STOP);
  motor_direction[motor] = MOTOR_DIRECTION_STOP;
}

void MotorDriver_Init(void)
{
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  Motor_StopAll();
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
  motor_driver_enabled = 0U;
}

void MotorDriver_Enable(void)
{
  Motor_StopAll();
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
  motor_driver_enabled = 1U;
}

void MotorDriver_Disable(void)
{
  Motor_StopAll();
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
  motor_driver_enabled = 0U;
}

uint8_t MotorDriver_IsEnabled(void)
{
  return motor_driver_enabled;
}

void Motor_SetPwm(MotorId_t motor, uint16_t pwm)
{
  if (Motor_IsValid(motor) == 0U)
  {
    return;
  }

  pwm = Motor_LimitPwm(pwm);
  if (pwm == 0U)
  {
    Motor_Stop(motor);
  }
  else if ((motor_driver_enabled != 0U) &&
           (motor_direction[motor] != MOTOR_DIRECTION_STOP))
  {
    Motor_SetCompare(motor, pwm);
  }
}

void Motor_Forward(MotorId_t motor, uint16_t pwm)
{
  if (Motor_IsValid(motor) == 0U)
  {
    return;
  }

  pwm = Motor_LimitPwm(pwm);
  if (pwm == 0U)
  {
    Motor_Stop(motor);
    return;
  }
  if (motor_driver_enabled == 0U)
  {
    return;
  }

  if (motor_direction[motor] == MOTOR_DIRECTION_REVERSE)
  {
    Motor_StopRaw(motor);
    HAL_Delay(MOTOR_DIRECTION_CHANGE_DELAY_MS);
  }

  if (motor_direction[motor] != MOTOR_DIRECTION_FORWARD)
  {
    Motor_SetDirectionPins(motor, MOTOR_DIRECTION_FORWARD);
    motor_direction[motor] = MOTOR_DIRECTION_FORWARD;
  }
  Motor_SetCompare(motor, pwm);
}

void Motor_Reverse(MotorId_t motor, uint16_t pwm)
{
  if (Motor_IsValid(motor) == 0U)
  {
    return;
  }

  pwm = Motor_LimitPwm(pwm);
  if (pwm == 0U)
  {
    Motor_Stop(motor);
    return;
  }
  if (motor_driver_enabled == 0U)
  {
    return;
  }

  if (motor_direction[motor] == MOTOR_DIRECTION_FORWARD)
  {
    Motor_StopRaw(motor);
    HAL_Delay(MOTOR_DIRECTION_CHANGE_DELAY_MS);
  }

  if (motor_direction[motor] != MOTOR_DIRECTION_REVERSE)
  {
    Motor_SetDirectionPins(motor, MOTOR_DIRECTION_REVERSE);
    motor_direction[motor] = MOTOR_DIRECTION_REVERSE;
  }
  Motor_SetCompare(motor, pwm);
}

void Motor_Stop(MotorId_t motor)
{
  if (Motor_IsValid(motor) != 0U)
  {
    Motor_StopRaw(motor);
  }
}

void Motor_StopAll(void)
{
  MotorId_t motor;

  for (motor = MOTOR_ID_A; motor < MOTOR_ID_COUNT; motor++)
  {
    Motor_StopRaw(motor);
  }
}

uint16_t Motor_GetPwm(MotorId_t motor)
{
  return (Motor_IsValid(motor) != 0U) ? motor_pwm[motor] : 0U;
}

MotorDirection_t Motor_GetDirection(MotorId_t motor)
{
  return (Motor_IsValid(motor) != 0U) ? motor_direction[motor] :
                                         MOTOR_DIRECTION_STOP;
}

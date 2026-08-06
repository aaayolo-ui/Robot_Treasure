#include "Motor.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

#define MOTOR_A_PWM_MAX 999U
#define MOTOR_A_DIRECTION_CHANGE_DELAY_MS 50U

static uint16_t MotorA_LimitPwm(uint16_t pwm)
{
  return (pwm > MOTOR_A_PWM_MAX) ? MOTOR_A_PWM_MAX : pwm;
}

void MotorA_Init(void)
{
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  MotorA_Stop();
}

void MotorA_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
}

void MotorA_Forward(uint16_t pwm)
{
  MotorA_Stop();
  HAL_Delay(MOTOR_A_DIRECTION_CHANGE_DELAY_MS);
  HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MotorA_LimitPwm(pwm));
}

void MotorA_Reverse(uint16_t pwm)
{
  MotorA_Stop();
  HAL_Delay(MOTOR_A_DIRECTION_CHANGE_DELAY_MS);
  HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MotorA_LimitPwm(pwm));
}

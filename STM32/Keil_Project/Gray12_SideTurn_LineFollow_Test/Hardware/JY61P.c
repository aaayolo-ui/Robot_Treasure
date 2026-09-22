#include "JY61P.h"
#include "main.h"

extern UART_HandleTypeDef huart5;

#define JY61P_FRAME_HEADER       0x55U
#define JY61P_FRAME_ANGLE        0x53U
#define JY61P_FRAME_LENGTH       11U
#define JY61P_POLL_BYTES_MAX     32U

static uint8_t jy61p_frame[JY61P_FRAME_LENGTH];
static uint8_t jy61p_frame_index;
static uint8_t jy61p_yaw_valid;
static int16_t jy61p_yaw_x100;
static uint32_t jy61p_last_update_ms;

static uint8_t JY61P_FrameChecksumValid(void)
{
  uint8_t index;
  uint8_t checksum = 0U;

  for (index = 0U; index < (JY61P_FRAME_LENGTH - 1U); index++)
  {
    checksum = (uint8_t)(checksum + jy61p_frame[index]);
  }
  return (checksum == jy61p_frame[JY61P_FRAME_LENGTH - 1U]) ? 1U : 0U;
}

static void JY61P_ParseByte(uint8_t value)
{
  int16_t raw_yaw;

  if (jy61p_frame_index == 0U)
  {
    if (value == JY61P_FRAME_HEADER)
    {
      jy61p_frame[jy61p_frame_index++] = value;
    }
    return;
  }

  if ((jy61p_frame_index == 1U) && (value != JY61P_FRAME_ANGLE))
  {
    jy61p_frame_index = (value == JY61P_FRAME_HEADER) ? 1U : 0U;
    if (jy61p_frame_index != 0U)
    {
      jy61p_frame[0] = value;
    }
    return;
  }

  jy61p_frame[jy61p_frame_index++] = value;
  if (jy61p_frame_index < JY61P_FRAME_LENGTH)
  {
    return;
  }

  if (JY61P_FrameChecksumValid() != 0U)
  {
    raw_yaw = (int16_t)(((uint16_t)jy61p_frame[7] << 8) |
                        (uint16_t)jy61p_frame[6]);
    jy61p_yaw_x100 = (int16_t)(((int32_t)raw_yaw * 18000L) / 32768L);
    jy61p_last_update_ms = HAL_GetTick();
    jy61p_yaw_valid = 1U;
  }
  jy61p_frame_index = 0U;
}

void JY61P_Init(void)
{
  jy61p_frame_index = 0U;
  jy61p_yaw_valid = 0U;
  jy61p_yaw_x100 = 0;
  jy61p_last_update_ms = 0U;
}

void JY61P_Task(void)
{
  uint8_t byte;
  uint8_t count;

  for (count = 0U; count < JY61P_POLL_BYTES_MAX; count++)
  {
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE) == RESET)
    {
      break;
    }
    if (HAL_UART_Receive(&huart5, &byte, 1U, 0U) != HAL_OK)
    {
      break;
    }
    JY61P_ParseByte(byte);
  }
}

uint8_t JY61P_GetYawX100(int16_t *yaw_x100)
{
  if ((yaw_x100 == 0) || (jy61p_yaw_valid == 0U))
  {
    return 0U;
  }
  *yaw_x100 = jy61p_yaw_x100;
  return 1U;
}

uint32_t JY61P_GetLastUpdateMs(void)
{
  return jy61p_last_update_ms;
}

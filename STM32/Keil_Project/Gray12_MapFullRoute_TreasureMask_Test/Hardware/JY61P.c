#include "JY61P.h"
#include "main.h"

extern UART_HandleTypeDef huart5;

#define JY61P_FRAME_SIZE       11U
#define JY61P_FRAME_HEADER     0x55U
#define JY61P_ANGLE_FRAME      0x53U

static uint8_t jy61p_rx_buffer[JY61P_FRAME_SIZE];
static uint8_t jy61p_rx_byte;
static uint8_t jy61p_rx_state;
static uint8_t jy61p_rx_index;
static volatile JY61P_Data_t jy61p_data;

static int16_t JY61P_ReadS16(uint8_t low, uint8_t high)
{
  return (int16_t)(((uint16_t)high << 8U) | (uint16_t)low);
}

static int16_t JY61P_RawAngleToX100(int16_t raw_angle)
{
  return (int16_t)(((int32_t)raw_angle * 18000L) / 32768L);
}

static void JY61P_ResetParser(void)
{
  jy61p_rx_state = 0U;
  jy61p_rx_index = 0U;
}

static void JY61P_ArmReceive(void)
{
  if (HAL_UART_Receive_IT(&huart5, &jy61p_rx_byte, 1U) != HAL_OK)
  {
    jy61p_data.uart_error_count++;
  }
}

void JY61P_ReceiveByte(uint8_t rx_data)
{
  uint8_t index;
  uint8_t checksum = 0U;

  if (jy61p_rx_state == 0U)
  {
    if (rx_data == JY61P_FRAME_HEADER)
    {
      jy61p_rx_buffer[0] = rx_data;
      jy61p_rx_index = 1U;
      jy61p_rx_state = 1U;
    }
    return;
  }

  if (jy61p_rx_state == 1U)
  {
    if (rx_data == JY61P_ANGLE_FRAME)
    {
      jy61p_rx_buffer[1] = rx_data;
      jy61p_rx_index = 2U;
      jy61p_rx_state = 2U;
    }
    else if (rx_data == JY61P_FRAME_HEADER)
    {
      /* Keep the new header and wait for its frame type. */
      jy61p_rx_buffer[0] = rx_data;
      jy61p_rx_index = 1U;
    }
    else
    {
      JY61P_ResetParser();
    }
    return;
  }

  jy61p_rx_buffer[jy61p_rx_index++] = rx_data;
  if (jy61p_rx_index < JY61P_FRAME_SIZE)
  {
    return;
  }

  for (index = 0U; index < (JY61P_FRAME_SIZE - 1U); index++)
  {
    checksum = (uint8_t)(checksum + jy61p_rx_buffer[index]);
  }

  if (checksum == jy61p_rx_buffer[JY61P_FRAME_SIZE - 1U])
  {
    /* JY61P angles are signed. int16_t is required for negative angles. */
    jy61p_data.roll_x100 = JY61P_RawAngleToX100(
        JY61P_ReadS16(jy61p_rx_buffer[2], jy61p_rx_buffer[3]));
    jy61p_data.pitch_x100 = JY61P_RawAngleToX100(
        JY61P_ReadS16(jy61p_rx_buffer[4], jy61p_rx_buffer[5]));
    jy61p_data.yaw_x100 = JY61P_RawAngleToX100(
        JY61P_ReadS16(jy61p_rx_buffer[6], jy61p_rx_buffer[7]));
    jy61p_data.frame_count++;
    jy61p_data.last_update_ms = HAL_GetTick();
    jy61p_data.valid = 1U;
  }
  else
  {
    jy61p_data.checksum_error_count++;
  }

  JY61P_ResetParser();
}

void JY61P_Init(void)
{
  jy61p_data.roll_x100 = 0;
  jy61p_data.pitch_x100 = 0;
  jy61p_data.yaw_x100 = 0;
  jy61p_data.frame_count = 0U;
  jy61p_data.checksum_error_count = 0U;
  jy61p_data.uart_error_count = 0U;
  jy61p_data.last_update_ms = 0U;
  jy61p_data.valid = 0U;
  JY61P_ResetParser();
  JY61P_ArmReceive();
}

void JY61P_Task(void)
{
  /* UART5 normally rearms reception from the byte-complete interrupt. If a
   * blocking UART error left HAL idle, restart it from the main loop. */
  if (huart5.RxState == HAL_UART_STATE_READY)
  {
    JY61P_ArmReceive();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != UART5)
  {
    return;
  }

  JY61P_ReceiveByte(jy61p_rx_byte);
  JY61P_ArmReceive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != UART5)
  {
    return;
  }

  jy61p_data.uart_error_count++;
  JY61P_ResetParser();
  if (huart5.RxState == HAL_UART_STATE_READY)
  {
    JY61P_ArmReceive();
  }
}

void JY61P_GetData(JY61P_Data_t *out)
{
  if (out == 0)
  {
    return;
  }
  *out = jy61p_data;
}

uint8_t JY61P_IsValid(void)
{
  return jy61p_data.valid;
}

uint32_t JY61P_GetFrameCount(void)
{
  return jy61p_data.frame_count;
}

uint32_t JY61P_GetChecksumErrorCount(void)
{
  return jy61p_data.checksum_error_count;
}

uint32_t JY61P_GetUartErrorCount(void)
{
  return jy61p_data.uart_error_count;
}

uint8_t JY61P_GetPitchX100(int16_t *pitch_x100)
{
  if ((pitch_x100 == 0) || (jy61p_data.valid == 0U))
  {
    return 0U;
  }
  *pitch_x100 = jy61p_data.pitch_x100;
  return 1U;
}

uint8_t JY61P_GetYawX100(int16_t *yaw_x100)
{
  if ((yaw_x100 == 0) || (jy61p_data.valid == 0U))
  {
    return 0U;
  }
  *yaw_x100 = jy61p_data.yaw_x100;
  return 1U;
}

uint32_t JY61P_GetLastUpdateMs(void)
{
  return jy61p_data.last_update_ms;
}

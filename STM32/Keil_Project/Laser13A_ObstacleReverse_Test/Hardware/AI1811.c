#include "AI1811.h"
#include "main.h"

#define AI1811_FRAME_LENGTH 8U
#define AI1811_FRAME_HEADER 0xAAU
#define AI1811_FRAME_TYPE   0x05U

extern UART_HandleTypeDef huart2;

static uint8_t rx_byte;
static uint8_t frame[AI1811_FRAME_LENGTH];
static uint8_t frame_length;
static volatile AI1811_Status_t last_status;

static void AI1811_StartReceive(void)
{
  (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
}

static void AI1811_ResetParser(void)
{
  frame_length = 0U;
}

static void AI1811_ProcessByte(uint8_t byte)
{
  uint8_t index;
  uint8_t checksum = 0U;
  uint16_t distance_mm;

  if (frame_length == 0U)
  {
    if (byte == AI1811_FRAME_HEADER)
    {
      frame[0] = byte;
      frame_length = 1U;
    }
    return;
  }

  if (frame_length == 1U)
  {
    if (byte == AI1811_FRAME_TYPE)
    {
      frame[1] = byte;
      frame_length = 2U;
    }
    else if (byte == AI1811_FRAME_HEADER)
    {
      frame[0] = byte;
    }
    else
    {
      AI1811_ResetParser();
    }
    return;
  }

  frame[frame_length] = byte;
  frame_length++;
  if (frame_length != AI1811_FRAME_LENGTH)
  {
    return;
  }

  for (index = 0U; index < (AI1811_FRAME_LENGTH - 1U); index++)
  {
    checksum = (uint8_t)(checksum + frame[index]);
  }

  if (checksum == frame[AI1811_FRAME_LENGTH - 1U])
  {
    distance_mm = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8U);
    last_status.firmware = (uint16_t)frame[2] |
                           ((uint16_t)frame[3] << 8U);
    last_status.distance_mm = distance_mm;
    last_status.confidence = frame[6];
    last_status.timestamp_ms = HAL_GetTick();
    last_status.measurement_valid =
      (distance_mm >= AI1811_DISTANCE_MIN_MM &&
       distance_mm <= AI1811_DISTANCE_MAX_MM) ? 1U : 0U;
    if ((last_status.measurement_valid != 0U) &&
        (distance_mm <= AI1811_OBSTACLE_MAX_MM))
    {
      if (last_status.obstacle_streak < 255U)
      {
        last_status.obstacle_streak++;
      }
    }
    else
    {
      last_status.obstacle_streak = 0U;
    }
    last_status.sequence++;
    last_status.has_frame = 1U;
    AI1811_ResetParser();
  }
  else
  {
    /* Reprocess the failed frame tail so a header is not lost. */
    frame_length = 0U;
    for (index = 1U; index < AI1811_FRAME_LENGTH; index++)
    {
      AI1811_ProcessByte(frame[index]);
    }
  }
}

void AI1811_Init(void)
{
  rx_byte = 0U;
  frame_length = 0U;
  last_status.distance_mm = 0U;
  last_status.confidence = 0U;
  last_status.firmware = 0U;
  last_status.timestamp_ms = 0U;
  last_status.sequence = 0U;
  last_status.has_frame = 0U;
  last_status.measurement_valid = 0U;
  last_status.obstacle_streak = 0U;
  AI1811_StartReceive();
}

void AI1811_GetStatus(AI1811_Status_t *status)
{
  if (status == NULL)
  {
    return;
  }

  __disable_irq();
  *status = last_status;
  __enable_irq();
}

void AI1811_ResetObstacleStreak(void)
{
  __disable_irq();
  last_status.obstacle_streak = 0U;
  __enable_irq();
}

void AI1811_OnRxComplete(void)
{
  AI1811_ProcessByte(rx_byte);
  AI1811_StartReceive();
}

void AI1811_OnRxError(void)
{
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  AI1811_ResetParser();
  AI1811_StartReceive();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART2))
  {
    AI1811_OnRxComplete();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART2))
  {
    AI1811_OnRxError();
  }
}

#include "LaserRx.h"
#include "main.h"
#define RAW_FIFO_SIZE 512U
#define LASER_FRAME_LENGTH 8U
#define LASER_FRAME_HEADER 0xAAU
#define LASER_FRAME_TYPE 0x05U
extern UART_HandleTypeDef huart2;
static volatile uint8_t raw_fifo[RAW_FIFO_SIZE];
static volatile uint16_t raw_fifo_head, raw_fifo_tail;
static volatile uint32_t raw_fifo_dropped, uart_errors;
static uint32_t bad_frames;
static uint8_t usart2_rx_byte, laser_frame[LASER_FRAME_LENGTH], laser_frame_length;
static LaserRx_Handler on_frame;
static uint8_t RawFifo_Pop(uint8_t *byte);
static void USART2_StartReceive(void);
static void LaserParser_ProcessByte(uint8_t byte);
static void LaserParser_Resync(void);
void LaserRx_Init(void) { USART2_StartReceive(); }
void LaserRx_Poll(LaserRx_Handler handler)
{
  uint8_t byte;
  on_frame = handler;
  while (RawFifo_Pop(&byte)) { LaserParser_ProcessByte(byte); }
}
void LaserRx_Counters(uint32_t *bad, uint32_t *dropped, uint32_t *errors)
{ *bad = bad_frames; *dropped = raw_fifo_dropped; *errors = uart_errors; }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint16_t next_head;

  if (huart->Instance != USART2)
  {
    return;
  }

  next_head = (uint16_t)((raw_fifo_head + 1U) % RAW_FIFO_SIZE);
  if (next_head == raw_fifo_tail)
  {
    raw_fifo_dropped++;
  }
  else
  {
    raw_fifo[raw_fifo_head] = usart2_rx_byte;
    raw_fifo_head = next_head;
  }

  USART2_StartReceive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uart_errors++;
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    USART2_StartReceive();
  }
}

static uint8_t RawFifo_Pop(uint8_t *byte)
{
  if (raw_fifo_tail == raw_fifo_head)
  {
    return 0U;
  }

  *byte = raw_fifo[raw_fifo_tail];
  raw_fifo_tail = (uint16_t)((raw_fifo_tail + 1U) % RAW_FIFO_SIZE);
  return 1U;
}

static void LaserParser_ProcessByte(uint8_t byte)
{
  uint8_t index;
  uint8_t checksum = 0U;
  uint16_t firmware;
  uint16_t distance_mm;

  if (laser_frame_length == 0U)
  {
    if (byte == LASER_FRAME_HEADER)
    {
      laser_frame[0] = byte;
      laser_frame_length = 1U;
    }
    return;
  }

  if (laser_frame_length == 1U)
  {
    if (byte == LASER_FRAME_TYPE)
    {
      laser_frame[1] = byte;
      laser_frame_length = 2U;
    }
    else if (byte == LASER_FRAME_HEADER)
    {
      laser_frame[0] = byte;
    }
    else
    {
      laser_frame_length = 0U;
    }
    return;
  }

  laser_frame[laser_frame_length] = byte;
  laser_frame_length++;

  if (laser_frame_length != LASER_FRAME_LENGTH)
  {
    return;
  }

  for (index = 0U; index < (LASER_FRAME_LENGTH - 1U); index++)
  {
    checksum = (uint8_t)(checksum + laser_frame[index]);
  }

  if (checksum == laser_frame[LASER_FRAME_LENGTH - 1U])
  {
    firmware = (uint16_t)laser_frame[2] | ((uint16_t)laser_frame[3] << 8U);
    distance_mm = (uint16_t)laser_frame[4] | ((uint16_t)laser_frame[5] << 8U);
    if (on_frame != 0) { on_frame(distance_mm, laser_frame[6], firmware); }
    laser_frame_length = 0U;
  }
  else
  {
    bad_frames++;
    LaserParser_Resync();
  }
}

static void LaserParser_Resync(void)
{
  uint8_t index;
  uint8_t byte;

  /* Reprocess the failed frame tail so a header in data or SUM is not lost. */
  laser_frame_length = 0U;
  for (index = 1U; index < LASER_FRAME_LENGTH; index++)
  {
    byte = laser_frame[index];
    LaserParser_ProcessByte(byte);
  }
}

static void USART2_StartReceive(void)
{
  (void)HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1U);
}


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
static volatile uint8_t receiver_enabled;
static uint8_t RawFifo_Pop(uint8_t *byte);
static void USART2_StartReceive(void);
static void LaserParser_ProcessByte(uint8_t byte);
static void LaserParser_Resync(void);
void LaserRx_Init(void)
{
  /* CubeMX has configured USART2 already; startup is stationary. */
  receiver_enabled = 1U;
  USART2_StartReceive();
}

uint8_t LaserRx_IsEnabled(void) { return receiver_enabled; }

uint8_t LaserRx_SetEnabled(uint8_t enabled)
{
  HAL_StatusTypeDef status;
  if (enabled == 0U)
  {
    /* Merely ignoring frames or clearing RXNEIE leaves the peripheral and
     * its alternate-function path enabled. Remove that difference from the
     * validated no-USART2 driving baseline before any wheel is commanded. */
    receiver_enabled = 0U;
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    if (__HAL_RCC_USART2_IS_CLK_ENABLED())
    {
      (void)HAL_UART_AbortReceive(&huart2);
      status = HAL_UART_DeInit(&huart2); /* disables UE and MSP clock */
      if (status != HAL_OK) return 0U;
    }
    __HAL_RCC_USART2_FORCE_RESET();
    __HAL_RCC_USART2_RELEASE_RESET();
    HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
    raw_fifo_head = 0U;
    raw_fifo_tail = 0U;
    laser_frame_length = 0U;
    on_frame = 0;
    return __HAL_RCC_USART2_IS_CLK_DISABLED() ? 1U : 0U;
  }
  if (receiver_enabled != 0U) return 1U;
  raw_fifo_head = 0U;
  raw_fifo_tail = 0U;
  laser_frame_length = 0U;
  HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
  if (HAL_UART_Init(&huart2) != HAL_OK) return 0U;
  receiver_enabled = 1U;
  if (HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1U) != HAL_OK)
  {
    (void)LaserRx_SetEnabled(0U);
    return 0U;
  }
  return 1U;
}
void LaserRx_Poll(LaserRx_Handler handler)
{
  uint8_t byte;
  if (receiver_enabled == 0U) return;
  on_frame = handler;
  while (RawFifo_Pop(&byte)) { LaserParser_ProcessByte(byte); }
}
void LaserRx_Counters(uint32_t *bad, uint32_t *dropped, uint32_t *errors)
{ *bad = bad_frames; *dropped = raw_fifo_dropped; *errors = uart_errors; }
void LaserRx_UartRxComplete(UART_HandleTypeDef *huart)
{
  uint16_t next_head;

  if ((huart->Instance != USART2) || (receiver_enabled == 0U))
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

void LaserRx_UartError(UART_HandleTypeDef *huart)
{
  if ((huart->Instance == USART2) && (receiver_enabled != 0U))
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
  if (receiver_enabled != 0U)
    (void)HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1U);
}


/*
 * 12B AI-1811 raw UART observation test.
 * USART2 is receive-only: this project never sends a sensor command.
 */
#include "main.h"

#define RAW_FIFO_SIZE            256U
#define RAW_LOG_COLUMNS          16U
#define LASER_FRAME_LENGTH        8U
#define LASER_FRAME_HEADER        0xAAU
#define LASER_FRAME_TYPE          0x05U

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

static volatile uint8_t raw_fifo[RAW_FIFO_SIZE];
static volatile uint16_t raw_fifo_head;
static volatile uint16_t raw_fifo_tail;
static volatile uint32_t raw_fifo_dropped;
static uint8_t usart2_rx_byte;
static uint8_t raw_log_column;
static uint32_t last_status_ms;
static uint8_t laser_frame[LASER_FRAME_LENGTH];
static uint8_t laser_frame_length;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void Debug_SendString(const char *text);
static void Debug_PrintRawByte(uint8_t byte);
static void Debug_PrintMeasurement(uint16_t distance_mm, uint8_t confidence, uint16_t firmware);
static uint16_t Debug_AppendString(char *text, uint16_t index, const char *source);
static uint16_t Debug_AppendUint16(char *text, uint16_t index, uint16_t value);
static uint8_t RawFifo_Pop(uint8_t *byte);
static void LaserParser_ProcessByte(uint8_t byte);
static void LaserParser_Resync(void);
static void USART2_StartReceive(void);

int main(void)
{
  uint8_t byte;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  Debug_SendString("\r\n=== 12B AI-1811 Raw UART Test ===\r\n");
  Debug_SendString("USART2 RX: PA3, 115200 8N1\r\n");
  Debug_SendString("USART3 log: PB10/PB11, 115200 8N1\r\n");
  Debug_SendString("Sensor TX is disabled in software; raw bytes only.\r\n");
  Debug_SendString("RAW_LASER: ");

  USART2_StartReceive();
  last_status_ms = HAL_GetTick();

  while (1)
  {
    while (RawFifo_Pop(&byte) != 0U)
    {
      Debug_PrintRawByte(byte);
      LaserParser_ProcessByte(byte);
    }

    if ((uint32_t)(HAL_GetTick() - last_status_ms) >= 1000U)
    {
      last_status_ms = HAL_GetTick();
      if (raw_fifo_dropped != 0U)
      {
        Debug_SendString("\r\nRAW_LASER: FIFO_OVERFLOW\r\nRAW_LASER: ");
        raw_fifo_dropped = 0U;
        raw_log_column = 0U;
      }
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

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
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    USART2_StartReceive();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

static void Debug_SendString(const char *text)
{
  uint16_t length = 0U;

  while (text[length] != '\0')
  {
    length++;
  }

  (void)HAL_UART_Transmit(&huart3, (uint8_t *)text, length, HAL_MAX_DELAY);
}

static void Debug_PrintRawByte(uint8_t byte)
{
  static const char hex[] = "0123456789ABCDEF";
  char text[3];

  text[0] = hex[(byte >> 4U) & 0x0FU];
  text[1] = hex[byte & 0x0FU];
  text[2] = (raw_log_column == (RAW_LOG_COLUMNS - 1U)) ? '\n' : ' ';
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)text, sizeof(text), HAL_MAX_DELAY);

  raw_log_column++;
  if (raw_log_column >= RAW_LOG_COLUMNS)
  {
    raw_log_column = 0U;
    Debug_SendString("RAW_LASER: ");
  }
}

static void Debug_PrintMeasurement(uint16_t distance_mm, uint8_t confidence, uint16_t firmware)
{
  char text[72];
  uint16_t length = 0U;

  length = Debug_AppendString(text, length, "\r\nLASER DIST_MM=");
  length = Debug_AppendUint16(text, length, distance_mm);
  length = Debug_AppendString(text, length, " CONF=");
  length = Debug_AppendUint16(text, length, confidence);
  length = Debug_AppendString(text, length, " FW=");
  length = Debug_AppendUint16(text, length, firmware);
  length = Debug_AppendString(text, length, " CHECKSUM=OK\r\nRAW_LASER: ");
  text[length] = '\0';

  Debug_SendString(text);
  raw_log_column = 0U;
}

static uint16_t Debug_AppendString(char *text, uint16_t index, const char *source)
{
  while (*source != '\0')
  {
    text[index] = *source;
    index++;
    source++;
  }

  return index;
}

static uint16_t Debug_AppendUint16(char *text, uint16_t index, uint16_t value)
{
  char digits[5];
  uint8_t digit_count = 0U;

  do
  {
    digits[digit_count] = (char)('0' + (value % 10U));
    digit_count++;
    value = (uint16_t)(value / 10U);
  } while (value != 0U);

  while (digit_count != 0U)
  {
    digit_count--;
    text[index] = digits[digit_count];
    index++;
  }

  return index;
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
    Debug_PrintMeasurement(distance_mm, laser_frame[6], firmware);
    laser_frame_length = 0U;
  }
  else
  {
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

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif

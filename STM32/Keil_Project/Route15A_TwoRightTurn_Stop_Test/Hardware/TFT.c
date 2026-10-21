#include "TFT.h"
#include "main.h"

#define TFT_X_OFFSET 2U
#define TFT_Y_OFFSET 1U

#define TFT_CMD_SWRESET 0x01U
#define TFT_CMD_SLPOUT  0x11U
#define TFT_CMD_NORON   0x13U
#define TFT_CMD_DISPON  0x29U
#define TFT_CMD_CASET   0x2AU
#define TFT_CMD_RASET   0x2BU
#define TFT_CMD_RAMWR   0x2CU
#define TFT_CMD_MADCTL  0x36U
#define TFT_CMD_COLMOD  0x3AU
#define TFT_CMD_FRMCTR1 0xB1U
#define TFT_CMD_INVCTR  0xB4U
#define TFT_CMD_PWCTR1  0xC0U
#define TFT_CMD_PWCTR2  0xC1U
#define TFT_CMD_PWCTR3  0xC2U
#define TFT_CMD_PWCTR4  0xC3U
#define TFT_CMD_PWCTR5  0xC4U
#define TFT_CMD_VMCTR1  0xC5U
#define TFT_CMD_GMCTRP1 0xE0U
#define TFT_CMD_GMCTRN1 0xE1U

typedef struct
{
  char character;
  uint8_t columns[5];
} TFT_Glyph_t;

/*
 * The command sequence is the common ST7735/ST7735S-compatible 128x128
 * profile used by many 1.44-inch modules. The installed controller and its
 * row/column offsets are not confirmed by the current hardware information.
 */
static const TFT_Glyph_t tft_glyphs[] =
{
  {' ', {0x00U, 0x00U, 0x00U, 0x00U, 0x00U}},
  {'.', {0x00U, 0x00U, 0x60U, 0x60U, 0x00U}},
  {'0', {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU}},
  {'1', {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U}},
  {'2', {0x42U, 0x61U, 0x51U, 0x49U, 0x46U}},
  {'3', {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U}},
  {'4', {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U}},
  {'5', {0x27U, 0x45U, 0x45U, 0x45U, 0x39U}},
  {'6', {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U}},
  {'7', {0x01U, 0x71U, 0x09U, 0x05U, 0x03U}},
  {'8', {0x36U, 0x49U, 0x49U, 0x49U, 0x36U}},
  {'9', {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}},
  {'A', {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU}},
  {'B', {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U}},
  {'C', {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U}},
  {'D', {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU}},
  {'E', {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U}},
  {'F', {0x7FU, 0x09U, 0x09U, 0x01U, 0x01U}},
  {'G', {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU}},
  {'H', {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU}},
  {'I', {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U}},
  {'J', {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U}},
  {'K', {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U}},
  {'L', {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U}},
  {'N', {0x7FU, 0x02U, 0x0CU, 0x10U, 0x7FU}},
  {'O', {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU}},
  {'P', {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U}},
  {'R', {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U}},
  {'S', {0x46U, 0x49U, 0x49U, 0x49U, 0x31U}},
  {'T', {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U}},
  {'U', {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU}},
  {'V', {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU}},
  {'W', {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU}},
  {'_', {0x40U, 0x40U, 0x40U, 0x40U, 0x40U}}
};

static void TFT_PinHigh(GPIO_TypeDef *port, uint16_t pin)
{
  port->BSRR = pin;
}

static void TFT_PinLow(GPIO_TypeDef *port, uint16_t pin)
{
  port->BRR = pin;
}

static void TFT_WriteByte(uint8_t value)
{
  uint8_t bit;

  TFT_PinLow(TFT_SCLK_GPIO_Port, TFT_SCLK_Pin);
  for (bit = 0U; bit < 8U; bit++)
  {
    if ((value & 0x80U) != 0U)
    {
      TFT_PinHigh(TFT_MOSI_GPIO_Port, TFT_MOSI_Pin);
    }
    else
    {
      TFT_PinLow(TFT_MOSI_GPIO_Port, TFT_MOSI_Pin);
    }
    TFT_PinHigh(TFT_SCLK_GPIO_Port, TFT_SCLK_Pin);
    value <<= 1U;
    TFT_PinLow(TFT_SCLK_GPIO_Port, TFT_SCLK_Pin);
  }
}

static void TFT_WriteCommandData(uint8_t command,
                                 const uint8_t *data,
                                 uint16_t length)
{
  uint16_t index;

  TFT_PinLow(TFT_CS_GPIO_Port, TFT_CS_Pin);
  TFT_PinLow(TFT_RS_DC_GPIO_Port, TFT_RS_DC_Pin);
  TFT_WriteByte(command);
  if ((data != 0) && (length != 0U))
  {
    TFT_PinHigh(TFT_RS_DC_GPIO_Port, TFT_RS_DC_Pin);
    for (index = 0U; index < length; index++)
    {
      TFT_WriteByte(data[index]);
    }
  }
  TFT_PinHigh(TFT_CS_GPIO_Port, TFT_CS_Pin);
}

static void TFT_WriteCommand(uint8_t command)
{
  TFT_WriteCommandData(command, 0, 0U);
}

static void TFT_SetAddressWindow(uint8_t x0,
                                 uint8_t y0,
                                 uint8_t x1,
                                 uint8_t y1)
{
  uint8_t data[4];
  uint16_t x_start = (uint16_t)x0 + TFT_X_OFFSET;
  uint16_t x_end = (uint16_t)x1 + TFT_X_OFFSET;
  uint16_t y_start = (uint16_t)y0 + TFT_Y_OFFSET;
  uint16_t y_end = (uint16_t)y1 + TFT_Y_OFFSET;

  data[0] = (uint8_t)(x_start >> 8U);
  data[1] = (uint8_t)x_start;
  data[2] = (uint8_t)(x_end >> 8U);
  data[3] = (uint8_t)x_end;
  TFT_WriteCommandData(TFT_CMD_CASET, data, 4U);

  data[0] = (uint8_t)(y_start >> 8U);
  data[1] = (uint8_t)y_start;
  data[2] = (uint8_t)(y_end >> 8U);
  data[3] = (uint8_t)y_end;
  TFT_WriteCommandData(TFT_CMD_RASET, data, 4U);
  TFT_WriteCommand(TFT_CMD_RAMWR);
}

static const uint8_t *TFT_FindGlyph(char character)
{
  uint32_t index;

  for (index = 0U; index < (sizeof(tft_glyphs) / sizeof(tft_glyphs[0]));
       index++)
  {
    if (tft_glyphs[index].character == character)
    {
      return tft_glyphs[index].columns;
    }
  }
  return tft_glyphs[0].columns;
}

void TFT_Init(void)
{
  static const uint8_t colmod[] = {0x05U};
  static const uint8_t madctl[] = {0xC8U};
  static const uint8_t frmctr1[] = {0x01U, 0x2CU, 0x2DU};
  static const uint8_t invctr[] = {0x07U};
  static const uint8_t pwctr1[] = {0xA2U, 0x02U, 0x84U};
  static const uint8_t pwctr2[] = {0xC5U};
  static const uint8_t pwctr3[] = {0x0AU, 0x00U};
  static const uint8_t pwctr4[] = {0x8AU, 0x2AU};
  static const uint8_t pwctr5[] = {0x8AU, 0xEEU};
  static const uint8_t vmctr1[] = {0x0EU};
  static const uint8_t gmctrp1[] = {
      0x02U, 0x1CU, 0x07U, 0x12U, 0x37U, 0x32U, 0x29U, 0x2DU,
      0x29U, 0x25U, 0x2BU, 0x39U, 0x00U, 0x01U, 0x03U, 0x10U};
  static const uint8_t gmctrn1[] = {
      0x03U, 0x1DU, 0x07U, 0x06U, 0x2EU, 0x2CU, 0x29U, 0x2DU,
      0x2EU, 0x2EU, 0x37U, 0x3FU, 0x00U, 0x00U, 0x02U, 0x10U};

  TFT_PinHigh(TFT_CS_GPIO_Port, TFT_CS_Pin);
  TFT_PinLow(TFT_SCLK_GPIO_Port, TFT_SCLK_Pin);
  TFT_PinHigh(TFT_RS_DC_GPIO_Port, TFT_RS_DC_Pin);
  TFT_PinHigh(TFT_BLK_GPIO_Port, TFT_BLK_Pin);
  TFT_PinHigh(TFT_RST_GPIO_Port, TFT_RST_Pin);
  HAL_Delay(5U);
  TFT_PinLow(TFT_RST_GPIO_Port, TFT_RST_Pin);
  HAL_Delay(20U);
  TFT_PinHigh(TFT_RST_GPIO_Port, TFT_RST_Pin);
  HAL_Delay(120U);

  TFT_WriteCommand(TFT_CMD_SWRESET);
  HAL_Delay(150U);
  TFT_WriteCommand(TFT_CMD_SLPOUT);
  HAL_Delay(120U);
  TFT_WriteCommandData(TFT_CMD_COLMOD, colmod, sizeof(colmod));
  TFT_WriteCommandData(TFT_CMD_MADCTL, madctl, sizeof(madctl));
  TFT_WriteCommandData(TFT_CMD_FRMCTR1, frmctr1, sizeof(frmctr1));
  TFT_WriteCommandData(TFT_CMD_INVCTR, invctr, sizeof(invctr));
  TFT_WriteCommandData(TFT_CMD_PWCTR1, pwctr1, sizeof(pwctr1));
  TFT_WriteCommandData(TFT_CMD_PWCTR2, pwctr2, sizeof(pwctr2));
  TFT_WriteCommandData(TFT_CMD_PWCTR3, pwctr3, sizeof(pwctr3));
  TFT_WriteCommandData(TFT_CMD_PWCTR4, pwctr4, sizeof(pwctr4));
  TFT_WriteCommandData(TFT_CMD_PWCTR5, pwctr5, sizeof(pwctr5));
  TFT_WriteCommandData(TFT_CMD_VMCTR1, vmctr1, sizeof(vmctr1));
  TFT_WriteCommandData(TFT_CMD_GMCTRP1, gmctrp1, sizeof(gmctrp1));
  TFT_WriteCommandData(TFT_CMD_GMCTRN1, gmctrn1, sizeof(gmctrn1));
  TFT_WriteCommand(TFT_CMD_NORON);
  HAL_Delay(10U);
  TFT_WriteCommand(TFT_CMD_DISPON);
  HAL_Delay(100U);
  TFT_Clear(TFT_COLOR_BLACK);
}

void TFT_Clear(uint16_t color)
{
  uint8_t x;
  uint8_t y;

  TFT_SetAddressWindow(0U, 0U, TFT_WIDTH - 1U, TFT_HEIGHT - 1U);
  TFT_PinLow(TFT_CS_GPIO_Port, TFT_CS_Pin);
  TFT_PinHigh(TFT_RS_DC_GPIO_Port, TFT_RS_DC_Pin);
  for (y = 0U; y < TFT_HEIGHT; y++)
  {
    for (x = 0U; x < TFT_WIDTH; x++)
    {
      TFT_WriteByte((uint8_t)(color >> 8U));
      TFT_WriteByte((uint8_t)color);
    }
  }
  TFT_PinHigh(TFT_CS_GPIO_Port, TFT_CS_Pin);
}

void TFT_DrawChar(uint8_t x,
                  uint8_t y,
                  char character,
                  uint16_t foreground,
                  uint16_t background)
{
  const uint8_t *glyph;
  uint8_t column;
  uint8_t row;

  if ((x > (TFT_WIDTH - 6U)) || (y > (TFT_HEIGHT - 8U)))
  {
    return;
  }

  glyph = TFT_FindGlyph(character);
  TFT_SetAddressWindow(x, y, (uint8_t)(x + 5U), (uint8_t)(y + 7U));
  TFT_PinLow(TFT_CS_GPIO_Port, TFT_CS_Pin);
  TFT_PinHigh(TFT_RS_DC_GPIO_Port, TFT_RS_DC_Pin);
  for (row = 0U; row < 8U; row++)
  {
    for (column = 0U; column < 6U; column++)
    {
      uint16_t color = background;
      if ((column < 5U) && (row < 7U) &&
          ((glyph[column] & (uint8_t)(1U << row)) != 0U))
      {
        color = foreground;
      }
      TFT_WriteByte((uint8_t)(color >> 8U));
      TFT_WriteByte((uint8_t)color);
    }
  }
  TFT_PinHigh(TFT_CS_GPIO_Port, TFT_CS_Pin);
}

void TFT_DrawString(uint8_t x,
                    uint8_t y,
                    const char *text,
                    uint16_t foreground,
                    uint16_t background)
{
  uint8_t start_x = x;

  if (text == 0)
  {
    return;
  }
  while (*text != '\0')
  {
    if (*text == '\n')
    {
      x = start_x;
      y = (uint8_t)(y + 8U);
    }
    else
    {
      if (x > (TFT_WIDTH - 6U))
      {
        break;
      }
      TFT_DrawChar(x, y, *text, foreground, background);
      x = (uint8_t)(x + 6U);
    }
    text++;
  }
}

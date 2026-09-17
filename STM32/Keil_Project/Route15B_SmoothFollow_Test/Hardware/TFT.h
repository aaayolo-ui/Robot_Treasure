#ifndef TFT_H
#define TFT_H

#include <stdint.h>

#define TFT_WIDTH  128U
#define TFT_HEIGHT 128U

#define TFT_COLOR_BLACK 0x0000U
#define TFT_COLOR_WHITE 0xFFFFU
#define TFT_COLOR_GREEN 0x07E0U
#define TFT_COLOR_YELLOW 0xFFE0U

void TFT_Init(void);
void TFT_Clear(uint16_t color);
void TFT_DrawChar(uint8_t x,
                  uint8_t y,
                  char character,
                  uint16_t foreground,
                  uint16_t background);
void TFT_DrawString(uint8_t x,
                    uint8_t y,
                    const char *text,
                    uint16_t foreground,
                    uint16_t background);

#endif /* TFT_H */

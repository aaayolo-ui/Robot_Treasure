#ifndef AI1811_MUX_H
#define AI1811_MUX_H
#include <stdint.h>

#define AI1811_MUX_CHANNELS 4U
#define AI1811_MUX_FRESH_MS 250U
typedef struct {
  uint8_t channel, prefix, mode, count, high, has_high;
  uint8_t frame[8];
  uint32_t first_tick, last_tick;
  uint32_t bad[AI1811_MUX_CHANNELS], valid[AI1811_MUX_CHANNELS];
} AI1811Mux;
typedef void (*AI1811Mux_Handler)(void *context, uint8_t channel,
    uint16_t distance, uint8_t confidence, uint16_t firmware, uint32_t tick);
void AI1811Mux_Init(AI1811Mux *mux);
void AI1811Mux_ResetStream(AI1811Mux *mux);
void AI1811Mux_Byte(AI1811Mux *mux, uint8_t byte, uint32_t tick,
    AI1811Mux_Handler handler, void *context);
#endif

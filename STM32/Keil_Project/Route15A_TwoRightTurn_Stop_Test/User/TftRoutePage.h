#ifndef TFT_ROUTE_PAGE_H
#define TFT_ROUTE_PAGE_H

#include <stdint.h>

void TftRoutePage_Init(void);
void TftRoutePage_Task(const char *state_text, uint8_t junction_count);

#endif /* TFT_ROUTE_PAGE_H */

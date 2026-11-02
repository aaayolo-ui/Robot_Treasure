#ifndef TFT_PID_PAGE_H
#define TFT_PID_PAGE_H

#include <stdint.h>
#include "LineFollowPID.h"

void TftPidPage_Init(void);
void TftPidPage_Task(const LineFollowPID_Parameters_t *parameters,
                    LineFollowPID_ParameterId_t selected_parameter,
                    uint8_t running,
                    const char *status_text);

#endif /* TFT_PID_PAGE_H */

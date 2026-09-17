#ifndef RESET_REASON_H
#define RESET_REASON_H

typedef enum
{
  RESET_REASON_POWER_ON = 0,
  RESET_REASON_PIN,
  RESET_REASON_SOFTWARE,
  RESET_REASON_WATCHDOG,
  RESET_REASON_LOW_POWER,
  RESET_REASON_UNKNOWN
} ResetReason_t;

ResetReason_t ResetReason_ReadAndClear(void);
const char *ResetReason_ToString(ResetReason_t reason);

#endif

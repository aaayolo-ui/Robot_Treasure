#include "ResetReason.h"
#include "main.h"

ResetReason_t ResetReason_ReadAndClear(void)
{
  ResetReason_t reason = RESET_REASON_UNKNOWN;

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
  {
    reason = RESET_REASON_POWER_ON;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
  {
    reason = RESET_REASON_PIN;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
  {
    reason = RESET_REASON_SOFTWARE;
  }
  else if ((__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) ||
           (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET))
  {
    reason = RESET_REASON_WATCHDOG;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET)
  {
    reason = RESET_REASON_LOW_POWER;
  }

  __HAL_RCC_CLEAR_RESET_FLAGS();
  return reason;
}

const char *ResetReason_ToString(ResetReason_t reason)
{
  static const char *const names[] =
  {
    "POWER_ON_RESET",
    "PIN_RESET",
    "SOFTWARE_RESET",
    "WATCHDOG_RESET",
    "LOW_POWER_RESET",
    "UNKNOWN_RESET"
  };

  return ((unsigned int)reason < (sizeof(names) / sizeof(names[0]))) ?
         names[reason] : "UNKNOWN_RESET";
}

#ifndef KEY_H
#define KEY_H

#include <stdint.h>

typedef enum
{
  KEY_EVENT_NONE = 0,
  KEY_EVENT_KEY1_SHORT,
  KEY_EVENT_KEY2_SHORT,
  KEY_EVENT_WK_UP_SHORT,
  KEY_EVENT_KEY2_LONG,
  KEY_EVENT_WK_UP_LONG
} KeyEvent_t;

/* Compatibility names for the older ground-test application source. */
#define KEY_EVENT_KEY1  KEY_EVENT_KEY1_SHORT
#define KEY_EVENT_KEY2  KEY_EVENT_KEY2_SHORT
#define KEY_EVENT_WK_UP KEY_EVENT_WK_UP_SHORT

typedef enum
{
  KEY_ID_KEY1 = 0,
  KEY_ID_KEY2,
  KEY_ID_WK_UP,
  KEY_ID_COUNT
} KeyId_t;

void Key_Init(void);
KeyEvent_t Key_GetEvent(void);
uint8_t Key_IsPressed(KeyId_t key);

#endif /* KEY_H */

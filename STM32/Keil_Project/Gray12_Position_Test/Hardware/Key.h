#ifndef KEY_H
#define KEY_H

typedef enum
{
  KEY_EVENT_NONE = 0,
  KEY_EVENT_KEY1,
  KEY_EVENT_KEY2,
  KEY_EVENT_WK_UP
} KeyEvent_t;

void Key_Init(void);
KeyEvent_t Key_GetEvent(void);

#endif /* KEY_H */

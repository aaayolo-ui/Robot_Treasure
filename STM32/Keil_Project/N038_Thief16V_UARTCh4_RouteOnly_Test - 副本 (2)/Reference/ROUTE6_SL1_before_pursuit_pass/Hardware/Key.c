#include "Key.h"
#include "main.h"

#define KEY_DEBOUNCE_DELAY_MS 20U
#define KEY_LONG_PRESS_DELAY_MS 800U

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  GPIO_PinState pressed_state;
  GPIO_PinState stable_state;
  GPIO_PinState raw_state;
  KeyEvent_t short_event;
  KeyEvent_t long_event;
  uint32_t raw_change_tick;
  uint32_t press_start_tick;
  uint8_t long_reported;
} Key_t;

static Key_t keys[] =
{
  {KEY1_GPIO_Port, KEY1_Pin, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_SET,
   KEY_EVENT_KEY1_SHORT, KEY_EVENT_NONE, 0U, 0U, 0U},
  {KEY2_GPIO_Port, KEY2_Pin, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_SET,
   KEY_EVENT_KEY2_SHORT, KEY_EVENT_KEY2_LONG, 0U, 0U, 0U},
  {WK_UP_GPIO_Port, WK_UP_Pin, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET,
   KEY_EVENT_WK_UP_SHORT, KEY_EVENT_WK_UP_LONG, 0U, 0U, 0U}
};

void Key_Init(void)
{
  uint32_t index;
  uint32_t now = HAL_GetTick();

  for (index = 0U; index < (sizeof(keys) / sizeof(keys[0])); index++)
  {
    keys[index].stable_state = HAL_GPIO_ReadPin(keys[index].port, keys[index].pin);
    keys[index].raw_state = keys[index].stable_state;
    keys[index].raw_change_tick = now;
    keys[index].press_start_tick = 0U;
    keys[index].long_reported = 0U;
  }
}

KeyEvent_t Key_GetEvent(void)
{
  uint32_t index;
  uint32_t now = HAL_GetTick();
  KeyEvent_t event = KEY_EVENT_NONE;

  for (index = 0U; index < (sizeof(keys) / sizeof(keys[0])); index++)
  {
    GPIO_PinState current_state = HAL_GPIO_ReadPin(keys[index].port, keys[index].pin);

    if (current_state != keys[index].raw_state)
    {
      keys[index].raw_state = current_state;
      keys[index].raw_change_tick = now;
    }

    if ((keys[index].raw_state != keys[index].stable_state) &&
        ((uint32_t)(now - keys[index].raw_change_tick) >=
         KEY_DEBOUNCE_DELAY_MS))
    {
      keys[index].stable_state = keys[index].raw_state;
      if (keys[index].stable_state == keys[index].pressed_state)
      {
        keys[index].press_start_tick = now;
        keys[index].long_reported = 0U;
      }
      else
      {
        if ((keys[index].long_reported == 0U) &&
            (event == KEY_EVENT_NONE))
        {
          event = keys[index].short_event;
        }
        keys[index].long_reported = 0U;
      }
    }

    if ((keys[index].stable_state == keys[index].pressed_state) &&
        (keys[index].long_event != KEY_EVENT_NONE) &&
        (keys[index].long_reported == 0U) &&
        ((uint32_t)(now - keys[index].press_start_tick) >=
         KEY_LONG_PRESS_DELAY_MS) &&
        (event == KEY_EVENT_NONE))
    {
      keys[index].long_reported = 1U;
      event = keys[index].long_event;
    }
  }

  return event;
}

uint8_t Key_IsPressed(KeyId_t key)
{
  if ((uint32_t)key >= (uint32_t)KEY_ID_COUNT)
  {
    return 0U;
  }
  return (keys[key].stable_state == keys[key].pressed_state) ? 1U : 0U;
}

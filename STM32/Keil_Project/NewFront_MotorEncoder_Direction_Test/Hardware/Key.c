#include "Key.h"
#include "main.h"

#define KEY_DEBOUNCE_DELAY_MS 20U

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  GPIO_PinState pressed_state;
  GPIO_PinState stable_state;
  KeyEvent_t event;
} Key_t;

static Key_t keys[] =
{
  {KEY1_GPIO_Port, KEY1_Pin, GPIO_PIN_RESET, GPIO_PIN_SET, KEY_EVENT_KEY1},
  {KEY2_GPIO_Port, KEY2_Pin, GPIO_PIN_RESET, GPIO_PIN_SET, KEY_EVENT_KEY2},
  {WK_UP_GPIO_Port, WK_UP_Pin, GPIO_PIN_SET, GPIO_PIN_RESET, KEY_EVENT_WK_UP}
};

void Key_Init(void)
{
  uint32_t index;

  for (index = 0U; index < (sizeof(keys) / sizeof(keys[0])); index++)
  {
    keys[index].stable_state = HAL_GPIO_ReadPin(keys[index].port, keys[index].pin);
  }
}

KeyEvent_t Key_GetEvent(void)
{
  uint32_t index;

  for (index = 0U; index < (sizeof(keys) / sizeof(keys[0])); index++)
  {
    GPIO_PinState current_state = HAL_GPIO_ReadPin(keys[index].port, keys[index].pin);

    if (current_state != keys[index].stable_state)
    {
      HAL_Delay(KEY_DEBOUNCE_DELAY_MS);
      current_state = HAL_GPIO_ReadPin(keys[index].port, keys[index].pin);

      if (current_state != keys[index].stable_state)
      {
        keys[index].stable_state = current_state;
        if (current_state == keys[index].pressed_state)
        {
          return keys[index].event;
        }
      }
    }
  }

  return KEY_EVENT_NONE;
}

uint8_t Key_IsPressed(KeyEvent_t key)
{
  uint32_t index;

  for (index = 0U; index < (sizeof(keys) / sizeof(keys[0])); index++)
  {
    if (keys[index].event == key)
    {
      return (HAL_GPIO_ReadPin(keys[index].port, keys[index].pin) ==
              keys[index].pressed_state) ? 1U : 0U;
    }
  }

  return 0U;
}

uint8_t Key_IsAnyPressed(void)
{
  return ((Key_IsPressed(KEY_EVENT_KEY1) != 0U) ||
          (Key_IsPressed(KEY_EVENT_KEY2) != 0U) ||
          (Key_IsPressed(KEY_EVENT_WK_UP) != 0U)) ? 1U : 0U;
}

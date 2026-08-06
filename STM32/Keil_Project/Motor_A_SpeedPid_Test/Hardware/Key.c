#include "Key.h"
#include "main.h"

#define KEY_DEBOUNCE_DELAY_MS 20U

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  GPIO_PinState pressedState;
  GPIO_PinState stableState;
  KeyEvent_t event;
} Key_t;

static Key_t keys[] =
{
  {WK_UP_GPIO_Port, WK_UP_Pin, GPIO_PIN_SET, GPIO_PIN_RESET, KEY_EVENT_WK_UP},
  {KEY1_GPIO_Port, KEY1_Pin, GPIO_PIN_RESET, GPIO_PIN_SET, KEY_EVENT_KEY1},
  {KEY2_GPIO_Port, KEY2_Pin, GPIO_PIN_RESET, GPIO_PIN_SET, KEY_EVENT_KEY2}
};

void Key_Init(void)
{
  uint32_t index;

  for (index = 0U; index < (sizeof(keys) / sizeof(keys[0])); index++)
  {
    keys[index].stableState = HAL_GPIO_ReadPin(keys[index].port, keys[index].pin);
  }
}

KeyEvent_t Key_GetEvent(void)
{
  uint32_t index;

  for (index = 0U; index < (sizeof(keys) / sizeof(keys[0])); index++)
  {
    GPIO_PinState currentState = HAL_GPIO_ReadPin(keys[index].port, keys[index].pin);

    if (currentState != keys[index].stableState)
    {
      HAL_Delay(KEY_DEBOUNCE_DELAY_MS);
      currentState = HAL_GPIO_ReadPin(keys[index].port, keys[index].pin);

      if (currentState != keys[index].stableState)
      {
        keys[index].stableState = currentState;
        if (currentState == keys[index].pressedState)
        {
          return keys[index].event;
        }
      }
    }
  }

  return KEY_EVENT_NONE;
}

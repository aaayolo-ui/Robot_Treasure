#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef struct { void *Instance; } UART_HandleTypeDef;
typedef enum { HAL_OK=0, HAL_ERROR=1 } HAL_StatusTypeDef;
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *p);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *p);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *p);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *p,uint8_t *b,uint16_t n);
uint32_t HAL_GetTick(void);
#define __HAL_UART_CLEAR_OREFLAG(p) ((void)(p))
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(p) ((void)(p))
#define __HAL_RCC_USART1_FORCE_RESET() ((void)0)
#define __HAL_RCC_USART1_RELEASE_RESET() ((void)0)
#define __HAL_RCC_USART2_FORCE_RESET() ((void)0)
#define __HAL_RCC_USART2_RELEASE_RESET() ((void)0)
#define __HAL_RCC_UART4_FORCE_RESET() ((void)0)
#define __HAL_RCC_UART4_RELEASE_RESET() ((void)0)
#endif

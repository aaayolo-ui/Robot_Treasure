#ifndef __LED_H
#define __LED_H
#include "sys.h"


#define LED  PAout(8)

/* 定义SPI连接的GPIO端口, 用户只需要修改下面代码即可任意改变相应的引脚*/
#define LED_GPIO_PORT	 GPIOA			            /* GPIO端口 */
#define LED_RCC 	     RCC_APB2Periph_GPIOA		/* GPIO端口时钟 */
#define LED_PIN		     GPIO_Pin_8	

void LED_GPIO_Init(void);

#endif


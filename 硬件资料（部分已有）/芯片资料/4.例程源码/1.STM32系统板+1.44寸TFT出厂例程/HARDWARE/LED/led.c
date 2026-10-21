#include "led.h"


void LED_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(LED_RCC, ENABLE);	 //使能端口时钟

	GPIO_InitStructure.GPIO_Pin = LED_PIN;	 //推挽输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz
	GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);	 //初始化GPIO
}


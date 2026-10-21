/*
1.项目名称：绿深旗舰店-STM32综合测试板SPI接口1.8吋LCD测试程序
2.显示模块：1.8吋LCD屏
3.配套APP：无
4.配套上位机：无
5.项目组成：1.8吋LCD、绿深STM32核心板
6.项目功能：1.8吋LCD显示中英文、数字、图片。
7.主要原理：具体参考1.8 LCD数据手册
8.购买地址：https://lssz.tmall.com 或淘宝上搜索“绿深旗舰店”
*/
#include "stm32f10x.h"
#include "delay.h"
#include "QDTFT_demo.h"
#include "Lcd_Driver.h"
#include "GUI.h"


int main(void)
{

	SystemInit();	//System init.
	delay_init(72);//Delay init.
	while(1)
	{
		QDTFT_Test_Demo();	//See the test details in QDTFT_Demo.c
	}

}


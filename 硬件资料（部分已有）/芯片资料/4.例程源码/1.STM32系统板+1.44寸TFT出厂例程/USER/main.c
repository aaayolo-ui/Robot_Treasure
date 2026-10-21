/*
1.项目名称：绿深旗舰店-STM32综合测试板SPI接口1.44吋LCD测试程序
2.显示模块：1.44吋LCD屏
3.配套APP：无
4.配套上位机：无
5.项目组成：1.44吋LCD、绿深STM32综合测试板
6.项目功能：1.44吋LCD显示中英文、数字、图片。具体操作请看演示视频
7.主要原理：具体参考1.44吋LCD数据手册
8.购买地址：https://lssz.tmall.com 或淘宝上搜索“绿深旗舰店”
9.演示视频：宝贝详情页面或购买后像客服索要！
10.版权声明：绿深旗舰店所有程序都申请软件著作权。均与本店产品配套出售，请不要传播，以免追究其法律责任！
*/
#include "sys.h"
#include "delay.h"
#include "lcd.h"
#include "stdio.h"
#include "Picture.h"
#include "led.h"

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	delay_init();
	LED_GPIO_Init();
	LCD_Init();
	LCD_BLK = 0;
	LCD_Clear(WHITE);
	LCD_Showimage(14, 0, 100, 50, gImage_11);
	LCD_DrawFont_GBK16(24, 60, BLUE, WHITE, "绿深旗舰店");
	LCD_DrawFont_GBK16(28, 80, RED, WHITE, "1.44 LCD");
	LCD_DrawFont_GBK16(8, 100, BLACK, WHITE, "lssz.tmall.com");
	LED=0;
	
	while(1)
	{
		delay_ms(500);
		LED=!LED;
	}
}



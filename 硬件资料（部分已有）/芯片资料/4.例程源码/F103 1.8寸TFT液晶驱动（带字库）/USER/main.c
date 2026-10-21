//////////////////////////////////////////////////////////////////////////////////	 
//绿深旗舰店 1.8寸带字库TFT测试程序   
//店铺地址：http://tmall.lvsn.com
//测试平台：STM32F103RCT6
//              ----------------------------------------------------------------
//              GND   电源地
//              VCC   电源正极（3.3-5V）
//              SCL   PB9（SCLK）
//              SDA   PB8（MOSI）
//              CS    PB7
//              SDO   PB6（MISO）
//              DC    PB5
//              BLK   PB4
//              ----------------------------------------------------------------
//******************************************************************************/
#include "delay.h"
#include "sys.h"
#include "led.h"
#include "lcd_init.h"
#include "lcd.h"
#include "pic.h"
int main(void)
{
	float t=0;
	delay_init();
	LED_Init();//LED初始化
	LCD_Init();//LCD初始化
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
	LED=0;
	while(1)
	{
		Display_GB2312_String(4,0,3, "绿深旗舰店",BLUE,WHITE);
		LCD_ShowString(24,30,"LCD_W:",RED,WHITE,16,0);
		LCD_ShowIntNum(72,30,LCD_W,3,RED,WHITE,16);
		LCD_ShowString(24,50,"LCD_H:",RED,WHITE,16,0);
		LCD_ShowIntNum(72,50,LCD_H,3,RED,WHITE,16);
		LCD_ShowFloatNum1(20,80,t,4,RED,WHITE,16);
		t+=0.11;
		LCD_ShowPicture(65,80,40,40,gImage_1);
		delay_ms(1000);
		LED=~LED;
		delay_ms(1000);
		LED=~LED;
		LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
		Display_Asc_String(0,15,1, "ASCII_5x7",RED,WHITE);    //ASC 5X7点阵
		Display_Asc_String(0,25,2, "ASCII_7x8",RED,WHITE);    //ASC 7X8点阵
		Display_Asc_String(0,35,3, "ASCII_6x12",RED,WHITE);	  //ASC 6X12点阵
		Display_Asc_String(0,50,4, "ASCII_8x16",RED,WHITE);	  //ASC 8X16点阵 
		Display_GB2312_String(0,70,1, "绿深旗舰店１２ｘ１２",RED,WHITE); //12x12汉字
		Display_GB2312_String(0,90,2, "绿深旗舰店",RED,WHITE); //16x16汉字
		Display_Asc_String(0,110,4, "16 x 16",RED,WHITE);	  //ASC 8X16点阵
		delay_ms(1000);
		LED=~LED;
		delay_ms(1000);
		LED=~LED;
		LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
	
		Display_TimesNewRoman_String(0,15,1, "ASCII_8x12",RED,WHITE);   //ASC 8x12点阵(TimesNewRoman类型)
		Display_TimesNewRoman_String(0,30,2, "ASCII",RED,WHITE);        //ASC 12x16点阵(TimesNewRoman类型)
		Display_TimesNewRoman_String(0,50,3, "ASCII",RED,WHITE);	      //ASC 16x24点阵(TimesNewRoman类型)
		Display_Arial_String(0,90,1, "ASCII_8x1",RED,WHITE);   //ASC 8x12点阵(Arial类型)
		Display_Arial_String(0,110,2, "ASCII",RED,WHITE);      //ASC 12x16点阵(Arial类型)
		Display_Arial_String(0,130,3, "ASCII",RED,WHITE);	 //ASC 16x24点阵(Arial类型)
		delay_ms(1000);
		LED=~LED;
		delay_ms(1000);
		LED=~LED;
		LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
		Display_GB2312_String(0,40,2, "雷填填兮雨冥冥，",BLUE,WHITE);
		LCD_ShowChinese(0,60,"猨",BLUE,WHITE,16,0);
		Display_GB2312_String(16,60,2, "啾啾兮",BLUE,WHITE);
		LCD_ShowChinese(64,60,"狖",BLUE,WHITE,16,0);
		Display_GB2312_String(80,60,2, "夜鸣。",BLUE,WHITE);
		Display_GB2312_String(0,80,2, "风飒飒兮木萧萧，",BLUE,WHITE);
		Display_GB2312_String(0,100,2, "思公子兮徒离忧。",BLUE,WHITE);
		delay_ms(1000);
		LED=~LED;
		delay_ms(1000);
		LED=~LED;
		LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
	}
}


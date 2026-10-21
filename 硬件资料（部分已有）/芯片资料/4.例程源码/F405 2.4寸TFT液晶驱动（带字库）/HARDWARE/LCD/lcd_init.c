#include "lcd_init.h"
#include "delay.h"

void LCD_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA~E,PORTG时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化

}


/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
void LCD_Writ_Bus(u8 dat)
{
	u8 i;
	LCD_CS_Clr();
	for(i = 0; i < 8; i++)
	{
		LCD_SCLK_Clr();
		if(dat & 0x80)
		{
			LCD_MOSI_Set();
		}
		else
		{
			LCD_MOSI_Clr();
		}
		LCD_SCLK_Set();
		dat <<= 1;
	}
	LCD_CS_Set();
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA8(u8 dat)
{
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA(u16 dat)
{
	LCD_Writ_Bus(dat >> 8);
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG(u8 dat)
{
	LCD_DC_Clr();//写命令
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//写数据
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2)
{
	if(USE_HORIZONTAL == 0)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL == 1)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL == 2)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y2);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
	else
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2 );
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1 );
		LCD_WR_DATA(y2 );
		LCD_WR_REG(0x2c);//储存器写
	}
}

void LCD_Init(void)
{
	LCD_GPIO_Init();//初始化GPIO

//板子自带复位电路  无需手动复位
//	LCD_RES_Clr();//复位
//	delay_ms(100);
//	LCD_RES_Set();
//	delay_ms(100);

	LCD_BLK_Set();//打开背光
	delay_ms(100);

	//************* Start Initial Sequence **********//
	LCD_WR_REG(0x11); //Sleep out
	delay_ms(120);              //Delay 120ms
	LCD_WR_REG(0xCF);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0xC1);
    LCD_WR_DATA8(0X30);
    LCD_WR_REG(0xED);
    LCD_WR_DATA8(0x64);
    LCD_WR_DATA8(0x03);
    LCD_WR_DATA8(0X12);
    LCD_WR_DATA8(0X81);
    LCD_WR_REG(0xE8);
    LCD_WR_DATA8(0x85);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x79);
    LCD_WR_REG(0xCB);
    LCD_WR_DATA8(0x39);
    LCD_WR_DATA8(0x2C);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x34);
    LCD_WR_DATA8(0x02);
    LCD_WR_REG(0xF7);
    LCD_WR_DATA8(0x20);
    LCD_WR_REG(0xEA);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_REG(0xC0); //Power control
    LCD_WR_DATA8(0x1D); //VRH[5:0]
    LCD_WR_REG(0xC1); //Power control
    LCD_WR_DATA8(0x12); //SAP[2:0];BT[3:0]
    LCD_WR_REG(0xC5); //VCM control
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x3F);
    LCD_WR_REG(0xC7); //VCM control
    LCD_WR_DATA8(0x92);
    LCD_WR_REG(0x3A); // Memory Access Control
    LCD_WR_DATA8(0x55);
    LCD_WR_REG(0x36); // Memory Access Control
    if (USE_HORIZONTAL == 0)LCD_WR_DATA8(0x08);
    else if (USE_HORIZONTAL == 1)LCD_WR_DATA8(0xC8);
    else if (USE_HORIZONTAL == 2)LCD_WR_DATA8(0x78);
    else LCD_WR_DATA8(0xA8);
    LCD_WR_REG(0xB1);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x12);
    LCD_WR_REG(0xB6); // Display Function Control
    LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0xA2);

    LCD_WR_REG(0x44);
    LCD_WR_DATA8(0x02);

    LCD_WR_REG(0xF2); // 3Gamma Function Disable
    LCD_WR_DATA8(0x00);
    LCD_WR_REG(0x26); //Gamma curve selected
    LCD_WR_DATA8(0x01);
    LCD_WR_REG(0xE0); //Set Gamma
    LCD_WR_DATA8(0x0F);
    LCD_WR_DATA8(0x22);
    LCD_WR_DATA8(0x1C);
    LCD_WR_DATA8(0x1B);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x0F);
    LCD_WR_DATA8(0x48);
    LCD_WR_DATA8(0xB8);
    LCD_WR_DATA8(0x34);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x0F);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x00);
    LCD_WR_REG(0XE1); //Set Gamma
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x23);
    LCD_WR_DATA8(0x24);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x10);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x38);
    LCD_WR_DATA8(0x47);
    LCD_WR_DATA8(0x4B);
    LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x13);
    LCD_WR_DATA8(0x06);
    LCD_WR_DATA8(0x30);
    LCD_WR_DATA8(0x38);
    LCD_WR_DATA8(0x0F);
    LCD_WR_REG(0x29); //Display on
}









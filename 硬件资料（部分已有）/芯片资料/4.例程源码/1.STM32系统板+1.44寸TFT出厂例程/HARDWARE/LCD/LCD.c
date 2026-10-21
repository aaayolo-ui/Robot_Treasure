#include "lcd.h"
#include "delay.h"
#include "font.h"

//液晶IO初始化配置
void LCD_GPIO_Init(void)
{

  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(LCD_SCL_RCC | LCD_SDA_RCC | LCD_RES_RCC | \
                         LCD_DC_RCC | LCD_CS_RCC| LCD_BLK_RCC, ENABLE);	 //使能端口时钟

  GPIO_InitStructure.GPIO_Pin = LCD_SCL_PIN;	 //推挽输出
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz
  GPIO_Init(LCD_SCL_GPIO_PORT, &GPIO_InitStructure);	 //初始化GPIO

  GPIO_InitStructure.GPIO_Pin = LCD_SDA_PIN;	 //推挽输出
  GPIO_Init(LCD_SDA_GPIO_PORT, &GPIO_InitStructure);//初始化GPIO

  GPIO_InitStructure.GPIO_Pin = LCD_RES_PIN;	 //推挽输出
  GPIO_Init(LCD_RES_GPIO_PORT, &GPIO_InitStructure);//初始化GPIO

  GPIO_InitStructure.GPIO_Pin = LCD_DC_PIN;	 //推挽输出
  GPIO_Init(LCD_DC_GPIO_PORT, &GPIO_InitStructure);//初始化GPIO

  GPIO_InitStructure.GPIO_Pin = LCD_CS_PIN;	 //推挽输出
  GPIO_Init(LCD_CS_GPIO_PORT, &GPIO_InitStructure);//初始化GPIO
  
  GPIO_InitStructure.GPIO_Pin = LCD_BLK_PIN;	 //推挽输出
  GPIO_Init(LCD_BLK_GPIO_PORT, &GPIO_InitStructure);//初始化GPIO

}
//向SPI总线传输一个8位数据
void  SPI_WriteData(u8 Data)
{
  unsigned char i = 0;
  for(i = 8; i > 0; i--)
  {
    if(Data & 0x80)
      LCD_SDA = 1; //输出数据
    else LCD_SDA = 0;

    LCD_SCL = 0;
    LCD_SCL = 1;
    Data <<= 1;
  }
}

//向液晶屏写一个8位指令
void LCD_WriteIndex(u8 Index)
{
  //SPI 写命令时序开始
  LCD_CS = 0;
  LCD_DC = 0;
  SPI_WriteData(Index);
  LCD_CS = 1;
}

//向液晶屏写一个8位数据
void LCD_WriteData(u8 Data)
{
  LCD_CS = 0;
  LCD_DC = 1;
  SPI_WriteData(Data);
  LCD_CS = 1;
}
//向液晶屏写一个16位数据
void LCD_WriteData_16Bit(u16 Data)
{
  LCD_CS = 0;
  LCD_DC = 1;
  SPI_WriteData(Data >> 8); 	//写入高8位数据
  SPI_WriteData(Data); 			//写入低8位数据
  LCD_CS = 1;
}

void LCD_WriteReg(u8 Index, u8 Data)
{
  LCD_WriteIndex(Index);
  LCD_WriteData(Data);
}

//复位LCD
void LCD_Reset(void)
{
  LCD_RES = 0;
  delay_ms(100);
  LCD_RES = 1;
  delay_ms(50);
}

//LCD 初始化，具体参考数据手册
void LCD_Init(void)
{
  LCD_GPIO_Init();
  LCD_Reset(); //Reset before LCD Init.

  //LCD Init For 1.44Inch LCD Panel with ST7735R.
  LCD_WriteIndex(0x11);//Sleep exit
  delay_ms (120);

  //ST7735R Frame Rate
  LCD_WriteIndex(0xB1);
  LCD_WriteData(0x01);
  LCD_WriteData(0x2C);
  LCD_WriteData(0x2D);

  LCD_WriteIndex(0xB2);
  LCD_WriteData(0x01);
  LCD_WriteData(0x2C);
  LCD_WriteData(0x2D);

  LCD_WriteIndex(0xB3);
  LCD_WriteData(0x01);
  LCD_WriteData(0x2C);
  LCD_WriteData(0x2D);
  LCD_WriteData(0x01);
  LCD_WriteData(0x2C);
  LCD_WriteData(0x2D);

  LCD_WriteIndex(0xB4); //Column inversion
  LCD_WriteData(0x07);

  //ST7735R Power Sequence
  LCD_WriteIndex(0xC0);
  LCD_WriteData(0xA2);
  LCD_WriteData(0x02);
  LCD_WriteData(0x84);
  LCD_WriteIndex(0xC1);
  LCD_WriteData(0xC5);

  LCD_WriteIndex(0xC2);
  LCD_WriteData(0x0A);
  LCD_WriteData(0x00);

  LCD_WriteIndex(0xC3);
  LCD_WriteData(0x8A);
  LCD_WriteData(0x2A);
  LCD_WriteIndex(0xC4);
  LCD_WriteData(0x8A);
  LCD_WriteData(0xEE);

  LCD_WriteIndex(0xC5); //VCOM
  LCD_WriteData(0x0E);

  LCD_WriteIndex(0x36); //MX, MY, RGB mode
  LCD_WriteData(0xC8);

  //ST7735R Gamma Sequence
  LCD_WriteIndex(0xe0);
  LCD_WriteData(0x0f);
  LCD_WriteData(0x1a);
  LCD_WriteData(0x0f);
  LCD_WriteData(0x18);
  LCD_WriteData(0x2f);
  LCD_WriteData(0x28);
  LCD_WriteData(0x20);
  LCD_WriteData(0x22);
  LCD_WriteData(0x1f);
  LCD_WriteData(0x1b);
  LCD_WriteData(0x23);
  LCD_WriteData(0x37);
  LCD_WriteData(0x00);
  LCD_WriteData(0x07);
  LCD_WriteData(0x02);
  LCD_WriteData(0x10);

  LCD_WriteIndex(0xe1);
  LCD_WriteData(0x0f);
  LCD_WriteData(0x1b);
  LCD_WriteData(0x0f);
  LCD_WriteData(0x17);
  LCD_WriteData(0x33);
  LCD_WriteData(0x2c);
  LCD_WriteData(0x29);
  LCD_WriteData(0x2e);
  LCD_WriteData(0x30);
  LCD_WriteData(0x30);
  LCD_WriteData(0x39);
  LCD_WriteData(0x3f);
  LCD_WriteData(0x00);
  LCD_WriteData(0x07);
  LCD_WriteData(0x03);
  LCD_WriteData(0x10);

  LCD_WriteIndex(0x2a);
  LCD_WriteData(0x00);
  LCD_WriteData(0x00);
  LCD_WriteData(0x00);
  LCD_WriteData(0x7f);

  LCD_WriteIndex(0x2b);
  LCD_WriteData(0x00);
  LCD_WriteData(0x00);
  LCD_WriteData(0x00);
  LCD_WriteData(0x9f);

  LCD_WriteIndex(0xF0); //Enable test command
  LCD_WriteData(0x01);
  LCD_WriteIndex(0xF6); //Disable ram power save mode
  LCD_WriteData(0x00);

  LCD_WriteIndex(0x3A); //65k mode
  LCD_WriteData(0x05);


  LCD_WriteIndex(0x29);//Display on
}


/*************************************************
函数名：LCD_Set_Region
功能：设置lcd显示区域，在此区域写点数据自动换行
入口参数：xy起点和终点
返回值：无
*************************************************/
void LCD_SetRegion(u16 x_start, u16 y_start, u16 x_end, u16 y_end)
{
  LCD_WriteIndex(0x2a);
  LCD_WriteData(0x00);
  LCD_WriteData(x_start + 2);
  LCD_WriteData(0x00);
  LCD_WriteData(x_end + 2);

  LCD_WriteIndex(0x2b);
  LCD_WriteData(0x00);
  LCD_WriteData(y_start + 3);
  LCD_WriteData(0x00);
  LCD_WriteData(y_end + 3);

  LCD_WriteIndex(0x2c);

}

/*************************************************
函数名：LCD_Set_XY
功能：设置lcd显示起始点
入口参数：xy坐标
返回值：无
*************************************************/
void LCD_SetXY(u16 x, u16 y)
{
  LCD_SetRegion(x, y, x, y);
}


/*************************************************
函数名：LCD_DrawPoint
功能：画一个点
入口参数：无
返回值：无
*************************************************/
void LCD_DrawPoint(u16 x, u16 y, u16 Data)
{
  LCD_SetRegion(x, y, x + 1, y + 1);
  LCD_WriteData_16Bit(Data);

}

/*****************************************
 函数功能：读TFT某一点的颜色
 出口参数：color  点颜色值
******************************************/
unsigned int LCD_ReadPoint(u16 x, u16 y)
{
  unsigned int Data;
  LCD_SetXY(x, y);

  //LCD_ReadData();//丢掉无用字节
  //Data=LCD_ReadData();
  LCD_WriteData(Data);
  return Data;
}
/*************************************************
函数名：LCD_Clear
功能：全屏清屏函数
入口参数：填充颜色COLOR
返回值：无
*************************************************/
void LCD_Clear(u16 Color)
{
  unsigned int i, m;
  LCD_SetRegion(0, 0, X_MAX_PIXEL - 1, Y_MAX_PIXEL - 1);
  LCD_WriteIndex(0x2C);
  for(i = 0; i < X_MAX_PIXEL; i++)
    for(m = 0; m < Y_MAX_PIXEL; m++)
    {
      LCD_WriteData_16Bit(Color);
    }
}

//从ILI93xx读出的数据为GBR格式，而我们写入的时候为RGB格式。
//通过该函数转换
//c:GBR格式的颜色值
//返回值：RGB格式的颜色值
u16 LCD_BGR2RGB(u16 c)
{
  u16  r, g, b, rgb;
  b = (c >> 0) & 0x1f;
  g = (c >> 5) & 0x3f;
  r = (c >> 11) & 0x1f;
  rgb = (b << 11) + (g << 5) + (r << 0);
  return(rgb);
}


//画圆
void LCD_Circle(u16 X, u16 Y, u16 R, u16 fc)
{
  //Bresenham算法
  unsigned short  a, b;
  int c;
  a = 0;
  b = R;
  c = 3 - 2 * R;
  while (a < b)
  {
    LCD_DrawPoint(X + a, Y + b, fc); //        7
    LCD_DrawPoint(X - a, Y + b, fc); //        6
    LCD_DrawPoint(X + a, Y - b, fc); //        2
    LCD_DrawPoint(X - a, Y - b, fc); //        3
    LCD_DrawPoint(X + b, Y + a, fc); //        8
    LCD_DrawPoint(X - b, Y + a, fc); //        5
    LCD_DrawPoint(X + b, Y - a, fc); //        1
    LCD_DrawPoint(X - b, Y - a, fc); //        4

    if(c < 0) c = c + 4 * a + 6;
    else
    {
      c = c + 4 * (a - b) + 10;
      b -= 1;
    }
    a += 1;
  }
  if (a == b)
  {
    LCD_DrawPoint(X + a, Y + b, fc);
    LCD_DrawPoint(X + a, Y + b, fc);
    LCD_DrawPoint(X + a, Y - b, fc);
    LCD_DrawPoint(X - a, Y - b, fc);
    LCD_DrawPoint(X + b, Y + a, fc);
    LCD_DrawPoint(X - b, Y + a, fc);
    LCD_DrawPoint(X + b, Y - a, fc);
    LCD_DrawPoint(X - b, Y - a, fc);
  }
}

//画线函数，使用Bresenham 画线算法
void LCD_DrawLine(u16 x0, u16 y0, u16 x1, u16 y1, u16 Color)
{
  int dx,             // difference in x's
      dy,             // difference in y's
      dx2,            // dx,dy * 2
      dy2,
      x_inc,          // amount in pixel space to move during drawing
      y_inc,          // amount in pixel space to move during drawing
      error,          // the discriminant i.e. error i.e. decision variable
      index;          // used for looping


  LCD_SetXY(x0, y0);
  dx = x1 - x0; //计算x距离
  dy = y1 - y0; //计算y距离

  if (dx >= 0)
  {
    x_inc = 1;
  }
  else
  {
    x_inc = -1;
    dx    = -dx;
  }

  if (dy >= 0)
  {
    y_inc = 1;
  }
  else
  {
    y_inc = -1;
    dy    = -dy;
  }

  dx2 = dx << 1;
  dy2 = dy << 1;

  if (dx > dy)//x距离大于y距离，那么每个x轴上只有一个点，每个y轴上有若干个点
  {
    //且线的点数等于x距离，以x轴递增画点
    // initialize error term
    error = dy2 - dx;

    // draw the line
    for (index = 0; index <= dx; index++) //要画的点数不会超过x距离
    {
      //画点
      LCD_DrawPoint(x0, y0, Color);

      // test if error has overflowed
      if (error >= 0) //是否需要增加y坐标值
      {
        error -= dx2;

        // move to next line
        y0 += y_inc; //增加y坐标值
      } // end if error overflowed

      // adjust the error term
      error += dy2;

      // move to the next pixel
      x0 += x_inc; //x坐标值每次画点后都递增1
    } // end for
  } // end if |slope| <= 1
  else//y轴大于x轴，则每个y轴上只有一个点，x轴若干个点
  {
    //以y轴为递增画点
    // initialize error term
    error = dx2 - dy;

    // draw the line
    for (index = 0; index <= dy; index++)
    {
      // set the pixel
      LCD_DrawPoint(x0, y0, Color);

      // test if error overflowed
      if (error >= 0)
      {
        error -= dy2;

        // move to next line
        x0 += x_inc;
      } // end if error overflowed

      // adjust the error term
      error += dx2;

      // move to the next pixel
      y0 += y_inc;
    } // end for
  } // end else |slope| > 1
}

//画矩形
void LCD_box(u16 x, u16 y, u16 w, u16 h, u16 bc)
{
  LCD_DrawLine(x, y, x + w, y, 0xEF7D);
  LCD_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0x2965);
  LCD_DrawLine(x, y + h, x + w, y + h, 0x2965);
  LCD_DrawLine(x, y, x, y + h, 0xEF7D);
  LCD_DrawLine(x + 1, y + 1, x + 1 + w - 2, y + 1 + h - 2, bc);
}

//画矩形
void LCD_box2(u16 x, u16 y, u16 w, u16 h, u8 mode)
{
  if (mode == 0)
  {
    LCD_DrawLine(x, y, x + w, y, 0xEF7D);
    LCD_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0x2965);
    LCD_DrawLine(x, y + h, x + w, y + h, 0x2965);
    LCD_DrawLine(x, y, x, y + h, 0xEF7D);
  }
  if (mode == 1)
  {
    LCD_DrawLine(x, y, x + w, y, 0x2965);
    LCD_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0xEF7D);
    LCD_DrawLine(x, y + h, x + w, y + h, 0xEF7D);
    LCD_DrawLine(x, y, x, y + h, 0x2965);
  }
  if (mode == 2)
  {
    LCD_DrawLine(x, y, x + w, y, 0xffff);
    LCD_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0xffff);
    LCD_DrawLine(x, y + h, x + w, y + h, 0xffff);
    LCD_DrawLine(x, y, x, y + h, 0xffff);
  }
}

/**************************************************************************************
功能描述: 在屏幕显示一凸起的按钮框
输    入: u16 x1,y1,x2,y2 按钮框左上角和右下角坐标
输    出: 无
**************************************************************************************/
void DisplayButtonDown(u16 x1, u16 y1, u16 x2, u16 y2)
{
  LCD_DrawLine(x1,  y1,  x2, y1, GRAY2); //H
  LCD_DrawLine(x1 + 1, y1 + 1, x2, y1 + 1, GRAY1); //H
  LCD_DrawLine(x1,  y1,  x1, y2, GRAY2); //V
  LCD_DrawLine(x1 + 1, y1 + 1, x1 + 1, y2, GRAY1); //V
  LCD_DrawLine(x1,  y2,  x2, y2, WHITE); //H
  LCD_DrawLine(x2,  y1,  x2, y2, WHITE); //V
}

/**************************************************************************************
功能描述: 在屏幕显示一凹下的按钮框
输    入: u16 x1,y1,x2,y2 按钮框左上角和右下角坐标
输    出: 无
**************************************************************************************/
void DisplayButtonUp(u16 x1, u16 y1, u16 x2, u16 y2)
{
  LCD_DrawLine(x1,  y1,  x2, y1, WHITE); //H
  LCD_DrawLine(x1,  y1,  x1, y2, WHITE); //V

  LCD_DrawLine(x1 + 1, y2 - 1, x2, y2 - 1, GRAY1); //H
  LCD_DrawLine(x1,  y2,  x2, y2, GRAY2); //H
  LCD_DrawLine(x2 - 1, y1 + 1, x2 - 1, y2, GRAY1); //V
  LCD_DrawLine(x2, y1, x2, y2, GRAY2);  //V
}

//显示16*16中英文字模
//x:横坐标
//y:纵坐标
//fc:字体颜色
//bc:背景颜色
//*s:要显示的字
void LCD_DrawFont_GBK16(u16 x, u16 y, u16 fc, u16 bc, u8 *s)
{
  unsigned char i, j;
  unsigned short k, x0;
  x0 = x;

  while(*s)
  {
    if((*s) < 128)
    {
      k = *s;
      if (k == 13)
      {
        x = x0;
        y += 16;
      }
      else
      {
        if (k > 32) k -= 32;
        else k = 0;

        for(i = 0; i < 16; i++)
          for(j = 0; j < 8; j++)
          {
            if(asc16[k * 16 + i] & (0x80 >> j))	LCD_DrawPoint(x + j, y + i, fc);
            else
            {
              if (fc != bc) LCD_DrawPoint(x + j, y + i, bc);
            }
          }
        x += 8;
      }
      s++;
    }

    else
    {


      for (k = 0; k < hz16_num; k++)
      {
        if ((hz16[k].Index[0] == *(s)) && (hz16[k].Index[1] == *(s + 1)))
        {
          for(i = 0; i < 16; i++)
          {
            for(j = 0; j < 8; j++)
            {
              if(hz16[k].Msk[i * 2] & (0x80 >> j))	LCD_DrawPoint(x + j, y + i, fc);
              else
              {
                if (fc != bc) LCD_DrawPoint(x + j, y + i, bc);
              }
            }
            for(j = 0; j < 8; j++)
            {
              if(hz16[k].Msk[i * 2 + 1] & (0x80 >> j))	LCD_DrawPoint(x + j + 8, y + i, fc);
              else
              {
                if (fc != bc) LCD_DrawPoint(x + j + 8, y + i, bc);
              }
            }
          }
        }
      }
      s += 2;
      x += 16;
    }

  }
}

//显示24*24中英文字模
//x:横坐标
//y:纵坐标
//fc:字体颜色
//bc:背景颜色
//*s:要显示的字
void LCD_DrawFont_GBK24(u16 x, u16 y, u16 fc, u16 bc, u8 *s)
{
  unsigned char i, j;
  unsigned short k;

  while(*s)
  {
    if( *s < 0x80 )
    {
      k = *s;
      if (k > 32) k -= 32;
      else k = 0;

      for(i = 0; i < 16; i++)
        for(j = 0; j < 8; j++)
        {
          if(asc16[k * 16 + i] & (0x80 >> j))
            LCD_DrawPoint(x + j, y + i, fc);
          else
          {
            if (fc != bc) LCD_DrawPoint(x + j, y + i, bc);
          }
        }
      s++;
      x += 8;
    }
    else
    {

      for (k = 0; k < hz24_num; k++)
      {
        if ((hz24[k].Index[0] == *(s)) && (hz24[k].Index[1] == *(s + 1)))
        {
          for(i = 0; i < 24; i++)
          {
            for(j = 0; j < 8; j++)
            {
              if(hz24[k].Msk[i * 3] & (0x80 >> j))
                LCD_DrawPoint(x + j, y + i, fc);
              else
              {
                if (fc != bc) LCD_DrawPoint(x + j, y + i, bc);
              }
            }
            for(j = 0; j < 8; j++)
            {
              if(hz24[k].Msk[i * 3 + 1] & (0x80 >> j))	LCD_DrawPoint(x + j + 8, y + i, fc);
              else
              {
                if (fc != bc) LCD_DrawPoint(x + j + 8, y + i, bc);
              }
            }
            for(j = 0; j < 8; j++)
            {
              if(hz24[k].Msk[i * 3 + 2] & (0x80 >> j))
                LCD_DrawPoint(x + j + 16, y + i, fc);
              else
              {
                if (fc != bc) LCD_DrawPoint(x + j + 16, y + i, bc);
              }
            }
          }
        }
      }
      s += 2;
      x += 24;
    }
  }
}

//显示32*32数码管数字
//x:横坐标
//y:纵坐标
//fc:字体颜色
//bc:背景颜色
//num:要显示的数字（0--9）
void LCD_DrawFont_Num32(u16 x, u16 y, u16 fc, u16 bc, u16 num)
{
  unsigned char i, j, k, c;

  for(i = 0; i < 32; i++)
  {
    for(j = 0; j < 4; j++)
    {
      c = *(sz32 + num * 32 * 4 + i * 4 + j);
      for (k = 0; k < 8; k++)
      {

        if(c & (0x80 >> k))	LCD_DrawPoint(x + j * 8 + k, y + i, fc);
        else
        {
          if (fc != bc) LCD_DrawPoint(x + j * 8 + k, y + i, bc);
        }
      }
    }
  }
}

//取模方式 水平扫描 从左到右 低位在前
//显示图片
//x:起始横坐标
//y:起始纵坐标
//x1:图片宽度
//y1:图片高度

//*p:图片数组
void LCD_Showimage(u16 x0, u16 y0, u16 x1, u16 y1, const unsigned char *p) //显示128*128 图片
{
  int i;
  unsigned char picH, picL;

  LCD_SetRegion(x0, y0, x0 + x1 - 1, y0 + y1 - 1);		//坐标设置:扫描起点到终点0到127，刚好128个点
  for(i = 0; i < x1 * y1; i++)
  {
    picL = *(p + i * 2);	//数据低位在前
    picH = *(p + i * 2 + 1);
    LCD_WriteData_16Bit(picH << 8 | picL);
  }

}


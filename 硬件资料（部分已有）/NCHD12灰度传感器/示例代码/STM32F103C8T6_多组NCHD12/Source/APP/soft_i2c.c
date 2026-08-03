#include "soft_i2c.h"
/* 私有宏 ------------------------------------------------------------------------------------*/
#define GPIO_PORT_I2C	GPIOA			              /* GPIO端口 */
#define RCC_I2C_PORT 	RCC_APB2Periph_GPIOA		/* GPIO端口时钟 */
#define I2C_SCL_PIN		GPIO_Pin_0			        /* 连接到SCL时钟线的GPIO */
#define I2C_SDA_PIN		GPIO_Pin_1			        /* 连接到SDA数据线的GPIO */

#define I2C_SCL_1()  GPIO_SetBits(GPIO_PORT_I2C, I2C_SCL_PIN)		/* SCL = 1 */
#define I2C_SCL_0()  GPIO_ResetBits(GPIO_PORT_I2C, I2C_SCL_PIN)		/* SCL = 0 */

#define I2C_SDA_1()  GPIO_SetBits(GPIO_PORT_I2C, I2C_SDA_PIN)		/* SDA = 1 */
#define I2C_SDA_0()  GPIO_ResetBits(GPIO_PORT_I2C, I2C_SDA_PIN)		/* SDA = 0 */

#define I2C_SDA_READ()  GPIO_ReadInputDataBit(GPIO_PORT_I2C, I2C_SDA_PIN)	/* 读SDA口线状态 */

/* 私有变量 ----------------------------------------------------------------------------------*/

/* 私有结构体 --------------------------------------------------------------------------------*/

/* 私有函数声明 ------------------------------------------------------------------------------*/
static void I2C_GPIO_Configuration(void);
static void i2c_Delay(void);
 
/* 私有函数模型 ------------------------------------------------------------------------------*/

/**********************************************************************************************
 *名    称：static void I2C_GPIO_Configuration(void)
 *
 *参    数：无
 *
 *返 回 值：无 
 *
 *描    述：I2C IO口配置
 *********************************************************************************************/
static void I2C_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_I2C_PORT, ENABLE);	/* 打开GPIO时钟 */

	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  	/* 开漏输出 */
	GPIO_Init(GPIO_PORT_I2C, &GPIO_InitStructure);

	/* 给一个停止信号, 复位I2C总线上的所有设备到待机模式 */
	i2c_Stop();
}
 
/**********************************************************************************************
 *名    称：static void i2c_Delay(void)
 *
 *参    数：无
 *
 *返 回 值：无
 *
 *描    述：延时函数
 *********************************************************************************************/
static void i2c_Delay(void)
{
	uint8_t i;

	/*　
	 	下面的时间是通过安富莱AX-Pro逻辑分析仪测试得到的。
		CPU主频72MHz时，在内部Flash运行, MDK工程不优化
		循环次数为10时，SCL频率 = 205KHz 
		循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us 
	 	循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us 
        
    IAR工程编译效率高，不能设置为7
	*/
	for (i = 0; i < 10; i++);
}

/**********************************************************************************************
 *名    称：void i2c_Start(void)
 *
 *参    数：无
 *
 *返 回 值：无
 *
 *描    述：发送起始信号
 *********************************************************************************************/
void i2c_Start(void)
{
	/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/**********************************************************************************************
 *名    称：void i2c_Stop(void)
 *
 *参    数：无
 *
 *返 回 值：无
 *
 *描    述：发送终止信号
 *********************************************************************************************/
void i2c_Stop(void)
{
	/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
	I2C_SDA_0();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
}

/**********************************************************************************************
 *名    称：void i2c_SendByte(uint8_t _ucByte)
 *
 *参    数： \param  uint8_t _ucByte   //被发送的数据
 *
 *返 回 值：无
 *
 *描    述：向I2C总线设备发送8bit数据
 *********************************************************************************************/
void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* 先发送字节的高位bit7 */
	for (i = 0; i < 8; i++)
	{		
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();	
		I2C_SCL_0();
		if (i == 7)
		{
			 I2C_SDA_1(); // 释放总线
		}
		_ucByte <<= 1;	/* 左移一个bit */
		i2c_Delay();
	}
}

/**********************************************************************************************
 *名    称：uint8_t i2c_ReadByte(uint8_t Ack)
 *
 *参    数：uint8_t Ack   1为ACK信号，0为NACK信号
 *
 *返 回 值：value  通过I2C总线读取的数据
 *
 *描    述：从I2C总线设备读取8bit数据
 *********************************************************************************************/
uint8_t i2c_ReadByte(uint8_t Ack)
{
	uint8_t i;
	uint8_t value;

	/* 读到第1个bit为数据的bit7 */
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_0();
		i2c_Delay();
	}
	if(Ack)
	{
		i2c_Ack();
	}
	else
	{
		i2c_NAck();
	}
	return value;
}

/**********************************************************************************************
 *名    称：uint8_t i2c_WaitAck(void)
 *
 *参    数：无
 *
 *返 回 值：re  返回0表示正确应答，1表示无器件响应
 *
 *描    述：产生一个时钟，并读取器件的ACK应答信号
 *********************************************************************************************/
uint8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	/* CPU释放SDA总线 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	i2c_Delay();
	if (I2C_SDA_READ())	/* CPU读取SDA口线状态 */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_0();
	i2c_Delay();
	return re;
}

/**********************************************************************************************
 *名    称：void i2c_Ack(void)
 *
 *参    数：无
 *
 *返 回 值：无
 *
 *描    述：产生一个ACK信号
 *********************************************************************************************/
void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU驱动SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU释放SDA总线 */
}

/**********************************************************************************************
 *名    称：void i2c_NAck(void)
 *
 *参    数：无
 *
 *返 回 值：无
 *
 *描    述：产生1个NACK信号
 *********************************************************************************************/
void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU驱动SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();	
}

/**********************************************************************************************
 *名    称：uint8_t i2c_CheckDevice(uint8_t _Address)
 *
 *参    数： \param  uint8_t _Address  器件地址
 * 
 *返 回 值：ucAck  返回值 0 表示正确， 返回1表示未探测到
 *
 *描    述：检测I2C总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
 *********************************************************************************************/
uint8_t i2c_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	I2C_GPIO_Configuration();		/* 配置GPIO */
	
	i2c_Start();		/* 发送启动信号 */

	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	i2c_SendByte(_Address | I2C_WR);
	ucAck = i2c_WaitAck();	/* 检测设备的ACK应答 */

	i2c_Stop();			/* 发送停止信号 */

	return ucAck;
}
/* 文件结束 ---------------------------------------------------------------------------------*/




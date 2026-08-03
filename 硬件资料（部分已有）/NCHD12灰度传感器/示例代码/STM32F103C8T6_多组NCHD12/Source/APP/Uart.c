#include "stm32f10x.h"
#include "Uart.h"
#include "stdio.h"
#include "stdint.h"
#include "math.h"
#include "delay.h"
int fputc(int ch, FILE *f)
{
	/* 将Printf内容发往串口 */
	USART_SendData(USART1, (unsigned char) ch);
	while (!(USART1->SR & USART_FLAG_TXE));	
	return (ch);
}


void USART1_Send_Byte(uint16_t Data)
{ 
	USART_SendData(USART1,Data);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	 
}

void USART1_Send_String(u8 *p)
{   
	while(*p)
	{	 
	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_SendData(USART1,*p++);			 
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{;}
	}
}

uint8_t USART1_Receive_Byte(void)
{ 
	while(!(USART_GetFlagStatus(USART1,USART_FLAG_RXNE))); //USART_GetFlagStatus：得到接收状态位
	//USART_FLAG_RXNE:接收数据寄存器非空标志位 
	//1：忙状态  0：空闲(没收到数据，等待。。。)
	return USART_ReceiveData(USART1);					   //接收一个字符
}


void Usart1_Configuration(uint32_t BaudRate)
{	
	GPIO_InitTypeDef GPIO_InitStructure;					//定义一个GPIO结构体变量
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,ENABLE);	
	//使能各个端口时钟，重要！！！
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 				//配置串口接收端口挂接到9端口
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	   		//复用功能输出开漏
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   	//配置端口速度为50M
	GPIO_Init(GPIOA, &GPIO_InitStructure);				   	//根据参数初始化GPIOA寄存器	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//浮空输入(复位状态);

	
	GPIO_Init(GPIOA, &GPIO_InitStructure);				   	//根据参数初始化GPIOA寄存器	
	USART_InitTypeDef USART_InitStructure;							    	//定义一个串口结构体
	USART_InitStructure.USART_BaudRate            =BaudRate ;	  			//波特率115200
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b; 	//传输过程中使用8位数据
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;	 	//在帧结尾传输1位停止位
	USART_InitStructure.USART_Parity              = USART_Parity_No ;	 	//奇偶失能
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流失能
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx; //接收和发送模式
	USART_Init(USART1, &USART_InitStructure);								//根据参数初始化串口寄存器
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);							//使能串口中断接收
	USART_Cmd(USART1, ENABLE);     											//使能串口外设
}

void USART1_IRQHandler()
{
	uint8_t ReceiveDate;								//定义一个变量存放接收的数据
	if(!(USART_GetITStatus(USART1,USART_IT_RXNE))){;} 	//读取接收中断标志位USART_IT_RXNE 
	{
	ReceiveDate=USART_ReceiveData(USART1);
	USART1_Send_Byte(ReceiveDate);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);		    
	}  
}

void Usart_Senddata(unsigned char *Data,unsigned int  Len)
{
		USART_ClearFlag(USART1, USART_FLAG_TC);
		USART_ClearITPendingBit(USART1, USART_FLAG_TXE);
		while(Len--)
		{
		USART_SendData(USART1,*Data);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != 1);
		USART_ClearFlag(USART1, USART_FLAG_TC);
		USART_ClearITPendingBit(USART1, USART_FLAG_TXE);
		Data++; 
		}
}
void wust_sendware(unsigned char *wareaddr, uint16_t waresize)//示波器
{
    #define CMD_WARE     3
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};   
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};   
    Usart_Senddata(cmdf, sizeof(cmdf));    //先发送命令
    Usart_Senddata(wareaddr, waresize); //再发送图像
    Usart_Senddata(cmdr, sizeof(cmdr)); //先发送命令
}


/********************************************************/
/**
* @brief  RingBuff_Init
* @param  void
* @return void
* @author 杰杰
* @date   2018
* @version v1.0
* @note   初始化环形缓冲区
*/
void RingBuff_Init(RingBuff_t *ringBuff)
{
  //初始化相关信息
  ringBuff->Head = 0;
  ringBuff->Tail = 0;
  ringBuff->Lenght = 0;
}

/**
* @brief  Write_RingBuff
* @param  u8 data
* @return FLASE:环形缓冲区已满，写入失败;TRUE:写入成功
* @author 杰杰
* @date   2018
* @version v1.0
* @note   往环形缓冲区写入u8类型的数据
*/
uint8_t Write_RingBuff(uint8_t data,RingBuff_t *ringBuff)
{
  if(ringBuff->Lenght >= RINGBUFF_LEN) //判断缓冲区是否已满
  {
    return 0;
  }
  ringBuff->Ring_Buff[ringBuff->Tail]=data;
  //ringBuff.Tail++;
  ringBuff->Tail = (ringBuff->Tail+1)%RINGBUFF_LEN;//防止越界非法访问
  ringBuff->Lenght++;
  return 1;
}

/**
* @brief  Read_RingBuff
* @param  u8 *rData，用于保存读取的数据
* @return FLASE:环形缓冲区没有数据，读取失败;TRUE:读取成功
* @author 杰杰
* @date   2018
* @version v1.0
* @note   从环形缓冲区读取一个u8类型的数据
*/
uint8_t Read_RingBuff(uint8_t *rData,RingBuff_t *ringBuff)
{
  if(ringBuff->Lenght == 0)//判断非空
  {
    return 0;
  }
  *rData = ringBuff->Ring_Buff[ringBuff->Head];//先进先出FIFO，从缓冲区头出
  //ringBuff.Head++;
  ringBuff->Head = (ringBuff->Head+1)%RINGBUFF_LEN;//防止越界非法访问
  ringBuff->Lenght--;
  return 1;
}



void RingBuf_Write(unsigned char data,RingBuff_t *ringBuff,uint16_t Length)
{
  ringBuff->Ring_Buff[ringBuff->Tail]=data;//从尾部追加
  if(++ringBuff->Tail>=Length)//尾节点偏移
    ringBuff->Tail=0;//大于数组最大长度 归零 形成环形队列  
  if(ringBuff->Tail==ringBuff->Head)//如果尾部节点追到头部节点，则修改头节点偏移位置丢弃早期数据
  {
    if((++ringBuff->Head)>=Length)  
      ringBuff->Head=0; 
  }
}

uint8_t RingBuf_Read(unsigned char* pData,RingBuff_t *ringBuff)
{
  if(ringBuff->Head==ringBuff->Tail)  return 1;//如果头尾接触表示缓冲区为空
  else 
  {  
    *pData=ringBuff->Ring_Buff[ringBuff->Head];//如果缓冲区非空则取头节点值并偏移头节点
    if((++ringBuff->Head)>=RINGBUFF_LEN)   ringBuff->Head=0;
    return 0;//返回0，表示读取数据成功
  }
}



void Init_Usart3(unsigned long bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  //	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = bound;//
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8bits
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//stop bit is 1
  USART_InitStructure.USART_Parity = USART_Parity_No;//no parity
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//no Hardware Flow Control
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//enable tx and rx
  USART_Init(USART3, &USART_InitStructure);//
  USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);//rx interrupt is enable
  USART_Cmd(USART3, ENABLE);	
}

void USART3_IRQHandler(void)
{
  if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
  {
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    USART_ReceiveData(USART3);
  }
}


/**			                                                    
		   ____                    _____ _____  _____        XTARK@塔克创新
		  / __ \                  / ____|  __ \|  __ \  
		 | |  | |_ __   ___ _ __ | |    | |__) | |__) |
		 | |  | | '_ \ / _ \ '_ \| |    |  _  /|  ___/ 
		 | |__| | |_) |  __/ | | | |____| | \ \| |     
		  \____/| .__/ \___|_| |_|\_____|_|  \_\_|     
		    	| |                                    
		    	|_|  OpenCRP 树莓派 专用ROS机器人控制器                                   
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 官网网站： www.xtark.cn
  * 淘宝店铺： https://shop246676508.taobao.com  
  * 塔克媒体： www.cnblogs.com/xtark（博客）
  * 塔克微信： 微信公众号：塔克创新（获取最新资讯）
  *      
  ******************************************************************************
  * @作  者  塔克创新团队
  * @brief   电机PWM控制函数
  *
  ******************************************************************************
  * @说  明
  *
  ******************************************************************************
  */

#include "ax_motor.h" 

/**
  * @简  述  电机PWM控制初始化，PWM频率为10KHZ	
  * @参  数  无
  * @返回值  无
  */
void AX_MOTOR_Init(void)
{ 
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 

	//定时器通道IO配置
	//GPIO及复用功能时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);	

	//配置IO口为复用功能-定时器通道
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//电机方向控制IO配置
	//IO时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

	//电机A方向控制IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//电机B方向控制IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//电机C方向控制IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_0;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//电机D方向控制IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_1;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//TIM时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	//PWM频率为10KHZ，PWM占空比7200档可调
	TIM_TimeBaseStructure.TIM_Period = 7200-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	//PWM1 Mode configuration: Channel1 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;	    //占空比初始化
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

	//PWM1 Mode configuration: Channel2
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);

	//PWM1 Mode configuration: Channel3
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

	//PWM1 Mode configuration: Channel4
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM8, ENABLE);

	//TIM enable counter
	TIM_Cmd(TIM8, ENABLE);   

	//使能MOE位
	TIM_CtrlPWMOutputs(TIM8,ENABLE);	
}

/**
  * @简  述 电机A PWM速度控制
  * @参  数 speed 电机转速数值，范围-7200~7200
  * @返回值 无
  */
void AX_MOTOR_A_SetSpeed(int16_t speed)
{
	uint16_t temp;

	if(speed > 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);
	    GPIO_SetBits(GPIOB, GPIO_Pin_13);
		temp = speed;	
	}
	else if(speed < 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
		temp = (-speed);
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);
		temp = 0;
	}

	if(temp>7200)
		temp = 7200;

	TIM_SetCompare1(TIM8,temp);
}

/**
  * @简  述 电机B PWM速度控制
  * @参  数 speed 电机转速数值，范围-7200~7200
  * @返回值 无
  */
void AX_MOTOR_B_SetSpeed(int16_t speed)
{
	uint16_t temp;

    if(speed > 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_14);
	  GPIO_SetBits(GPIOB, GPIO_Pin_12);
		temp = speed;	
	}
	else if(speed < 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	  GPIO_SetBits(GPIOB, GPIO_Pin_14);
		temp = (-speed);
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	  GPIO_ResetBits(GPIOB, GPIO_Pin_14);
		temp = 0;
	}
	
	if(temp>7200)
		temp = 7200;
	
	TIM_SetCompare2(TIM8,temp);
}
/**
  * @简  述 电机C PWM速度控制
  * @参  数 speed 电机转速数值，范围-7200~7200
  * @返回值 无
  */
void AX_MOTOR_C_SetSpeed(int16_t speed)
{
	uint16_t temp;

  if(speed > 0)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_2);
	  GPIO_SetBits(GPIOC, GPIO_Pin_0);
		temp = speed;	
	}
	else if(speed < 0)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_0);
	  GPIO_SetBits(GPIOC, GPIO_Pin_2);
		temp = (-speed);
	}
	else
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_2);
	  GPIO_ResetBits(GPIOC, GPIO_Pin_0);
		temp = 0;
	}
	
	if(temp>7200)
		temp = 7200;
	
	TIM_SetCompare3(TIM8,temp);
}

/**
  * @简  述 电机D PWM速度控制
  * @参  数 speed 电机转速数值，范围-7200~7200
  * @返回值 无
  */
void AX_MOTOR_D_SetSpeed(int16_t speed)
{
	uint16_t temp;

  if(speed > 0)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_3);
	  GPIO_SetBits(GPIOC, GPIO_Pin_1);
		temp = speed;	
	}
	else if(speed < 0)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_1);
	  GPIO_SetBits(GPIOC, GPIO_Pin_3);
		temp = (-speed);
	}
	else
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_1);
	  GPIO_ResetBits(GPIOC, GPIO_Pin_3);
		temp = 0;
	}
	
	if(temp>7200)
		temp = 7200;
	
	TIM_SetCompare4(TIM8,temp);
}



/******************* (C) 版权 2026 XTARK **************************************/

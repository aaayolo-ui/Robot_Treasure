/**			                                                    
		   ____                    _____ _______ _____       XTARK@塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   机器人控制器
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *           
  ******************************************************************************
  * @作  者  塔克创新团队
  * @内  容  TB6612四路编码器电机例程
  * 
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>   


#include "ax_sys.h"    //系统设置
#include "ax_delay.h"  //软件延时
#include "ax_uart1.h"  //调试串口
#include "ax_vin.h"    //输入电压检测

#include "ax_motor.h"   //直流电机调速控制
#include "ax_encoder.h" //编码器控制

int16_t  encoder[4];	//编码器数值
uint16_t adc_value;     //电压采集值

/**
  * @简  述  主程序
  * @参  数  无
  * @返回值  无
  */
int main(void)
{
	//软件延时初始化
	AX_DELAY_Init(); 	
	
	//调试串口初始化
	AX_UART1_Init(115200); //调试串口
	printf("  \r\n"); //输出空格，CPUBUG	
	
	printf("XTARK TB6612 四路编码器电机驱动例程 \r\n");  
	
	//JTAG口设置
	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 		
	
	//电机初始化
	AX_MOTOR_Init();
	
 	//正交编码器初始化
	AX_ENCODER_A_Init();  
	AX_ENCODER_B_Init(); 	
	AX_ENCODER_C_Init();  
	AX_ENCODER_D_Init(); 

	//电压采样初始化
	AX_VIN_Init();
	
	while (1) 
	{		
		//控制电机转速，设置范围（±7200）
		AX_MOTOR_A_SetSpeed(-2000);
		AX_MOTOR_B_SetSpeed(-2000);
		AX_MOTOR_C_SetSpeed(-2000);
		AX_MOTOR_D_SetSpeed(-2000);
		
		//电机编码器采集
		encoder[0] = AX_ENCODER_A_GetCounter();
		encoder[1] = AX_ENCODER_B_GetCounter();
	    encoder[2] = AX_ENCODER_C_GetCounter();
		encoder[3] = AX_ENCODER_D_GetCounter();
		
		//设置编码器为0
		AX_ENCODER_A_SetCounter(0);  
		AX_ENCODER_B_SetCounter(0);	
		AX_ENCODER_C_SetCounter(0);  
		AX_ENCODER_D_SetCounter(0);	
		
		//输入电压采集，AD转换
		adc_value = AX_VIN_GetVol_X100();
		
		//串口输出内容，两个编码器数值和电压值
		printf("MA:%d  MB:%d  MC:%d  MD:%d  VIN:%d  \r\n",encoder[0], encoder[1], encoder[2], encoder[3], adc_value);  
		
		//延时
		AX_Delayms(100);
	}
}

/******************* (C) 版权 2026 XTARK **************************************/


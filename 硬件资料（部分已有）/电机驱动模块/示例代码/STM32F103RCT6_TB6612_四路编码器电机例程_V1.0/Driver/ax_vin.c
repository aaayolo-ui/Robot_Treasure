/**			                                                    
		   ____                    _____ _______ _____       @塔克创新
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
  * @内  容  VIN输入电压检测
  *
  ******************************************************************************
  * @说  明
  *
  * 1.使用ADC2对VIN输入电压采样，查询方式
	* 2.输入电压检测功能，可实现对电池电量的评估
  * 
  ******************************************************************************
  */

#include "ax_vin.h"

#define ADC_REVISE  99.9

/**
  * @简  述  VIN 输入电压检测初始化
  * @参  数  无
  * @返回值  无
  */
void AX_VIN_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	//配置GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//**配置ADC******
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	//ADC模式配置
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC2, &ADC_InitStructure);	

	//设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	//配置ADC通道、转换顺序和采样时间
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 1, ADC_SampleTime_239Cycles5);

	//使能ADC
	ADC_Cmd(ADC2, ENABLE);

	//初始化ADC 校准寄存器  
	ADC_ResetCalibration(ADC2);

	//等待校准寄存器初始化完成
	while(ADC_GetResetCalibrationStatus(ADC2));

	//ADC开始校准
	ADC_StartCalibration(ADC2);

	//等待校准完成
	while(ADC_GetCalibrationStatus(ADC2)); 	
}


/**
  * @简  述  VIN 获得输入电压
  * @参  数  无	  
  * @返回值  电压值，扩大100倍。例如7.2V输出为720。
  */
uint16_t AX_VIN_GetVol_X100(void)
{
	uint16_t Input_Vol,temp;

	//软件启动转换功能 
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);  

	//等待转换结束 
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));

	temp = ADC_GetConversionValue(ADC2);
	
	//分压比例1/11
	Input_Vol = (uint16_t)((3.3*11.0*ADC_REVISE * temp)/4095);	
	
	return Input_Vol;
}


/******************* (C) 版权 2026 XTARK **************************************/

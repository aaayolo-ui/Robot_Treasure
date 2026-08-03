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
  * @内  容  编码器
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_ENCODER_H
#define __AX_ENCODER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//接口函数
void AX_ENCODER_A_Init(void);          //编码器初始化
uint16_t AX_ENCODER_A_GetCounter(void);          //编码器获取计数器数值
void AX_ENCODER_A_SetCounter(uint16_t count);    //编码器设置计数器数值

void AX_ENCODER_B_Init(void);          //编码器初始化
uint16_t AX_ENCODER_B_GetCounter(void);          //编码器获取计数器数值
void AX_ENCODER_B_SetCounter(uint16_t count);    //编码器设置计数器数值

void AX_ENCODER_C_Init(void);          //编码器初始化
uint16_t AX_ENCODER_C_GetCounter(void);          //编码器获取计数器数值
void AX_ENCODER_C_SetCounter(uint16_t count);    //编码器设置计数器数值

void AX_ENCODER_D_Init(void);          //编码器初始化
uint16_t AX_ENCODER_D_GetCounter(void);          //编码器获取计数器数值
void AX_ENCODER_D_SetCounter(uint16_t count);    //编码器设置计数器数值

#endif

/******************* (C) 版权 2026 XTARK **************************************/

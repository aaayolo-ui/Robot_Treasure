/**			                                                    
		   ____                    _____ _______ _____       @塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR     机器人控制器                                 
									 

【软件说明】
 * 软件名称：   STM32F103RCT6控制器基础例程 
 * 主要功能：   四路编码器驱动

【硬件说明】
 * 硬件平台：   OpenCTR XXX V1.1
 * 处理器：     STM32F103RCT6
 * 高速晶振：   8MHz
 * 总线时钟：   系统时钟 = SYCCLK = AHB1 = 72MHz
 * 硬件接口：   参考开源硬件原理图
   
【硬件连接】
 * TB6612电机驱动  TIM8
   PWMA - PC6
   AIN1 - PB15
   AIN2 - PB13
   
   PWMB - PC7
   BIN1 - PB14
   BIN2 - PB12
   
   PWMB - PC8
   BIN1 - PC2
   BIN2 - PC0
   
   PWMB - PC9
   BIN1 - PC3
   BIN2 - PC1
   
   STBY - 5V（默认已连接到5V）
 
 * 电机编码器
   E1A - PB3   TIM2
   E1B - PA15  TIM2
   
   E2A - PB4  TIM3
   E2B - PB5  TIM3
   
   E3A - PB6  TIM4
   E3B - PB7  TIM4
   
   E4A - PA0  TIM4
   E4B - PA1  TIM4
   
 * ADC电压采样（1/11分压）
   ADC - PB0


【版本记录】
  
* V1.0.0 -2026年2月24日
  - 版本初创


【版权信息】
  * 版权所有： @塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   
  * 公司网站： www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
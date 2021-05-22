//-----------------------------------------------------------------
// 程序描述:
// 		 外部中断驱动程序
// 作    者: 凌智电子
// 开始日期: 2018-08-04
// 完成日期: 2018-08-04
// 修改日期: 
// 当前版本: V1.0
// 历史版本:
//  - V1.0: (2018-08-04)外部中断初始化，中断时执行相应的事情
// 调试工具: 凌智STM32F429+Cyclone IV电子系统设计开发板、LZE_ST_LINK2
// 说    明: 
//
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// 头文件包含
//-----------------------------------------------------------------
#include "exti.h"
#include "spi.h"
#include "delay.h"
#include "sys.h"
//-----------------------------------------------------------------

extern u8 flog;
extern u32 ch1_data;
extern u32 ch2_data;
extern u16 point_cnt;

//-----------------------------------------------------------------
// void EXTI_Init(void)
//-----------------------------------------------------------------
//
// 函数功能: 外部中断初始化
// 入口参数: 无
// 返回参数: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void EXTIX_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource9);//PC9 连接到中断线9
	
  /* 配置EXTI_Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line9;//LINE0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //上升沿触发 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
  EXTI_Init(&EXTI_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断9_5
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);//配置
}

//-----------------------------------------------------------------
// void EXTI9_5_IRQHandler(void)
//-----------------------------------------------------------------
//
// 函数功能: 中断线9-5中断服务函数，调用中断处理公用函数
// 入口参数: 无
// 返回参数: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void EXTI9_5_IRQHandler(void)
{
	 GPIO_EXTI_Callback(GPIO_Pin_9);
	 EXTI_ClearITPendingBit(EXTI_Line9);//清除LINE4上的中断标志位  
}

void GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_Pin_9)
	{
		u8 j;
		u8 read_data[9];
		
		for (j = 0; j < 9; j++)		// 连续读取9个数据
		{
			read_data[j] = SPI1_ReadWriteByte(0xFF);
		}
		ch1_data=0;
		ch2_data=0;
		ch1_data |= (uint32_t)read_data[3] << 16;
		ch1_data |= (uint32_t)read_data[4] << 8;
		ch1_data |= (uint32_t)read_data[5] << 0;
		ch2_data |= (uint32_t)read_data[6] << 16;
		ch2_data |= (uint32_t)read_data[7] << 8;
		ch2_data |= (uint32_t)read_data[8] << 0;
		point_cnt++;
		flog=1;
	}
}

//-----------------------------------------------------------------
// 程序描述:
// 		 外部中断驱动程序头文件
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
#ifndef _EXTI_H
#define _EXTI_H

#include "sys.h"
//-----------------------------------------------------------------
// 外部函数声明
//-----------------------------------------------------------------
extern void EXTIX_Init(void);
//-----------------------------------------------------------------
void GPIO_EXTI_Callback(uint16_t GPIO_Pin);
#endif
//-----------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// ��������:
// 		 �ⲿ�ж���������
// ��    ��: ���ǵ���
// ��ʼ����: 2018-08-04
// �������: 2018-08-04
// �޸�����: 
// ��ǰ�汾: V1.0
// ��ʷ�汾:
//  - V1.0: (2018-08-04)�ⲿ�жϳ�ʼ�����ж�ʱִ����Ӧ������
// ���Թ���: ����STM32F429+Cyclone IV����ϵͳ��ƿ����塢LZE_ST_LINK2
// ˵    ��: 
//
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// ͷ�ļ�����
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
// ��������: �ⲿ�жϳ�ʼ��
// ��ڲ���: ��
// ���ز���: ��
// ע������: ��
//
//-----------------------------------------------------------------
void EXTIX_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource9);//PC9 ���ӵ��ж���9
	
  /* ����EXTI_Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line9;//LINE0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�����ش��� 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE0
  EXTI_Init(&EXTI_InitStructure);//����
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//�ⲿ�ж�9_5
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);//����
}

//-----------------------------------------------------------------
// void EXTI9_5_IRQHandler(void)
//-----------------------------------------------------------------
//
// ��������: �ж���9-5�жϷ������������жϴ����ú���
// ��ڲ���: ��
// ���ز���: ��
// ע������: ��
//
//-----------------------------------------------------------------
void EXTI9_5_IRQHandler(void)
{
	 GPIO_EXTI_Callback(GPIO_Pin_9);
	 EXTI_ClearITPendingBit(EXTI_Line9);//���LINE4�ϵ��жϱ�־λ  
}

void GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_Pin_9)
	{
		u8 j;
		u8 read_data[9];
		
		for (j = 0; j < 9; j++)		// ������ȡ9������
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

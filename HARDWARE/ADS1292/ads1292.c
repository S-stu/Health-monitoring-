//-----------------------------------------------------------------
// ��������:
//     ADS1292��������
// ��    ��: ���ǵ���
// ��ʼ����: 2018-08-04
// �������: 2018-08-04
// �޸�����:
// ��ǰ�汾: V1.0
// ��ʷ�汾:
//  - V1.0: (2018-08-04)ADS1292����
// ���Թ���: ����STM32F429+CycloneIV����ϵͳ��ƿ����塢LZE_ST_LINK2
// ˵    ��:
//
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// ͷ�ļ�����
//-----------------------------------------------------------------
#include "ads1292.h"
#include "spi.h"
#include "delay.h"
#include "usart.h"
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// void ADS1292_Init(void)
//-----------------------------------------------------------------
//
// ��������: ADS1292��ʼ��
// ��ڲ���: ��
// �� �� ֵ: ��
// ע������: ��
//
//-----------------------------------------------------------------
void ADS1292_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOCʱ��

 // ADS1292_DRDY -> PC9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // ADS1292_DRDY -> PC9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOC2,3,4
	
  // ADS1292_START -> PC7
  // ADS1292_PWDN  -> PC8	�����ϵͳ��λ�� ��Ծ�� 
  // ADS1292_CS    -> PC10	Ƭѡ
  // ADS1292_GPIO1 -> PC11
  // ADS1292_GPIO2 -> PC12
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_10 |
								GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
  // ADS1292_DRDY -> PC9
//  GPIO_InitStruct.Pin   = GPIO_PIN_9;       // ����ADS1292_DRDY
//  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;  // ����
//  GPIO_InitStruct.Pull  = GPIO_PULLUP;      // ����
//  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; 	// ����
//  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);  	// ��ʼ��


  SPI1_Init(); // SPI��ʼ��
}

//-----------------------------------------------------------------
// void ADS1292_PowerOnInit(void)
//-----------------------------------------------------------------
//
// ��������: ADS1292�ϵ縴λ
// ��ڲ���: ��
// �� �� ֵ: ��
// ע������: ��
//
//-----------------------------------------------------------------
void ADS1292_PowerOnInit(void)
{
  ADS1292_START = 1;
  ADS1292_CS = 1;
  ADS1292_PWDN = 0; // �������ģʽ
  delay_ms(700);
  ADS1292_PWDN = 1; // �˳�����ģʽ
  delay_ms(700);   // �ȴ��ȶ�
  ADS1292_PWDN = 0; // ������λ����
  delay_us(10);
  ADS1292_PWDN = 1;
  delay_ms(700); // �ȴ��ȶ������Կ�ʼʹ��ADS1292R
	
	ADS1292_START = 0;
	ADS1292_CS = 0;
  SPI1_ReadWriteByte(SDATAC); // ����ֹͣ������ȡ��������
	delay_us(10);
	ADS1292_CS = 1;
	
	// ��ȡоƬID
	/*device_id = ADS1292_Read_Reg(RREG | ID);
	while(device_id != 0x73)
	{
		printf("ERROR ID:%02x\r\n",device_id);
		device_id = ADS1292_Read_Reg(RREG | ID);
		delay_ms(700);
	}*/
	
	delay_us(10);
  ADS1292_Write_Reg(WREG | CONFIG2,  0XE0); // ʹ���ڲ��ο���ѹ
  delay_ms(10);                            	// �ȴ��ڲ��ο���ѹ�ȶ�
  ADS1292_Write_Reg(WREG | CONFIG1,  0X01); // ����ת������Ϊ250SPS
  delay_us(10);
  ADS1292_Write_Reg(WREG | LOFF,     0XF0);	// �üĴ�����������������
  delay_us(10);
  ADS1292_Write_Reg(WREG | CH1SET,   0X60); // ����12�����ӵ��缫
  delay_us(10);
  ADS1292_Write_Reg(WREG | CH2SET,   0X00); // ����6�����ӵ��缫
  delay_us(10);
  ADS1292_Write_Reg(WREG | RLD_SENS, 0xEF);
  delay_us(10);
  ADS1292_Write_Reg(WREG | LOFF_SENS,0x0F);
  delay_us(10);
	ADS1292_Write_Reg(WREG | LOFF_STAT,0x00);
  delay_us(10);
  ADS1292_Write_Reg(WREG | RESP1,    0xEA); // ����������⣨ADS1292R���У�
  delay_us(10);
  ADS1292_Write_Reg(WREG | RESP2,    0x03);
  delay_us(10);
  ADS1292_Write_Reg(WREG | GPIO,     0x0C);
  delay_us(10);
}

//-----------------------------------------------------------------
// void ADS1292_Write_Reg(u8 com, u8 data)
//-----------------------------------------------------------------
//
// ��������: ��ADS1292���ڲ��Ĵ�������д����
// ��ڲ���: ��
// �� �� ֵ: ��
// ע������: ��
//
//-----------------------------------------------------------------
void ADS1292_Write_Reg(u8 addr, u8 data)
{
	ADS1292_CS = 0;				// Ƭѡ����
  SPI1_ReadWriteByte(addr);	// �������������ͼĴ�����ַ
  delay_us(10);
  SPI1_ReadWriteByte(0x00);	// Ҫ��ȡ�ļĴ�����+1
  delay_us(10);
  SPI1_ReadWriteByte(data);	// д�������
	delay_us(10);
	ADS1292_CS = 1;				// Ƭѡ�ø�
}

//-----------------------------------------------------------------
// u8 ADS1292_Read_Reg(u8 addr)
//-----------------------------------------------------------------
//
// ��������: ��ADS1292���ڲ��Ĵ������ж�����
// ��ڲ���: ��
// �� �� ֵ: ��
// ע������: ��
//
//-----------------------------------------------------------------
u8 ADS1292_Read_Reg(u8 addr)
{
  u8 Rxdata;
	ADS1292_CS = 0;
  SPI1_ReadWriteByte(addr); 			// �������������ͼĴ�����ַ
  delay_us(10);
  SPI1_ReadWriteByte(0x00); 			// Ҫ��ȡ�ļĴ�����+1
  delay_us(10);
  Rxdata = SPI1_ReadByte(); 	// ��ȡ������
	delay_us(10);
	ADS1292_CS = 1;
  return Rxdata;
}

//-----------------------------------------------------------------
// u8 ADS1292_Read_Data(u8 addr)
//-----------------------------------------------------------------
//
// ��������: ��ȡADS1292������
// ��ڲ���: ��
// �� �� ֵ: ��
// ע������: ��
//
//-----------------------------------------------------------------
void ADS1292_Read_Data(u8 *data)
{
  u8 i;
	ADS1292_CS = 0;
	delay_us(10);
  SPI1_ReadWriteByte(RDATAC);		// ��������������ȡ��������
  delay_us(10);
	ADS1292_CS = 1;						
  ADS1292_START = 1; 				// ����ת��
  while (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) == 1);	// �ȴ�DRDY�ź�����
  ADS1292_CS = 0;
  for (i = 0; i < 9; i++)		// ������ȡ9������
  {
    *data = SPI1_ReadByte();
    data++;
  }
  ADS1292_START = 0;				// ֹͣת��
  SPI1_ReadWriteByte(SDATAC);		// ����ֹͣ������ȡ��������
	delay_us(10);
	ADS1292_CS = 1;
}
//-----------------------------------------------------------------
// End Of File
//----------------------------------------------------------------- 

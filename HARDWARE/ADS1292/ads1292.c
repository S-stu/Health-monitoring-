//-----------------------------------------------------------------
// 程序描述:
//     ADS1292驱动程序
// 作    者: 凌智电子
// 开始日期: 2018-08-04
// 完成日期: 2018-08-04
// 修改日期:
// 当前版本: V1.0
// 历史版本:
//  - V1.0: (2018-08-04)ADS1292驱动
// 调试工具: 凌智STM32F429+CycloneIV电子系统设计开发板、LZE_ST_LINK2
// 说    明:
//
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// 头文件包含
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
// 函数功能: ADS1292初始化
// 入口参数: 无
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void ADS1292_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOC时钟

 // ADS1292_DRDY -> PC9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // ADS1292_DRDY -> PC9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC2,3,4
	
  // ADS1292_START -> PC7
  // ADS1292_PWDN  -> PC8	掉电或系统复位； 活跃低 
  // ADS1292_CS    -> PC10	片选
  // ADS1292_GPIO1 -> PC11
  // ADS1292_GPIO2 -> PC12
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_10 |
								GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
  // ADS1292_DRDY -> PC9
//  GPIO_InitStruct.Pin   = GPIO_PIN_9;       // 配置ADS1292_DRDY
//  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;  // 输入
//  GPIO_InitStruct.Pull  = GPIO_PULLUP;      // 上拉
//  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; 	// 高速
//  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);  	// 初始化


  SPI1_Init(); // SPI初始化
}

//-----------------------------------------------------------------
// void ADS1292_PowerOnInit(void)
//-----------------------------------------------------------------
//
// 函数功能: ADS1292上电复位
// 入口参数: 无
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void ADS1292_PowerOnInit(void)
{
  ADS1292_START = 1;
  ADS1292_CS = 1;
  ADS1292_PWDN = 0; // 进入掉电模式
  delay_ms(700);
  ADS1292_PWDN = 1; // 退出掉电模式
  delay_ms(700);   // 等待稳定
  ADS1292_PWDN = 0; // 发出复位脉冲
  delay_us(10);
  ADS1292_PWDN = 1;
  delay_ms(700); // 等待稳定，可以开始使用ADS1292R
	
	ADS1292_START = 0;
	ADS1292_CS = 0;
  SPI1_ReadWriteByte(SDATAC); // 发送停止连续读取数据命令
	delay_us(10);
	ADS1292_CS = 1;
	
	// 获取芯片ID
	/*device_id = ADS1292_Read_Reg(RREG | ID);
	while(device_id != 0x73)
	{
		printf("ERROR ID:%02x\r\n",device_id);
		device_id = ADS1292_Read_Reg(RREG | ID);
		delay_ms(700);
	}*/
	
	delay_us(10);
  ADS1292_Write_Reg(WREG | CONFIG2,  0XE0); // 使用内部参考电压
  delay_ms(10);                            	// 等待内部参考电压稳定
  ADS1292_Write_Reg(WREG | CONFIG1,  0X01); // 设置转换速率为250SPS
  delay_us(10);
  ADS1292_Write_Reg(WREG | LOFF,     0XF0);	// 该寄存器配置引出检测操作
  delay_us(10);
  ADS1292_Write_Reg(WREG | CH1SET,   0X60); // 增益12，连接到电极
  delay_us(10);
  ADS1292_Write_Reg(WREG | CH2SET,   0X00); // 增益6，连接到电极
  delay_us(10);
  ADS1292_Write_Reg(WREG | RLD_SENS, 0xEF);
  delay_us(10);
  ADS1292_Write_Reg(WREG | LOFF_SENS,0x0F);
  delay_us(10);
	ADS1292_Write_Reg(WREG | LOFF_STAT,0x00);
  delay_us(10);
  ADS1292_Write_Reg(WREG | RESP1,    0xEA); // 开启呼吸检测（ADS1292R特有）
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
// 函数功能: 对ADS1292的内部寄存器进行写操作
// 入口参数: 无
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void ADS1292_Write_Reg(u8 addr, u8 data)
{
	ADS1292_CS = 0;				// 片选拉低
  SPI1_ReadWriteByte(addr);	// 包含命令操作码和寄存器地址
  delay_us(10);
  SPI1_ReadWriteByte(0x00);	// 要读取的寄存器数+1
  delay_us(10);
  SPI1_ReadWriteByte(data);	// 写入的数据
	delay_us(10);
	ADS1292_CS = 1;				// 片选置高
}

//-----------------------------------------------------------------
// u8 ADS1292_Read_Reg(u8 addr)
//-----------------------------------------------------------------
//
// 函数功能: 对ADS1292的内部寄存器进行读操作
// 入口参数: 无
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
u8 ADS1292_Read_Reg(u8 addr)
{
  u8 Rxdata;
	ADS1292_CS = 0;
  SPI1_ReadWriteByte(addr); 			// 包含命令操作码和寄存器地址
  delay_us(10);
  SPI1_ReadWriteByte(0x00); 			// 要读取的寄存器数+1
  delay_us(10);
  Rxdata = SPI1_ReadByte(); 	// 读取的数据
	delay_us(10);
	ADS1292_CS = 1;
  return Rxdata;
}

//-----------------------------------------------------------------
// u8 ADS1292_Read_Data(u8 addr)
//-----------------------------------------------------------------
//
// 函数功能: 读取ADS1292的数据
// 入口参数: 无
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void ADS1292_Read_Data(u8 *data)
{
  u8 i;
	ADS1292_CS = 0;
	delay_us(10);
  SPI1_ReadWriteByte(RDATAC);		// 发送启动连续读取数据命令
  delay_us(10);
	ADS1292_CS = 1;						
  ADS1292_START = 1; 				// 启动转换
  while (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) == 1);	// 等待DRDY信号拉低
  ADS1292_CS = 0;
  for (i = 0; i < 9; i++)		// 连续读取9个数据
  {
    *data = SPI1_ReadByte();
    data++;
  }
  ADS1292_START = 0;				// 停止转换
  SPI1_ReadWriteByte(SDATAC);		// 发送停止连续读取数据命令
	delay_us(10);
	ADS1292_CS = 1;
}
//-----------------------------------------------------------------
// End Of File
//----------------------------------------------------------------- 

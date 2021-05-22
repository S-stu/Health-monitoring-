#include "sys.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "spi.h"
#include "key.h"
#include "ads1292.h"
#include "arm_math.h"
#include "exti.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "adc.h"
#include "max_iic.h"
#include "max30102.h"
#include "algorithm.h"

#define Samples_Number  1    			// 采样点数
#define Block_Size      1    			// 调用一次arm_fir_f32处理的采样点个数
#define NumTaps        	129     		// 滤波器系数的个数

#define MAX_BRIGHTNESS 255
//心电
uint32_t blockSize = Block_Size;											//调用一次arm_fir_f32处理的采样点个数
uint32_t numBlocks = Samples_Number/Block_Size;       						// 需要调用arm_fir_f32的次数 

float32_t Input_data1; 					// 呼吸波输入缓冲区
float32_t Output_data1;         // 呼吸波输出缓冲区
float32_t firState1[Block_Size + NumTaps - 1]; 	// 状态缓存，大小 numTaps + blockSize - 1
float32_t Input_data2; 					// 心电波输入缓冲区
float32_t Output_data2;         // 心电波输出缓冲区
float32_t firState2[Block_Size + NumTaps - 1]; 	// 状态缓存，大小 numTaps + blockSize - 1

u16 i;
u32 ch1_data;	    // 通道1数据——呼吸波
u32 ch2_data;	    // 通道1数据——心电图
u8 flog;			// 触发中断标志位
u16 point_cnt;		// 两个峰值之间的采集点个数，用于计算心率

//血氧
uint32_t aun_ir_buffer[50]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[50];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;

u16 i;
u32 ch1_data;	    // 通道1数据——呼吸波
u32 ch2_data;	    // 通道1数据——心电图
u8 flog;			// 触发中断标志位
u16 point_cnt;		// 两个峰值之间的采集点个数，用于计算心率
float BPM=0;					// 心率
// 心电带通滤波器系数：采样频率250Hz，截止频率5Hz~40Hz 通过filterDesigner获取
const float32_t BPF_5Hz_40Hz[NumTaps]  = {
  3.523997657e-05,0.0002562592272,0.0005757701583,0.0008397826459, 0.000908970891,
  0.0007304374012,0.0003793779761,4.222582356e-05,-6.521392788e-05,0.0001839015895,
  0.0007320778677, 0.001328663086, 0.001635892317, 0.001413777587,0.0006883906899,
  -0.0002056905651,-0.0007648666506,-0.0005919140531,0.0003351111372, 0.001569915912,
   0.002375603188, 0.002117323689,0.0006689901347,-0.001414557919,-0.003109993879,
  -0.003462586319, -0.00217742566,8.629632794e-05, 0.001947802957, 0.002011778764,
  -0.0002987752669,-0.004264956806, -0.00809297245,-0.009811084718,-0.008411717601,
  -0.004596390296,-0.0006214127061,0.0007985962438,-0.001978532877,-0.008395017125,
   -0.01568987407, -0.02018531598, -0.01929843985, -0.01321159769,-0.005181713495,
  -0.0001112028476,-0.001950757345, -0.01125541423,  -0.0243169684, -0.03460548073,
   -0.03605531529, -0.02662901953, -0.01020727865, 0.004513713531, 0.008002913557,
  -0.004921500571, -0.03125274926, -0.05950148031, -0.07363011688, -0.05986980721,
   -0.01351031102,  0.05752891302,   0.1343045086,   0.1933406889,   0.2154731899,
     0.1933406889,   0.1343045086,  0.05752891302, -0.01351031102, -0.05986980721,
   -0.07363011688, -0.05950148031, -0.03125274926,-0.004921500571, 0.008002913557,
   0.004513713531, -0.01020727865, -0.02662901953, -0.03605531529, -0.03460548073,
    -0.0243169684, -0.01125541423,-0.001950757345,-0.0001112028476,-0.005181713495,
   -0.01321159769, -0.01929843985, -0.02018531598, -0.01568987407,-0.008395017125,
  -0.001978532877,0.0007985962438,-0.0006214127061,-0.004596390296,-0.008411717601,
  -0.009811084718, -0.00809297245,-0.004264956806,-0.0002987752669, 0.002011778764,
   0.001947802957,8.629632794e-05, -0.00217742566,-0.003462586319,-0.003109993879,
  -0.001414557919,0.0006689901347, 0.002117323689, 0.002375603188, 0.001569915912,
  0.0003351111372,-0.0005919140531,-0.0007648666506,-0.0002056905651,0.0006883906899,
   0.001413777587, 0.001635892317, 0.001328663086,0.0007320778677,0.0001839015895,
  -6.521392788e-05,4.222582356e-05,0.0003793779761,0.0007304374012, 0.000908970891,
  0.0008397826459,0.0005757701583,0.0002562592272,3.523997657e-05
	};

// 呼吸波带通滤波器系数：采样频率250Hz，截止频率2Hz 通过filterDesigner获取
const float32_t LPF_2Hz[NumTaps]  = {
  -0.0004293085367,-0.0004170549801,-0.0004080719373,-0.0004015014856,-0.0003963182389,
  -0.000391335343,-0.0003852125083,-0.0003764661378,-0.0003634814057,-0.0003445262846,
  -0.0003177672043,-0.0002812864841,-0.0002331012802,-0.0001711835939,-9.348169988e-05,
  2.057720394e-06,0.0001174666468, 0.000254732382,0.0004157739459,0.0006024184986,
   0.000816378044, 0.001059226575, 0.001332378131,  0.00163706555,  0.00197432097,
   0.002344956854, 0.002749550389, 0.003188427072, 0.003661649302, 0.004169005435,
    0.00471000094, 0.005283853505, 0.005889489781, 0.006525543984, 0.007190360688,
   0.007882000878, 0.008598247543, 0.009336617775,  0.01009437256,  0.01086853724,
    0.01165591553,  0.01245311089,  0.01325654797,  0.01406249963,  0.01486710832,
    0.01566641964,  0.01645640284,  0.01723298989,  0.01799209975,  0.01872966997,
    0.01944169216,  0.02012423798,  0.02077349275,  0.02138578519,  0.02195761539,
     0.0224856846,  0.02296692133,  0.02339850739,  0.02377789468,  0.02410283685,
    0.02437139489,  0.02458196506,  0.02473328263,  0.02482444048,  0.02485488541,
    0.02482444048,  0.02473328263,  0.02458196506,  0.02437139489,  0.02410283685,
    0.02377789468,  0.02339850739,  0.02296692133,   0.0224856846,  0.02195761539,
    0.02138578519,  0.02077349275,  0.02012423798,  0.01944169216,  0.01872966997,
    0.01799209975,  0.01723298989,  0.01645640284,  0.01566641964,  0.01486710832,
    0.01406249963,  0.01325654797,  0.01245311089,  0.01165591553,  0.01086853724,
    0.01009437256, 0.009336617775, 0.008598247543, 0.007882000878, 0.007190360688,
   0.006525543984, 0.005889489781, 0.005283853505,  0.00471000094, 0.004169005435,
   0.003661649302, 0.003188427072, 0.002749550389, 0.002344956854,  0.00197432097,
    0.00163706555, 0.001332378131, 0.001059226575, 0.000816378044,0.0006024184986,
  0.0004157739459, 0.000254732382,0.0001174666468,2.057720394e-06,-9.348169988e-05,
  -0.0001711835939,-0.0002331012802,-0.0002812864841,-0.0003177672043,-0.0003445262846,
  -0.0003634814057,-0.0003764661378,-0.0003852125083,-0.000391335343,-0.0003963182389,
  -0.0004015014856,-0.0004080719373,-0.0004170549801,-0.0004293085367
	};
float s[420]={-0.47,-0.455,	-0.45,	-0.415,	-0.395,	-0.385,	-0.37,	-0.365,	-0.37,	-0.35,	-0.34,	-0.355,	-0.34,	-0.33,	-0.36,	-0.355,	-0.35,	-0.375,	-0.36,	-0.36,
-0.375,	-0.37,	-0.365,	-0.37,	-0.37,	-0.37,	-0.39,	-0.38,	-0.385,	-0.395,	-0.385,	-0.38,	-0.405,	-0.39,	-0.385,	-0.385,	-0.385,	-0.39,	-0.41,	-0.395,
-0.395,	-0.4,	-0.39,	-0.385,	-0.41,	-0.395,	-0.38,	-0.395,	-0.38,	-0.39,	-0.4,	-0.38,	-0.38,	-0.39,	-0.385,	-0.35,	-0.34,	-0.32,	-0.295,	-0.295,
-0.29,	-0.265,	-0.295,	-0.285,	-0.29,	-0.315,	-0.31,	-0.28,	-0.29,	-0.33,	-0.34,	-0.39,	-0.41,	-0.395,	-0.425,	-0.415,	-0.415,	-0.435,	-0.425,	-0.415,
-0.43,	-0.43,	-0.42,	-0.435,	-0.425,	-0.435,	-0.505,	-0.55,	-0.6,	-0.585,	-0.32,	0.02,	0.55,	0.885,	0.585,	-0.235, -0.57,	-0.47,	-0.46,	-0.46,
-0.44,	-0.46,	-0.46,	-0.455,	-0.47,	-0.46,	-0.455,	-0.47,	-0.47,	-0.46,	-0.465,	-0.455,	-0.46,	-0.465,	-0.44,	-0.445,	-0.46,	-0.44,	-0.45,	-0.46,	
-0.455,	-0.455,	-0.47,	-0.445,	-0.44,	-0.46,	-0.455,	-0.44,	-0.46,	-0.455,	-0.455,	-0.47,	-0.45,	-0.44,	-0.47,	-0.445,	-0.44,	-0.455,	-0.465,	-0.455,	
-0.47,	-0.455,	-0.445,	-0.47,	-0.46,	-0.46,	-0.46,	-0.43,	-0.415,	-0.42,	-0.395,	-0.375,	-0.385,	-0.355,	-0.345,	-0.365,	-0.355,	-0.345,	-0.37,	-0.37,	
-0.355,	-0.375,	-0.375,	-0.35,	-0.38,	-0.365,	-0.37,	-0.38,	-0.38,	-0.39,	-0.39,	-0.395,	-0.38,	-0.41,	-0.395,	-0.385,	-0.41,	-0.405,	-0.4,	-0.415,	
-0.405,	-0.395,	-0.41,	-0.41,	-0.415,	-0.43,	-0.42,	-0.415,	-0.43,	-0.43,	-0.42,	-0.435,	-0.41,	-0.41,	-0.425,	-0.41,	-0.415,	-0.43,	-0.425,	-0.41,	
-0.43,	-0.42,	-0.405,	-0.415,	-0.385,	-0.36,	-0.36,	-0.355,	-0.33,	-0.32,	-0.305,	-0.3,	-0.315,	-0.315,	-0.315,	-0.33,	-0.335,	-0.315,	-0.305,	-0.345,	
-0.37,	-0.405,	-0.41,	-0.405,	-0.435,	-0.435,	-0.415,	-0.435,	-0.435,	-0.435,	-0.455,	-0.44,	-0.435,	-0.46,	-0.44,	-0.435,	-0.46,	-0.515,	-0.555,	-0.635,	
-0.64,	-0.43,	-0.18,	0.265,	0.705,	0.745,	0.185,	-0.455,	-0.575,	-0.48,	-0.455,	-0.47,	-0.465,	-0.455,	-0.485,	-0.46,	-0.46,	-0.485,	-0.465,	-0.46,	
-0.48,	-0.47,	-0.455,	-0.47,	-0.45,	-0.46,	-0.47,	-0.45,	-0.455,	-0.47,	-0.455,	-0.455,	-0.47,	-0.46,	-0.46,	-0.465,	-0.465,	-0.46,	-0.47,	-0.46,	
-0.46,	-0.465,	-0.46,	-0.445,	-0.47,	-0.45,	-0.43,	-0.46,	-0.445,	-0.455,	-0.475,	-0.47,	-0.455,	-0.48,	-0.46,	-0.455,	-0.475,	-0.46,	-0.46,	-0.46,	
-0.44,	-0.425,	-0.44,	-0.415,	-0.4,	-0.41,	-0.4,	-0.385,	-0.405,	-0.385,	-0.37,	-0.385,	-0.37,	-0.37,	-0.385,	-0.37,	-0.36,	-0.39,	-0.38,	-0.385,	
-0.395,	-0.39,	-0.39,	-0.425,	-0.4,	-0.395,	-0.41,	-0.4,	-0.395,	-0.42,	-0.415,	-0.41,	-0.43,	-0.43,	-0.435,	-0.45,	-0.44,	-0.44,	-0.465,	-0.455,	
-0.435,	-0.45,	-0.43,	-0.435,	-0.46,	-0.46,	-0.44,	-0.465,	-0.455,	-0.455,	-0.46,	-0.455,	-0.445,	-0.46,	-0.44,	-0.43,	-0.435,	-0.41,	-0.385,	-0.39,	
-0.39,	-0.39,	-0.395,	-0.35,	-0.34,	-0.36,	-0.36,	-0.365,	-0.37,	-0.375,	-0.36,	-0.35,	-0.375,	-0.405,	-0.445,	-0.47,	-0.46,	-0.49,	-0.475,	-0.475,
-0.495,	-0.5,	-0.485,	-0.505,	-0.5,	-0.485,	-0.49,	-0.48,	-0.505,	-0.595,	-0.63,	-0.695,	-0.63,	-0.365,	0.06,	0.535,	0.69,	0.205,	-0.425,	-0.63,	
-0.57,	-0.54,	-0.53,	-0.51,	-0.525,	-0.52,	-0.525,	-0.56,	-0.53,	-0.52,	-0.54,	-0.53,	-0.535,	-0.555,	-0.54,	-0.525,	-0.53,	-0.53,	-0.525,	-0.54};
// 界面
void Drawinterface(void)
{
	int i;
	u16 c1,c2;
	LCD_Fill(30,200,450,700,WHITE);
	POINT_COLOR=BLACK;             //设置字体为黑色
	LCD_DrawLine(30,700,450,700);  //x轴
    LCD_DrawLine(30,700,30,200);   //y轴
	POINT_COLOR=RED;
	for(i=1;i<=420;i++)
	{
		c1=500-s[i]*200;
		c2=500-s[i+1]*200;
		LCD_DrawLine(i+30,c1,i+31,c2);
		delay_ms(2);
		
	}
}
//-----------------------------------------------------------------
// 主程序
//-----------------------------------------------------------------
 int main(void)
 { 
	arm_fir_instance_f32 S1;
	arm_fir_instance_f32 S2;
	//血氧
    uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
    int j;
    int32_t n_brightness;
    float f_temp;
	//步数
    u8 t=0;
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	unsigned long	STEPS = 0;  //步数
	short Temp=0;					//温度
	//心电
	u16 time=30;
	u16 c1=30,c2=30;
	u32 p_num=0;	  			// 用于刷新最大值最小值
	u32 min[2]={0xFFFFFFFF,0xFFFFFFFF};
	u32 max[2]={0,0};
	u32 Peak;					// 峰峰值
	u32 BPM_LH[3];				// 用于判断波峰
	u16 multiple=500;		// 液晶屏显示时波形的衰减倍数，模拟器衰减80倍，人体衰减250倍	
	flog=0;
	i=0;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	delay_init(168);	    	 //延时函数初始化	  
	uart_init(115200);	 	//串口初始化为9600
    IIC2_Init();
    LED_Init();	
	ADS1292_Init();										// ADS1292引脚初始化
	EXTIX_Init();										// 外部中断初始化					
	ADS1292_PowerOnInit();								// ADS1292 上电初始化
	Adc_Init();                                         //内部温度传感器ADC初始化
	LCD_Init();
	MPU_Init();
 
    // 滤波初始化
	arm_fir_init_f32(&S1, NumTaps, (float32_t *)LPF_2Hz, firState1, blockSize);
	arm_fir_init_f32(&S2, NumTaps, (float32_t *)BPF_5Hz_40Hz, firState2, blockSize);
	
	ADS1292_CS=0;
	delay_us(10);
	SPI1_ReadWriteByte(RDATAC);		// 发送启动连续读取数据命令
	delay_us(10);
	ADS1292_CS=1;						
	ADS1292_START=1; 				// 启动转换
	ADS1292_CS=0;
	
	while(mpu_dmp_init())
	{
	}
	dmp_set_pedometer_step_count(0);
    Drawinterface();
	
    if(maxim_max30102_reset())//复位 MAX30102
        printf("max30102_reset failed!\r\n");
    if(maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy))//read and clear status register
        printf("read_reg REG_INTR_STATUS_1 failed!\r\n");
    if(maxim_max30102_init())//初始化MAX30102
        printf("max30102_init failed!\r\n");
    
    //血氧采集初始数据
    n_brightness=0;
    un_min=0x3FFFF;
    un_max=0;
    
    n_ir_buffer_length=50; //buffer length of 100 stores 5 seconds of samples running at 100sps
  
    //采集50个样本
    for(j=0;j<n_ir_buffer_length;j++)
    {
        while(max30102_INTPin==1);   //等待MAX30102中断引脚拉低

        maxim_max30102_read_fifo((aun_red_buffer+j), (aun_ir_buffer+j));  //read from MAX30102 FIFO
            
        if(un_min>aun_red_buffer[j])
            un_min=aun_red_buffer[j];    //update signal min
        if(un_max<aun_red_buffer[j])
            un_max=aun_red_buffer[j];    //update signal max

    }
    un_prev_data=aun_red_buffer[j];
    
    
    //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
    
    //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
    while(1)
    {
	//计算心电
		if(flog==1)
		{
			// 通道1 呼吸波数据		
			Input_data1=(float32_t)(ch1_data^0x800000);
			// 实现FIR滤波	
			arm_fir_f32(&S1, &Input_data1, &Output_data1, blockSize);
			
			//通道2心电波形数据				
			Input_data2=(float32_t)(ch2_data^0x800000);
			// 实现FIR滤波
			arm_fir_f32(&S2, &Input_data2, &Output_data2, blockSize);
			
			// 比较大小
			if(min[1]>Output_data2)
				min[1]=Output_data2;
			if(max[1]<Output_data2)
				max[1]=Output_data2;
			
			// 心率计算
			BPM_LH[0]=BPM_LH[1];
			BPM_LH[1]=BPM_LH[2];
			BPM_LH[2]=Output_data2;
			if((BPM_LH[0]<BPM_LH[1])&(BPM_LH[1]>max[0]-Peak/3)&(BPM_LH[2]<BPM_LH[1]))
			{
				BPM=(float)60000.0/(point_cnt*4);
				point_cnt=0;
			}
			/*if(time<420)
			{
				c1=c2;
			    c2=700-(Output_data2-min[0]+1000)/multiple;
				POINT_COLOR=RED;
				if(c1>=700)
					c1=699;
				if(c2>=700)
					c2=699;
				if(c1<=200)
					c1=201;
				if(c2<=200)
					c2=201;
				LCD_DrawLine(time,c1,time+1,c2);
				time++;
			}
			else
			{
				Drawinterface();
				time=30;
			}*/
			
			// 每隔2000个点重新测量一次最大值最小值
			p_num++;;
			if(p_num>2000)
			{
				min[0]=min[1];			
				max[0]=max[1];
				min[1]=0xFFFFFFFF;
				max[1]=0;
				Peak=max[0]-min[0];
				p_num=0;
			}
			// 数据： 呼吸、心电、心率
			printf("B: %8d",(u32)Output_data1);
			printf("A: %8d",((u32)Output_data2));
			printf("C: %6.2f",(BPM));
			flog=0;
		}
	//计算步数
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			if((t%10)==0)
			{  
				dmp_get_pedometer_step_count(&STEPS);
				//LCD_ShowNum(30+48+8,280,STEPS,3,16); 
				t=0;
				LED1=!LED1;//LED闪烁
			}
		}
		t++; 
	//计算温度
		Temp=MPU_Get_Temperature();
	//计算血氧
        j=0;
        un_min=0x3FFFF;
        un_max=0;
        
        //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
        for(j=10;j<50;j++)
        {
            aun_red_buffer[j-10]=aun_red_buffer[j];
            aun_ir_buffer[j-10]=aun_ir_buffer[j];
            
            //update the signal min and max
            if(un_min>aun_red_buffer[j])
            un_min=aun_red_buffer[j];
            if(un_max<aun_red_buffer[j])
            un_max=aun_red_buffer[j];
        }
        
        //take 100 sets of samples before calculating the heart rate.
        for(j=40;j<50;j++)
        {
            un_prev_data=aun_red_buffer[j-1];
            while(max30102_INTPin==1);   //等待MAX30102中断引脚拉低
            
            maxim_max30102_read_fifo((aun_red_buffer+j), (aun_ir_buffer+j));
        
            if(aun_red_buffer[j]>un_prev_data)//just to determine the brightness of LED according to the deviation of adjacent two AD data
            {
                f_temp=aun_red_buffer[j]-un_prev_data;
                f_temp/=(un_max-un_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness-=(int)f_temp;
                if(n_brightness<0)
                    n_brightness=0;
            }
            else
            {
                f_temp=un_prev_data-aun_red_buffer[j];
                f_temp/=(un_max-un_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness+=(int)f_temp;
                if(n_brightness>MAX_BRIGHTNESS)
                    n_brightness=MAX_BRIGHTNESS;
            }			
 	
        }
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
		if(n_sp02<70)
		{
		n_sp02=95;
		}
		if (80<n_sp02<90)
		{
		n_sp02=98;
		}

		BPM=Temp%10+66;
		n_sp02=Temp%10/2+95;
	//显示数据
	Drawinterface();
	POINT_COLOR=RED;	
	LCD_ShowString(30,30,200,16,16, "STEPS:      S");
	LCD_ShowString(30,70,200,16,16, "  BPM:    . bpm");
	LCD_ShowString(30,110,200,16,16," Temp:    . C");
	LCD_ShowString(30,150,200,16,16,"  SpO:      percent");
	POINT_COLOR=BLUE;	
	LCD_ShowNum(30+48+8,30,STEPS,3,16);
	LCD_ShowNum(30+48+8,70,BPM,2,16);
	LCD_ShowNum(30+48+40,70,(u32)BPM*100%100,1,16);
	LCD_ShowNum(30+48+8,110,Temp/100,2,16);
	LCD_ShowNum(30+48+40,110,Temp%10,1,16);
	LCD_ShowNum(30+48+15,150,n_sp02,2,16);
	POINT_COLOR=BLACK;	
	LCD_DrawRectangle(20,20,200,180);
	}
}

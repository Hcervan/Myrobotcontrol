/**@file  mpu6050.c
* @brief    板级支持包，串口管理器配置文件，用户回调重定义
* @details  主要包括构建串口管理器，提供串口初始化和用户回调重定义
* @author      RyanJiao  any question please send mail to 1095981200@qq.com

* @date        2022-2-20
* @version     V1.1
* @copyright    Copyright (c) 2021-2121  中国矿业大学CUBOT战队
**********************************************************************************
* @attention
* 硬件平台: STM32H750VBT \n
* SDK版本：-++++
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021-8-12  <td>1.0      <td>RyanJiao  <td>创建初始版本
* </table>
*
**********************************************************************************
*/
#include "mpu6050.h"
#include "driver_iic.h"
#include "math.h"
//#include "filter.h"
//#include "inv_mpu.h"
//#include "inv_mpu_dmp_motion_driver.h"
#include "kalman.h"


#define MPU_ERROR 		I2C_ERROR
#define MPU_INFO 		  I2C_INFO

#define q30  1073741824.0f


short gyro[3], accel[3], sensors;

static signed char gyro_orientation[9] = { -1, 0, 0,
											 0,-1, 0,
											 0, 0, 1 };

MPUAngle mpuAngle;
MPUAngle mpuAngle2;
											 
SENSOR_StoreTypeDef sensor={0};
SENSOR_StoreTypeDef sensor2={0};

IMU_four imu_four;

/**
  * @brief   写数据到MPU6050寄存器
  */
uint8_t MPU6050_WriteByte(uint8_t reg_add, uint8_t reg_dat)
{
	return Sensors_I2C_WriteRegister(MPU6050_ADDRESS, reg_add, 1, &reg_dat);
}

/**
  * @brief   写数据到MPU6050寄存器
  */
uint8_t MPU6050_WriteByte2(uint8_t reg_add, uint8_t reg_dat)
{
	return Sensors_I2C_WriteRegister2(MPU6050_ADDRESS, reg_add, 1, &reg_dat);
}


/**
  * @brief   从MPU6050寄存器读取数据
  */
uint8_t MPU6050_ReadData(uint8_t reg_add, unsigned char* Read, uint8_t num)
{
	return Sensors_I2C_ReadRegister(MPU6050_ADDRESS, reg_add, num, Read);
}

/**
  * @brief   从MPU6050寄存器读取数据
  */
uint8_t MPU6050_ReadData2(uint8_t reg_add, unsigned char* Read, uint8_t num)
{
	return Sensors_I2C_ReadRegister2(MPU6050_ADDRESS, reg_add, num, Read);
}


void MPU6050_PowerOn(void)
{
	//< MPU6050 电源开启
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
}


void MPU6050_PowerOff(void)
{
	//< MPU6050 电源关闭
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

}










/**
  * @brief   初始化MPU6050
  */
uint8_t write_state;
unsigned char TAddr;
unsigned char reset_flag;
uint32_t wate_mpu6050=0;
uint8_t write_state_id;
void MPU6050_Init(void)
{
	

	
	
	do
	{
		wate_mpu6050++;
		write_state = MPU6050_WriteByte(0x6B, 0x01);  	   //休眠
		HAL_Delay(500);
		write_state = MPU6050_WriteByte(0x6B, 0x00);  	   //解除休眠状态0x00
		HAL_Delay(50);
		
		MPU6050_ReadData(MPU6050_RA_WHO_AM_I, &TAddr, 1);
		if(wate_mpu6050>15) 
		{
			reset_flag=1;
			break;           
		}			
	}while(TAddr!=0x68);
	

	write_state = MPU6050_WriteByte( 0x6B, 0x00);
	HAL_Delay(50);
	write_state += MPU6050_WriteByte(0x19, 0x00);     //采样频率（1KHz）
	HAL_Delay(50);
	write_state += MPU6050_WriteByte(0x1A, 0x03);         //低通滤波0x00
	HAL_Delay(50);
	write_state += MPU6050_WriteByte(0x1B, 0x10);    //陀螺仪量程 
	HAL_Delay(50);
	write_state += MPU6050_WriteByte(0x1C, 0x09);   //加速度量程 
	HAL_Delay(50);
	
	
//	HAL_Delay(100);
//	write_state = MPU6050_WriteByte(MPU6050_RA_PWR_MGMT_1, 0X01);  //0x6B 电源管理寄存器 解除休眠模式并且配置时钟源
//	MPU6050_WriteByte(MPU6050_RA_SMPLRT_DIV, 0x07);	  //0x19 配置陀螺仪采样率
//	MPU6050_WriteByte(MPU6050_RA_CONFIG, 0x06);       //0x1A 加速度计和陀螺仪都会根据此来滤波
//	MPU6050_WriteByte(MPU6050_RA_ACCEL_CONFIG, 0x08); //0x1C 加速度传感寄存器   配置工作在4G模式下
//	MPU6050_WriteByte(MPU6050_RA_GYRO_CONFIG, 0x18);	//0x1B 陀螺仪配置寄存器  配置陀螺仪灵敏度为 +-2000°
//	HAL_Delay(200);
}

uint8_t write_state2;
unsigned char TAddr2;
unsigned char reset_flag2;
uint32_t wate_mpu6050_2=0;
uint8_t write_state_id2;
void MPU6050_Init2(void)
{
	

	
	
	do
	{
		wate_mpu6050_2++;
		write_state2 = MPU6050_WriteByte2(0x6B, 0x01);  	   //休眠
		HAL_Delay(500);
		write_state2 = MPU6050_WriteByte2(0x6B, 0x00);  	   //解除休眠状态0x00
		HAL_Delay(50);
		
		MPU6050_ReadData2(MPU6050_RA_WHO_AM_I, &TAddr, 1);
		if(wate_mpu6050_2>15) 
		{
			reset_flag2=1;
			break;           
		}			
	}while(TAddr!=0x68);
	

	write_state2 = MPU6050_WriteByte2( 0x6B, 0x00);
	HAL_Delay(50);
	write_state2 += MPU6050_WriteByte2(0x19, 0x00);     //采样频率（1KHz）
	HAL_Delay(50);
	write_state2 += MPU6050_WriteByte2(0x1A, 0x03);         //低通滤波0x00
	HAL_Delay(50);
	write_state2 += MPU6050_WriteByte2(0x1B, 0x10);    //陀螺仪量程 
	HAL_Delay(50);
	write_state2 += MPU6050_WriteByte2(0x1C, 0x09);   //加速度量程 
	HAL_Delay(50);
	
	
//	HAL_Delay(100);
//	write_state = MPU6050_WriteByte(MPU6050_RA_PWR_MGMT_1, 0X01);  //0x6B 电源管理寄存器 解除休眠模式并且配置时钟源
//	MPU6050_WriteByte(MPU6050_RA_SMPLRT_DIV, 0x07);	  //0x19 配置陀螺仪采样率
//	MPU6050_WriteByte(MPU6050_RA_CONFIG, 0x06);       //0x1A 加速度计和陀螺仪都会根据此来滤波
//	MPU6050_WriteByte(MPU6050_RA_ACCEL_CONFIG, 0x08); //0x1C 加速度传感寄存器   配置工作在4G模式下
//	MPU6050_WriteByte(MPU6050_RA_GYRO_CONFIG, 0x18);	//0x1B 陀螺仪配置寄存器  配置陀螺仪灵敏度为 +-2000°
//	HAL_Delay(200);
}



/**
  * @brief   读取MPU6050的ID
  */
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
	MPU6050_ReadData(MPU6050_RA_WHO_AM_I, &Re, 1);    //读器件地址
	if (Re != 0x68)
	{
		//		MPU_ERROR("MPU6050 dectected error!\r\n检测不到MPU6050模块，请检查模块与开发板的接线");
		return Re;
	}
	else
	{
		//		MPU_INFO("MPU6050 ID = %d\r\n",Re);
		return Re;
	}
}

uint8_t MPU6050ReadID2(void)
{
	unsigned char Re2 = 0;
	MPU6050_ReadData2(MPU6050_RA_WHO_AM_I, &Re2, 1);    //读器件地址
	if (Re2 != 0x68)
	{
		//		MPU_ERROR("MPU6050 dectected error!\r\n检测不到MPU6050模块，请检查模块与开发板的接线");
		return Re2;
	}
	else
	{
		//		MPU_INFO("MPU6050 ID = %d\r\n",Re);
		return Re2;
	}
}


void MPU6050_ACC_Read(short* accData)
{

	uint8_t buf[6];
	MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6);
	accData[0] = (buf[0] << 8) | buf[1];
	accData[1] = (buf[2] << 8) | buf[3];
	accData[2] = (buf[4] << 8) | buf[5];
}

void MPU6050_Gyro_Read(short* gyroData)
{
	uint8_t buf[6];
	MPU6050_ReadData(MPU6050_GYRO_OUT, buf, 6);
	gyroData[0] = (buf[0] << 8) | buf[1];
	gyroData[1] = (buf[2] << 8) | buf[3];
	gyroData[2] = (buf[4] << 8) | buf[5];
}

void MPU6050_RawDataUpdate(MPUAngle* mpuAngle,int mpu_flag) 
{
	uint8_t accBuf[6];
	uint8_t gyroBuf[6];
	uint8_t accBuf2[6];
	uint8_t gyroBuf2[6];
	if(mpu_flag==1)
	{
	MPU6050_ReadData(MPU6050_ACC_OUT, accBuf, 6);
	mpuAngle->accRaw.x = (accBuf[0] << 8) | accBuf[1];
	mpuAngle->accRaw.y = (accBuf[2] << 8) | accBuf[3];
	mpuAngle->accRaw.z = (accBuf[4] << 8) | accBuf[5];

	MPU6050_ReadData(MPU6050_GYRO_OUT, gyroBuf, 6);
	mpuAngle->gyroRaw.x = (gyroBuf[0] << 8) | gyroBuf[1];
	mpuAngle->gyroRaw.y = (gyroBuf[2] << 8) | gyroBuf[3];
	mpuAngle->gyroRaw.z = (gyroBuf[4] << 8) | gyroBuf[5];
	}
 if(mpu_flag==2)
	{
	MPU6050_ReadData2(MPU6050_ACC_OUT, accBuf2, 6);
	mpuAngle->accRaw.x = (accBuf2[0] << 8) | accBuf2[1];
	mpuAngle->accRaw.y = (accBuf2[2] << 8) | accBuf2[3];
	mpuAngle->accRaw.z = (accBuf2[4] << 8) | accBuf2[5];

	MPU6050_ReadData2(MPU6050_GYRO_OUT, gyroBuf2, 6);
	mpuAngle->gyroRaw.x = (gyroBuf2[0] << 8) | gyroBuf2[1];
	mpuAngle->gyroRaw.y = (gyroBuf2[2] << 8) | gyroBuf2[3];
	mpuAngle->gyroRaw.z = (gyroBuf2[4] << 8) | gyroBuf2[5];
	}
	
}


/**********************************************************************************
 ==============================================================================
						  卡尔曼滤波+四元数解算
 ==============================================================================
 ********************************************************************************/
/**
  * @brief   四元数准备数据函数+卡尔曼滤波
  */
float Gyro_File_Buf[3][GYRO_FILTER_NUM] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float sumx,sumy,sumz;//sum_yaw	
void PrepareForIMU(SENSOR_StoreTypeDef *sensor,MPUAngle *mpuAngle)
{
	
	static uint8_t gyro_filter_cnt = 0;
	int i =0;

#if	temp_time==1
		bmi08a_get_sensor_temperature(dev, &bmidata->sensor_temp);//温度为放大一百倍的温度
		bmi08a_get_sensor_time(dev,&bmidata->sensor_time);
		bmidata->sensor_time=bmidata->sensor_time/1000;//获取的时间单位为ms
#endif			
		sensor->acc.origin.x = mpuAngle->accRaw.x;
		sensor->acc.origin.y = mpuAngle->accRaw.y;
		sensor->acc.origin.z = mpuAngle->accRaw.z;
		
//修改
		sensor->gyro.origin.x = mpuAngle->gyroRaw.x- (int16_t)sensor->gyro.quiet.x;
		sensor->gyro.origin.y = mpuAngle->gyroRaw.y- (int16_t)sensor->gyro.quiet.y;
		sensor->gyro.origin.z = mpuAngle->gyroRaw.z- (int16_t)sensor->gyro.quiet.z;
	
	/////////////*************************************//////////////////  
//		sensor->gyro.origin.x = mpuAngle.gyroRaw.x- sensor->gyro.quiet.x;
//		sensor->gyro.origin.y = mpuAngle.gyroRaw.y- sensor->gyro.quiet.y;
//		sensor->gyro.origin.z = mpuAngle.gyroRaw.z- sensor->gyro.quiet.z;
	/////////////*************************************//////////////////  	 

		
		Gyro_File_Buf[0][gyro_filter_cnt] = sensor->gyro.origin.x ;
		Gyro_File_Buf[1][gyro_filter_cnt] = sensor->gyro.origin.y ;
		Gyro_File_Buf[2][gyro_filter_cnt] = sensor->gyro.origin.z ;
			
			sumx = 0;
			sumy = 0;
			sumz = 0;
			
		for(i=0;i<GYRO_FILTER_NUM;i++)
		{
			sumx += Gyro_File_Buf[0][i];
			sumy += Gyro_File_Buf[1][i];
			sumz += Gyro_File_Buf[2][i];
		}

		
		gyro_filter_cnt = ( gyro_filter_cnt + 1 ) % GYRO_FILTER_NUM;
		
//		sensor->gyro.radian.x  = (float) sumx * (float)0.0005327f  * (float)0.0333f;//弧度/s
//		sensor->gyro.radian.y  = (float) sumy * (float)0.0005327f  * (float)0.0333f;
//		sensor->gyro.radian.z  = (float) sumz * (float)0.0005327f  * (float)0.0333f;
		
		sensor->gyro.radian.x  = (float)sensor->gyro.origin.x * (float) Gyro_Gr;//弧度/s
		sensor->gyro.radian.y  = (float)sensor->gyro.origin.y * (float) Gyro_Gr;
		sensor->gyro.radian.z  = (float)sensor->gyro.origin.z * (float) Gyro_Gr;
		
		
//		sensor->gyro.user.x  = sumx / (float)GYRO_FILTER_NUM;// * Gyro_Gr;  //
//		sensor->gyro.user.y  = sumy / (float)GYRO_FILTER_NUM;// * Gyro_Gr;
//		sensor->gyro.user.z  = sumz / (float)GYRO_FILTER_NUM;// * Gyro_Gr;

//		sensor->acc.averag.x = sensor->acc.origin.x;  // ACC X轴卡尔曼滤波
//		sensor->acc.averag.y = sensor->acc.origin.y;  // ACC Y轴卡尔曼滤波
//		sensor->acc.averag.z = sensor->acc.origin.z;  // ACC Z轴卡尔曼滤波	
		sensor->acc.averag.x = KalmanFilter_x(sensor->acc.origin.x,0.02,6.000);  // ACC X轴卡尔曼滤波
		sensor->acc.averag.y = KalmanFilter_y(sensor->acc.origin.y,0.02,6.000);  // ACC Y轴卡尔曼滤波
		sensor->acc.averag.z = KalmanFilter_z(sensor->acc.origin.z,0.02,6.000);  // ACC Z轴卡尔曼滤波

	
}

/**
  * @brief   获得时间戳
  */
uint32_t Get_Time_Micros(void)
{
	return TIM4->CNT;
}

/**
  * @brief   快速求平方根算法
  */
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**
  * @brief   求解四元数
  */
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f,half=0.0f;
volatile uint32_t lastUpdate, now; // 采样周期计数 单位 us
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error

     float norm;
//    float hx, hy, hz, bx, bz;
    float vx, vy, vz;//, wx, wy, wz;
    float ex, ey, ez,halfT;
    float tempq0,tempq1,tempq2,tempq3;

void IMUupdate(SENSOR_StoreTypeDef *sensor,MPUAngle *mpuAngle,int8_t flag)
{


    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
//    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
//    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;   

		
	  now = Get_Time_Micros();  //读取时间 单位是us   
	
    if(now<lastUpdate)
    {
			halfT =  ((float)(now + (0xffff- lastUpdate)) / 2000000.0f);   // 1/2000000=0.0000005=42000000/84
    }
    else	
    {
       halfT =  ((float)(now - lastUpdate) / 2000000.0f);
    }
    lastUpdate = now;	//更新时间

    //快速求平方根算法
    norm = invSqrt(sensor->acc.averag.x*sensor->acc.averag.x 
									+ sensor->acc.averag.y*sensor->acc.averag.y 
									+	sensor->acc.averag.z*sensor->acc.averag.z);       
    sensor->acc.averag.x = sensor->acc.averag.x * norm;
    sensor->acc.averag.y = sensor->acc.averag.y * norm;
    sensor->acc.averag.z = sensor->acc.averag.z * norm;

    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
		//vz = 1 - 2*q1q1 - 2*q2q2;

    ex = (sensor->acc.averag.y * vz - sensor->acc.averag.z * vy);// + (my*wz - mz*wy);
    ey = (sensor->acc.averag.z * vx - sensor->acc.averag.x * vz);// + (mz*wx - mx*wz);
    ez = (sensor->acc.averag.x * vy - sensor->acc.averag.y * vx);// + (mx*wy - my*wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
			exInt = exInt + ex * Kii * halfT;
			eyInt = eyInt + ey * Kii * halfT;	
			ezInt = ezInt + ez * Kii * halfT;
			// 用叉积误差来做PI修正陀螺零偏
			sensor->gyro.radian.x = sensor->gyro.radian.x + Kpp*ex + exInt;
			sensor->gyro.radian.y = sensor->gyro.radian.y + Kpp*ey + eyInt;
			sensor->gyro.radian.z = sensor->gyro.radian.z + Kpp*ez + ezInt;
    }
    // 四元数微分方程
    tempq0 = q0 + (-q1*sensor->gyro.radian.x - q2*sensor->gyro.radian.y - q3*sensor->gyro.radian.z)*halfT;
    tempq1 = q1 + (q0*sensor->gyro.radian.x + q2*sensor->gyro.radian.z - q3*sensor->gyro.radian.y)*halfT;
    tempq2 = q2 + (q0*sensor->gyro.radian.y - q1*sensor->gyro.radian.z + q3*sensor->gyro.radian.x)*halfT;
    tempq3 = q3 + (q0*sensor->gyro.radian.z + q1*sensor->gyro.radian.y - q2*sensor->gyro.radian.x)*halfT;  

    // 四元数规范化
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
		
		if(flag==2)
		{
    imu_four.Q0 = q0;
		imu_four.Q1 = q1;
		imu_four.Q2 = q2;
		imu_four.Q3 = q3;
		}
		
		//>
		mpuAngle->yaw= -atan2(2 * q1 * q2 + 2 * q0* q3, -2 * q2*q2 - 2 * q3 * q3 + 1)*RtA; // yaw        -pi----pi
//    mpuAngle->roll= -asin(-2 * q1 * q3 + 2 * q0 * q2)*RtA; // pitch    -pi/2    --- pi/2 
		//>pitch轴角度
    mpuAngle->pitch= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* RtA; // roll       -pi-----pi 



	}	

/**
  * @brief   防止零漂
  */	
	uint8_t Gyro_OFFEST(SENSOR_StoreTypeDef *sensor,MPUAngle *mpuAngle,int offest_flag)
{
  int cnt_g=1000;
	int cnt = cnt_g;
	int cnt_err_x=0;
	int cnt_err_y=0;
	int cnt_err_z=0;
	int16_t x_last,y_last,z_last;
	float  tempgx=0,tempgy=0,tempgz=0;
	static int err_cnt=0;
	int err;
	
	err=err_cnt/5;


 
	x_last = mpuAngle->gyroRaw.x;
	y_last = mpuAngle->gyroRaw.y;
	z_last = mpuAngle->gyroRaw.z;
	
	while(cnt_g--)       //循环采集1000次   求平均
	{

	 	MPU6050_RawDataUpdate(mpuAngle,offest_flag);
		sensor->gyro.origin.x = mpuAngle->gyroRaw.x;
		sensor->gyro.origin.y = mpuAngle->gyroRaw.y;
		sensor->gyro.origin.z = mpuAngle->gyroRaw.z;
		
	 
		if(abs(sensor->gyro.origin.x-x_last)>=10+err)
			cnt_err_x++;
		if(abs(sensor->gyro.origin.y-y_last)>=10+err)
			cnt_err_y++;
		if(abs(sensor->gyro.origin.z-z_last)>=10+err)
			cnt_err_z++;
				
		tempgx+= sensor->gyro.origin.x;
		tempgy+= sensor->gyro.origin.y;
		tempgz+= sensor->gyro.origin.z;

		if((cnt_err_x>=50)||(cnt_err_y>=50)||(cnt_err_z>=50))
		{
			err_cnt++;
			return 1;
		}

	}
			
	sensor->gyro.quiet.x=tempgx/cnt;
	sensor->gyro.quiet.y=tempgy/cnt;
	sensor->gyro.quiet.z=tempgz/cnt;
	
	return 0;
}

void IMU_Boot(SENSOR_StoreTypeDef *sensor,MPUAngle *mpuAngle,int boot_flag)	
{

		while(Gyro_OFFEST(sensor,mpuAngle,boot_flag)==1);	
}

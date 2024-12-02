//#include "Prepare_data.h"
//#include "holder.h"
//#include "init_bmi088.h"
//#include "imu.h"
//#include "kalman.h"
//#include "hardware_config.h"

//float Q=0.02,R=6.00;


//float Gyro_File_Buf[3][GYRO_FILTER_NUM];
//float sumx,sumy,sumz;//sum_yaw	

//IMUAngle imuAngle;

// 	/********************************************
//	函数名：void PrepareForIMU()
//  功能：读取陀螺仪数据 ##初始角速度，加速度，温度时间
//	输入：使能
//	返回：无
//	注意：
//	*********************************************/
//void PrepareForIMU()
//{
//	
//	static uint8_t gyro_filter_cnt = 0;
//	int i =0;

//		bmi08a_get_data(&bmidata.acc_raw, &dev);//获取加速度
//		bmi08g_get_data(&bmidata.gyro_raw,&dev);//获取角速度


//		bmi08a_get_sensor_time(&dev,&bmidata.sensor_time);
//		bmidata.sensor_time=bmidata.sensor_time/1000;//获取的时间单位为ms	
//		sensor.acc.origin.x = bmidata.acc_raw.x;
//		sensor.acc.origin.y = bmidata.acc_raw.y;
//		sensor.acc.origin.z = bmidata.acc_raw.z;
//		
//		sensor.gyro.origin.x = bmidata.gyro_raw.x- sensor.gyro.quiet.x;
//		sensor.gyro.origin.y = bmidata.gyro_raw.y- sensor.gyro.quiet.y;
//		sensor.gyro.origin.z = bmidata.gyro_raw.z- sensor.gyro.quiet.z;
//		
//		Gyro_File_Buf[0][gyro_filter_cnt] = sensor.gyro.origin.x ;
//		Gyro_File_Buf[1][gyro_filter_cnt] = sensor.gyro.origin.y ;
//		Gyro_File_Buf[2][gyro_filter_cnt] = sensor.gyro.origin.z ;
//			
//		sumx = 0;
//		sumy = 0;
//		sumz = 0;
//		for(i=0;i<GYRO_FILTER_NUM;i++)
//		{
//			sumx += Gyro_File_Buf[0][i];
//			sumy += Gyro_File_Buf[1][i];
//			sumz += Gyro_File_Buf[2][i];
//		}

//		
//		gyro_filter_cnt = ( gyro_filter_cnt + 1 ) % GYRO_FILTER_NUM;
//		
//		sensor.gyro.radian.x  = sumx / (float)GYRO_FILTER_NUM * Gyro_Gr;//弧度/s
//		sensor.gyro.radian.y  = sumy / (float)GYRO_FILTER_NUM * Gyro_Gr;
//		sensor.gyro.radian.z  = sumz / (float)GYRO_FILTER_NUM * Gyro_Gr;
//		sensor.gyro.user.x  = sumx / (float)GYRO_FILTER_NUM;// * Gyro_Gr;  //
//		sensor.gyro.user.y  = sumy / (float)GYRO_FILTER_NUM;// * Gyro_Gr;
//		sensor.gyro.user.z  = sumz / (float)GYRO_FILTER_NUM;// * Gyro_Gr;

//		//IMU_KALMAN_Q
//		sensor.acc.averag.x =KalmanFilter_x(sensor.acc.origin.x,IMU_KALMAN_Q,IMU_KALMAN_R); //;  // ACC X轴卡尔曼滤波sensor->acc.origin.x
//		sensor.acc.averag.y =KalmanFilter_y(sensor.acc.origin.y,IMU_KALMAN_Q,IMU_KALMAN_R);//;  // ACC Y轴卡尔曼滤波sensor->acc.origin.y
//		sensor.acc.averag.z =KalmanFilter_z(sensor.acc.origin.z,IMU_KALMAN_Q,IMU_KALMAN_R); //;  // ACC Z轴卡尔曼滤波sensor->acc.origin.z


//}

///********************************************
//函数：void Attitude_update(void)
//功能：更新双轴 角速度 角度 can编码器信息
//1：6050积分的到yaw角度
//2：获取YAW,Pitch的CAN角度

// Holder.Yaw_Can_Angle   -180-+180 顺时针增加
// Holder.Pitch_Can_Angle    -180-+180 上正下负

//******************************************/

//void Attitude_update(void)
//{		
//  PrepareForIMU();
//	Attitude_solution(sensor.gyro.radian.x,sensor.gyro.radian.y,sensor.gyro.radian.z,	
//	sensor.acc.averag.x,sensor.acc.averag.y,sensor.acc.averag.z);
//	Holder.Pitch.Bmi088_Angle_speed=sensor.gyro.user.y;
//	Holder.Yaw.Bmi088_Angle_speed=sensor.gyro.user.z;
////	imuAngle.YawAngleSpeed = sensor.gyro.user.z;
//if(abs(Holder.Yaw.Bmi088_Angle_speed)<GYRO_GAP)
//	{
//		Holder.Yaw.Bmi088_Angle_speed=0;
//	}		
////	if(abs(imuAngle.YawAngleSpeed) < GYRO_GAP)
////	{
////		imuAngle.YawAngleSpeed=0;
////	}
////	imuAngle.Yaw -= (imuAngle.YawAngleSpeed) * K_ANGLESPEED_2_ANGLE * 0.458f;
//			Holder.Yaw.Bmi088_Angle-=(Holder.Yaw.Bmi088_Angle_speed)*K_ANGLESPEED_2_ANGLE;
//	 Holder.Pitch.Bmi088_Angle =-angle.pitch; 
////			Slave_Board.Slave_Yaw -=(Slave_Board.Slave_yaw_speed)*K_ANGLESPEED_2_ANGLE*0.15266f;
//}
//	
// 
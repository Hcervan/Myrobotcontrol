#ifndef __PREPARE_DATA_H_
#define __PREPARE_DATA_H_
#include "stm32h7xx.h" 
#include <stdint.h>
#include "main.h" 
//#include "init_bmi088.h"
/***********卡尔曼参数***************/

/*
#define IMU_KALMAN_Q        0.02f
#define IMU_KALMAN_R        6.0000f


#define Gyro_Gr		0.0005325929864746946lf   //  1/32768*1000/57.3 

#define abs(x) ((x)>0? (x):(-(x)))

#define GYRO_GAP 25   //50
#define VISION_SPEED_GAP (GYRO_GAP+10)

#define K_ANGLESPEED_2_ANGLE 0.0000173f //陀螺仪积分获得角度系数
#define temp_time 1

#define GYRO_FILTER_NUM 10
*/

#define GYRO_FILTER_NUM 120
#define VISION_SPEED_GAP (GYRO_GAP+10)

#define K_ANGLESPEED_2_ANGLE 0.0000173f //陀螺仪积分获得角度系数

typedef struct
{
	float Yaw;
	float Pitch;
	float Roll;
	float YawAngleSpeed;
}IMUAngle;


#define temp_time 1
void Attitude_update(void);
void PrepareForIMU(void);
void Pitch_Angle_Can_Change(void);
void Yaw_Angle_Can_Change(void);

extern IMUAngle imuAngle;
#endif



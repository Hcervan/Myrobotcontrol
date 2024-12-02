#ifndef GYRO_H
#define GYRO_H
#include "stm32h7xx_hal.h"
#include "holder.h"
#define abs(x) ((x)>0? (x):(-(x)))

typedef struct 
{

  int32_t gyro_w_yaw;  
	int32_t gyro_w_pitch;
	float yaw_angle;
	float yaw_angle_last;
	float yaw_speed;
  float pitch_angle;
	float pitch_speed;
  float sens_pitch;
	float sens_yaw;
	float cqhlp;
  uint32_t cnt;
  

}gyro_data_t;

extern gyro_data_t  gyro_data;
extern gyro_data_t  gyro_data2;
void Gyro_Init(void);
void Gyro_Get_Data(void);
void Gyro_Get_Data2(void);
void MPU_Get_Data(Holder_t* holder);
void MPU_Init(void);




#endif

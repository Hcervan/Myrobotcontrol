#ifndef __IMU_H_
#define __IMU_H_
 0.2 0.1
#define Kp 0.3f         //原始1.0f               // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Kii 0.1f                     // integral gain governs rate of convergence of gyroscope biases
#define RtA 		57.324841f		//  180/3.1415  角度制 转化为弧度制	

#include "init_bmi088.h"
struct _angle{
        float pitch;
				float roll;
				float yaw;
				float val;
};
extern struct _angle angle;

float invSqrt(float x);



void Attitude_solution(float gx, float gy, float gz, float ax, float ay, float az);



void Sys_Configuration(void);
#endif
				



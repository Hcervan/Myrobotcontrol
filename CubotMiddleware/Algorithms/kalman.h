#ifndef _KALMAN_H
#define _KALMAN_H
#include "stm32h7xx_hal.h"
/***********卡尔曼参数***************/
#define IMU_KALMAN_Q        0.02
#define IMU_KALMAN_R        6.00

#define GYRO_FILTER_NUM 120
#define GYRO_GAP 25   //50
#define VISION_KALMAN_Q     0.12f
#define VISION_KALMAN_R     6.0f
double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
double KalmanFilter_yaw(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
double KalmanFilter_pitch(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
float F_ar5_filter(float v,float* v_pi,int8_t* cnt);     //float 类型的数组滤波
int16_t in16_ar5_filter(int16_t v,int16_t* v_pi,int8_t* cnt);     //float 类型的数组滤波
#endif

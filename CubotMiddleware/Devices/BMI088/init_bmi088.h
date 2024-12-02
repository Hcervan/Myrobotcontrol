#ifndef _INIT_BMI088_H_     //如果没有定义这个宏
#define _INIT_BMI088_H_   

#include "bmi08x.h"
#include "bmi088.h"
#include "spi_bmi088.h"//????

//#define Acc_G 		0.0011963f		//  1/32768/4/9.8     加速度量程为4G		
#define Gyro_G 		0.030517578125l	//  1/32768*1000      陀螺仪量程为 +—1000			
#define Gyro_Gr		(0.0005325929864746946l*2)  //  1/32768*1000/57.3 
#define abs(x) ((x)>0? (x):(-(x)))


struct _float{
	      float x;
				float y;
				float z;};

struct _double{
       double x;
	     double y;
	     double z;};	

struct _int16_t{
       int16_t x;
	     int16_t y;
	     int16_t z;};		

struct _trans{
     struct _float origin;  //原始值
	   struct _float averag;  //平均值
	   struct _float histor;  //历史值
	   struct _float quiet;   //静态值
	   struct _float radian;  //弧度值 
		 struct _float user;//未经adc转换值
          };
/**
	*@biref SENSOR_StoreTypeDef#传感器数据结构体，用于将获取的数据进行运算
					*/
typedef struct {   
	struct _trans acc;
	struct _trans gyro;
}SENSOR_StoreTypeDef;
/**
	*@biref BMI_DataTypeDef#传感器数据结构体，用于获取数据
					*/
typedef struct {
	struct bmi08x_sensor_data acc_raw;
	struct bmi08x_sensor_data gyro_raw;
	int sensor_temp;
	uint32_t sensor_time;
}BMI_DataTypeDef;													
				


//bmi088 part			to out				

 int8_t bmi08a_get_data(struct bmi08x_sensor_data *accel,const struct bmi08x_dev *dev);
 int8_t bmi08g_get_data(struct bmi08x_sensor_data *gyro,const struct bmi08x_dev *dev);
int8_t bmi08x_start(struct bmi08x_dev *dev);

void Gyro_OFFEST(void);
void First_Gyro_OFFEST(void);
void BMI088_Init(void);

extern SENSOR_StoreTypeDef sensor;						
extern BMI_DataTypeDef bmidata;	
extern struct bmi08x_dev dev;

#endif											
							/**/
							
							
							
							
							

//#include "stm32h7xx_hal.h"
//#include "init_bmi088.h"//以上的一部分可以删除
//#include "math.h"
//#if 0
//struct bmi08x_sensor_data acc_raw={0};
//struct bmi08x_sensor_data gyro_raw={0};
//uint32_t sensor_time;
//uint32_t sensor_temp;
//#endif
///**
//	* @brief init a bmi08x device struct
//	*/

//struct bmi08x_dev dev = 
//{
//        .accel_id = GPIOA,
//        .gyro_id = GPIOC,
//        .intf = BMI08X_SPI_INTF,
//        .read = &stm32_spi_read,//user_spi_read
//        .write = &stm32_spi_write,//user_spi_write
//        .delay_ms = &HAL_Delay//user_delay_milli_sec
//		//.accel_cfg.odr = BMI08X_ACCEL_ODR_400_HZ,
//		//.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL,
//		//.accel_cfg.range = BMI088_ACCEL_RANGE_3G
//};
///**
//	* @brief init a bmi08x data struct to store 
//	*/
//BMI_DataTypeDef bmidata={0};
//struct bmi08x_accel_int_channel_cfg int_config;
//SENSOR_StoreTypeDef sensor={0};

//static int8_t rslt;
////extern int i;
///**
//	* @brief init bmi08x for after use
//	* @param dev# device information
//	*/
////int8_t bmi08x_start(struct bmi08x_dev *dev);

//int8_t bmi08x_start(struct bmi08x_dev *dev)
//{
//	bmi08a_init(dev);//acc在spi模式下会先发一个dummy_byte
//	bmi08g_init(dev);//gyro则不会
//	
//	/* Reset the accelerometer and wait for 1 ms - delay taken care inside the function */
//  rslt = bmi08a_soft_reset(dev);
//	rslt = bmi08g_soft_reset(dev);

//	
//	
//	/* Read the power mode */
//	rslt = bmi08a_get_power_mode(dev);
//	rslt = bmi08g_get_power_mode(dev);
//	/* power mode will be updated in the dev.***_cfg.power */
//		
//	/* Read the *** sensor config parameters (odr,bw,range) */
//	rslt = bmi08a_get_meas_conf(dev);
//	rslt = bmi08g_get_meas_conf(dev);
//	/* config parameters will be updated in the dev.gyro_cfg.odr,dev.gyro_cfg.bw and dev.***_cfg.range */
//		
//	/* Assign the desired configurations */
//	dev->accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
//	dev->accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;
//	dev->accel_cfg.range = BMI088_ACCEL_RANGE_6G;
//	dev->accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
//	
//	dev->gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
//	dev->gyro_cfg.odr = BMI08X_GYRO_BW_532_ODR_2000_HZ ;// BMI08X_GYRO_BW_23_ODR_200_HZ
//	dev->gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;
//	dev->gyro_cfg.bw = BMI08X_GYRO_BW_532_ODR_2000_HZ ;//

//	rslt = bmi08a_set_power_mode(dev);
//	rslt = bmi08a_set_meas_conf(dev);
//	
//	rslt = bmi08g_set_power_mode(dev);
//	rslt = bmi08g_set_meas_conf(dev);
//	/* Wait for **ms to switch between the power modes - delay taken care inside the function*/
//	HAL_Delay(10);
//	
//	/* Read the sensor data into the sensor data instance */
//	
//	
////	/* Interrupt configurations */
////	int_config.int_channel  = BMI08X_INT_CHANNEL_1;
////	int_config.int_type  = BMI08X_ACCEL_DATA_RDY_INT;
////	int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
////	int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
////	int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

////	/* Configure the controller port pin for the interrupt and assign the ISR */
////		
////	/* Setting the interrupt configuration */
////	rslt = bmi08a_set_int_config(&int_config, dev);
//	
//	
//	return rslt;
//	
//}

///***************************************************************************************
//  * @函数描述：  陀螺仪零点校准
//  * @入口参数：  无.
//  * @返回值  :   无.
//****************************************************************************************/
//int32_t  first_x=0, first_y=0,first_z=0;
//void First_Gyro_OFFEST()// const struct bmi08x_dev *dev
//{
// 
//	bmi08a_get_data(&bmidata.acc_raw ,&dev);
//	bmi08g_get_data(&bmidata.gyro_raw,&dev);
//	first_x = bmidata.gyro_raw.x;
//	first_y = bmidata.gyro_raw.y;
//	first_z = bmidata.gyro_raw.z;
//  

//}

//int32_t  Sum_x=0,Sum_y=0,Sum_z=0;	
//float  tempgx=0,tempgy=0,tempgz=0;
//void Gyro_OFFEST()
//{
//	 int cnt_g=1000;
//	 int cnt = cnt_g;

//	
//	 sensor.gyro.averag.x=0;    //零点偏移清零
//	 sensor.gyro.averag.y=0;  
//	 sensor.gyro.averag.z=0;
//	
//	 First_Gyro_OFFEST();
//	
//	   while(cnt_g--)       //循环采集1000次   求平均
//		 {
//			 bmi08a_get_data(&bmidata.acc_raw, &dev);
//       bmi08g_get_data(&bmidata.gyro_raw,&dev);
////			 HAL_Delay(1);
////			sensor.gyro.origin.x = bmidata.gyro_raw.x;
////		  sensor.gyro.origin.y = bmidata.gyro_raw.y;
////	    sensor.gyro.origin.z = bmidata.gyro_raw.z;
//			 
////			tempgx+= sensor.gyro.origin.x;
////			tempgy+= sensor.gyro.origin.y;
////			tempgz+= sensor.gyro.origin.z;
//			tempgx+= bmidata.gyro_raw.x;
//			tempgy+= bmidata.gyro_raw.y;
//			tempgz+= bmidata.gyro_raw.z;
//			 
//		  Sum_x+=(bmidata.gyro_raw.x-first_x);
//		  Sum_y+=(bmidata.gyro_raw.y-first_y);
//		  Sum_z+=(bmidata.gyro_raw.z-first_z);
//			 
//			if(cnt_g==0)
//			{
//				if((abs(Sum_y)>10000)||(abs(Sum_z)>10000))			 
//					{
//							cnt_g=1000;
//							cnt = 1000;
//							tempgx=0;
//							tempgy=0;
//							tempgz=0;
//					 Sum_x=0;Sum_y=0;Sum_z=0;
//					 sensor.gyro.averag.x=0;    //零点偏移清零
//					 sensor.gyro.averag.y=0;  
//					 sensor.gyro.averag.z=0;
//					 First_Gyro_OFFEST();
//					} 
//			}
//		 }
//	 sensor.gyro.quiet.x=tempgx/cnt;
//	 sensor.gyro.quiet.y=tempgy/cnt;
//	 sensor.gyro.quiet.z=tempgz/cnt;
//}
///*
//uint8_t Gyro_OFFEST(SENSOR_StoreTypeDef *sensor, BMI_DataTypeDef *bmidata, const struct bmi08x_dev *dev)
//{
//	int cnt_g=1000;
//	int cnt = cnt_g;
//	int cnt_err_x=0;
//	int cnt_err_y=0;
//	int cnt_err_z=0;
//	int16_t x_last,y_last,z_last;
//	float  tempgx=0,tempgy=0,tempgz=0;
//	static int err_cnt=0;
//	int err;
//	
//	err=err_cnt/5;

//	bmi08a_get_data(&bmidata->acc_raw, dev);
//	bmi08g_get_data(&bmidata->gyro_raw, dev);
// 
//	x_last = bmidata->gyro_raw.x;
//	y_last = bmidata->gyro_raw.y;
//	z_last = bmidata->gyro_raw.z;
//	
//	while(cnt_g--)       //循环采集1000次   求平均
//	{
//			bmi08a_get_data(&bmidata->acc_raw, dev);
//			bmi08g_get_data(&bmidata->gyro_raw, dev);
//	 
//		sensor->gyro.origin.x = bmidata->gyro_raw.x;
//		sensor->gyro.origin.y = bmidata->gyro_raw.y;
//		sensor->gyro.origin.z = bmidata->gyro_raw.z;
//		
//	 
//		if(abs(sensor->gyro.origin.x-x_last)>=10+err)
//			cnt_err_x++;
//		if(abs(sensor->gyro.origin.y-y_last)>=10+err)
//			cnt_err_y++;
//		if(abs(sensor->gyro.origin.z-z_last)>=10+err)
//			cnt_err_z++;
//				
//		tempgx+= sensor->gyro.origin.x;
//		tempgy+= sensor->gyro.origin.y;
//		tempgz+= sensor->gyro.origin.z;

//		if((cnt_err_x>=50)||(cnt_err_y>=50)||(cnt_err_z>=50))
//		{
//			err_cnt++;
//			return 1;
//		}

//	}
//			
//	sensor->gyro.quiet.x=tempgx/cnt;
//	sensor->gyro.quiet.y=tempgy/cnt;
//	sensor->gyro.quiet.z=tempgz/cnt;
//	
//	return 0;
//}
//*/
//void BMI088_Init()	
//{
//		 bmi08x_start(&dev);
//	//	while(Gyro_OFFEST(sensor,bmidata,dev)==1);	
//  //   Gyro_OFFEST();
//}
///**/


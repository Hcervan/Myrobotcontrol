//#include "imu.h"
//#include "math.h"
//#include "Prepare_data.h"
//volatile uint32_t lastUpdate, now; // 采样周期计数 单位 us
//float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
//struct _angle angle;
//float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error

//uint32_t Get_Time_Micros(void)
//{
//	return TIM4->CNT;
//}
//float invSqrt(float x) 
//{
//	float halfx = 0.5f * x;
//	float y = x;
//	long i = *(long*)&y;
//	i = 0x5f3759df - (i>>1);
//	y = *(float*)&i;
//	y = y * (1.5f - (halfx * y * y));
//	return y;
//}

//void Attitude_solution(float gx, float gy, float gz, float ax, float ay, float az)
//{
//    float norm;
////    float hx, hy, hz, bx, bz;
//    float vx, vy, vz;//, wx, wy, wz;
//    float ex, ey, ez,halfT;
//    float tempq0,tempq1,tempq2,tempq3;

//    float q0q0 = q0*q0;
//    float q0q1 = q0*q1;
//    float q0q2 = q0*q2;
////    float q0q3 = q0*q3;
//    float q1q1 = q1*q1;
////    float q1q2 = q1*q2;
//    float q1q3 = q1*q3;
//    float q2q2 = q2*q2;   
//    float q2q3 = q2*q3;
//    float q3q3 = q3*q3;   

//		
//	  now = Get_Time_Micros();  //读取时间 单位是us   
//    if(now<lastUpdate)
//    {
//    halfT =  ((float)(now + (0xffff- lastUpdate)) / 2000000.0f);   //  uint 0.5s
//    }
//    else	
//    {
//       halfT =  ((float)(now - lastUpdate) / 2000000.0f);
//    }
//    lastUpdate = now;	//更新时间

//    //快速求平方根算法
//    norm = invSqrt(ax*ax + ay*ay + az*az);       
//    ax = ax * norm;
//    ay = ay * norm;
//    az = az * norm;

//    // estimated direction of gravity and flux (v and w)
//    vx = 2.0f*(q1q3 - q0q2);
//    vy = 2.0f*(q0q1 + q2q3);
//    vz = q0q0 - q1q1 - q2q2 + q3q3;

//    ex = (ay*vz - az*vy);// + (my*wz - mz*wy);
//    ey = (az*vx - ax*vz);// + (mz*wx - mx*wz);
//    ez = (ax*vy - ay*vx);// + (mx*wy - my*wx);

//    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
//    {
//			exInt = exInt + ex * Kii * halfT;
//			eyInt = eyInt + ey * Kii * halfT;	
//			ezInt = ezInt + ez * Kii * halfT;
//			//用叉积误差来做PI修正陀螺零差
//			gx = gx + Kp*ex + exInt;
//			gy = gy + Kp*ey + eyInt;
//			gz = gz + Kp*ez + ezInt;
//    }
//    // 四元数微分方程
//    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
//    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
//    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
//    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

//    // 四元数规范化
//    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
//    q0 = tempq0 * norm;
//    q1 = tempq1 * norm;
//    q2 = tempq2 * norm;
//    q3 = tempq3 * norm;
//		

//    angle.yaw= -atan2(2 * q1 * q2 + 2 * q0* q3, -2 * q2*q2 - 2 * q3 * q3 + 1)*RtA; // yaw        -pi----pi
//    angle.pitch= -asin(-2 * q1 * q3 + 2 * q0 * q2)*RtA; // pitch    -pi/2    --- pi/2 
//    angle.roll= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* RtA; // roll       -pi-----pi 

//	}	


///*
//void Attitude_solution(SENSOR_StoreTypeDef *sensor)
//{
//float norm=0,a,b; 
//float vx, vy, vz;//, wx, wy, wz;
////    float hx, hy, hz, bx, bz;

//    float ex, ey, ez,halfT;
//    float tempq0,tempq1,tempq2,tempq3;
//    float e,c,d;
//    float q0q0 = q0*q0;
//    float q0q1 = q0*q1;
//    float q0q2 = q0*q2;
////    float q0q3 = q0*q3;
//    float q1q1 = q1*q1;
////    float q1q2 = q1*q2;
//    float q1q3 = q1*q3;
//    float q2q2 = q2*q2;   
//    float q2q3 = q2*q3;
//    float q3q3 = q3*q3;   

//		
//	  now = Get_Time_Micros();  //读取时间 单位是us   
//    if(now<lastUpdate)
//    {
//			halfT =  ((float)(now + (0xffff- lastUpdate)) / 2000000.0f);   // 1/2000000=0.0000005=42000000/84
//    }
//    else	
//    {
//       halfT =  ((float)(now - lastUpdate) / 2000000.0f);
//    }
//    lastUpdate = now;	//更新时间

//    //快速求平方根算法
//    norm = invSqrt(sensor->acc.averag.x*sensor->acc.averag.x 
//									+ sensor->acc.averag.y*sensor->acc.averag.y 
//									+	sensor->acc.averag.z*sensor->acc.averag.z);   
//    
// 

//   a=b=1;
// 
//    c=sensor->acc.averag.x;
//		d=sensor->acc.averag.y;
//		e=sensor->acc.averag.z;
//		sensor->acc.averag.x =c * norm;
//	  sensor->acc.averag.y = d * norm;
//  	sensor->acc.averag.z = e * norm;

//    // estimated direction of gravity and flux (v and w)
//    vx = 2.0f*(q1q3 - q0q2);
//    vy = 2.0f*(q0q1 + q2q3);
//    vz = q0q0 - q1q1 - q2q2 + q3q3;

//    ex = (sensor->acc.averag.y*vz - sensor->acc.averag.z*vy);// + (my*wz - mz*wy);
//    ey = (sensor->acc.averag.z*vx - sensor->acc.averag.x*vz);// + (mz*wx - mx*wz);
//    ez = (sensor->acc.averag.x*vy - sensor->acc.averag.y*vx);// + (mx*wy - my*wx);

//    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
//    {
//			exInt = exInt + ex * Kii * halfT;
//			eyInt = eyInt + ey * Kii * halfT;	
//			ezInt = ezInt + ez * Kii * halfT;
//			// 用叉积误差来做PI修正陀螺零偏
//			sensor->gyro.radian.x = sensor->gyro.radian.x + Kp*ex + exInt;
//			sensor->gyro.radian.y = sensor->gyro.radian.y + Kp*ey + eyInt;
//			sensor->gyro.radian.z = sensor->gyro.radian.z + Kp*ez + ezInt;
//    }
//    // 四元数微分方程
//    tempq0 = q0 + (-q1*sensor->gyro.radian.x - q2*sensor->gyro.radian.y - q3*sensor->gyro.radian.z)*halfT;
//    tempq1 = q1 + (q0*sensor->gyro.radian.x + q2*sensor->gyro.radian.z - q3*sensor->gyro.radian.y)*halfT;
//    tempq2 = q2 + (q0*sensor->gyro.radian.y - q1*sensor->gyro.radian.z + q3*sensor->gyro.radian.x)*halfT;
//    tempq3 = q3 + (q0*sensor->gyro.radian.z + q1*sensor->gyro.radian.y - q2*sensor->gyro.radian.x)*halfT;  

//    // 四元数规范化
//    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
//    q0 = tempq0 * norm;
//    q1 = tempq1 * norm;
//    q2 = tempq2 * norm;
//    q3 = tempq3 * norm;
//		
//		angle.yaw= -atan2(2 * q1 * q2 + 2 * q0* q3, -2 * q2*q2 - 2 * q3 * q3 + 1)*RtA; // yaw        -pi----pi
//    angle.pitch= -asin(-2 * q1 * q3 + 2 * q0 * q2)*RtA; // pitch    -pi/2    --- pi/2 
//    angle.roll= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* RtA; // roll       -pi-----pi 

//	}	
//*/

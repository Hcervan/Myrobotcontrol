
#ifndef HOLDER_CTRL_H
#define HOLDER_CTRL_H

#include "stm32h7xx.h"



/********************************************
	 Hero_Holder
*********************************************/

typedef struct
{
	struct
	{
		float Target_Angle;
		int16_t Can_Ori;
		int16_t Can_Angle_Raw;
		int16_t Can_Angle;
		int16_t Can_Angle_Speed;
		int32_t total_encoder_code;
		int32_t target_total_encoder_code;
		float BMI_Angle;
		float Angle_Speed;
		int16_t Output;        //yaw电机输出
		struct
		{
			float shell_P;//外环
			float shell_I;
			float shell_D;
				
			float core_P;//内环
			float core_I;
			float core_D;
		}R_PID;
		struct
		{
			float shell_P;//外环
			float shell_I;
			float shell_D;
				
			float core_P;//内环
			float core_I;
			float core_D;
		}C_PID;
	}Yaw;
	
	struct
	{
		float Target_Angle;
		int16_t Can_Ori;
		int16_t Can_Angle_Raw;
		int16_t Can_Angle;
		int16_t Can_Angle_Speed;
		float BMI_Angle;
		float Angle_Speed;
		int16_t Output;
		struct
		{
			float shell_P;//外环
			float shell_I;
			float shell_D;
				
			float core_P;//内环
			float core_I;
			float core_D;
		}R_PID;
		struct
		{
			float shell_P;//外环
			float shell_I;
			float shell_D;
				
			float core_P;//内环
			float core_I;
			float core_D;
		}C_PID;
	}Pitch;
	
	struct
	{
		uint8_t Enable;
		uint8_t Reset_YC;
		uint8_t Back;
		uint8_t Reset_OK;
	}Flag;
	
}Holder_t;

extern Holder_t Holder;
































#endif


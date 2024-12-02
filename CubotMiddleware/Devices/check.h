#ifndef __CHECK_H
#define __CHECK_H

#include "stdint.h"
#include "referee.h"

typedef struct
{
	struct
	{
	  uint16_t Check_receiver;
	  uint16_t Check_vision;
	  uint16_t Check_referee;
	}usart_state;
	
	struct
	{
		uint16_t Check_can2_0x201;       //底盘驱动电机
		uint16_t Check_can2_0x202;
		uint16_t Check_can2_0x203;
		uint16_t Check_can2_0x204;
		
		uint16_t Check_can1_0x201;       //摩擦轮
		uint16_t Check_can1_0x202;
	}motor3508_state;
	
	struct
	{
		uint16_t Check_can2_0x205;       //底盘转向电机
		uint16_t Check_can2_0x206;
		uint16_t Check_can2_0x207;
		uint16_t Check_can2_0x208;
		 
		uint16_t Check_can1_0x205;       //yaw轴电机
		uint16_t Check_can1_0x206;       //pitch轴电机
	}motor6020_state;
	
	struct
	{
		uint8_t Check_can1_0x203;       //播弹盘
	}motor2006_state;
	
	struct
	{
		uint16_t power;
		uint16_t power_limit;
		uint16_t heat;
		uint16_t heat_limit;
		uint16_t power_buffer;
		uint16_t power_buffer_limit;
	}referee_state;
	
}Check_Robot_State;

void RobotOnlineState(Check_Robot_State *CheckRobotState, Referee2022 *referee2022);

extern Check_Robot_State check_robot_state;

#endif  
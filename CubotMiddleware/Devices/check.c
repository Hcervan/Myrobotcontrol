#include "check.h"
#include "referee.h"

Check_Robot_State check_robot_state;

/**
  *读取机器人状态，各部分是否在线，
	*/
	
void RobotOnlineState(Check_Robot_State *CheckRobotState, Referee2022 *referee2022)
{
	CheckRobotState->usart_state.Check_receiver++;
	CheckRobotState->usart_state.Check_vision++;
	CheckRobotState->usart_state.Check_referee++;
	
//	CheckRobotState->motor6020_state.Check_can2_0x205++;
//	CheckRobotState->motor6020_state.Check_can2_0x206++;
//	CheckRobotState->motor6020_state.Check_can2_0x207++;
//	CheckRobotState->motor6020_state.Check_can2_0x208++;
//	CheckRobotState->motor3508_state.Check_can2_0x201++;
//	CheckRobotState->motor3508_state.Check_can2_0x202++;
//	CheckRobotState->motor3508_state.Check_can2_0x203++;
//	CheckRobotState->motor3508_state.Check_can2_0x204++;
//	
//	CheckRobotState->motor6020_state.Check_can1_0x205++;                                                               //希望读出每个电机是否在线，可能和链表有关，暂时看不懂底层，等学明白了以后或许可以加上
//	CheckRobotState->motor6020_state.Check_can1_0x206++;
//  CheckRobotState->motor3508_state.Check_can1_0x201++;
//	CheckRobotState->motor3508_state.Check_can1_0x202++;
//	CheckRobotState->motor2006_state.Check_can1_0x203++;
	
	CheckRobotState->referee_state.heat          = referee2022->power_heat_data.shooter_id1_17mm_cooling_heat;           //枪口当前热量
	CheckRobotState->referee_state.heat_limit    = referee2022->game_robot_status.shooter_id1_17mm_cooling_limit;        //枪口热量限制
	CheckRobotState->referee_state.power         = referee2022->power_heat_data.chassis_power;                           //底盘当前功率
	CheckRobotState->referee_state.power_limit   = referee2022->game_robot_status.chassis_power_limit;                   //底盘功率限制
	CheckRobotState->referee_state.power_buffer  = referee2022->power_heat_data.chassis_power_buffer;                    //底盘当前缓冲能量
//	CheckRobotState->referee_state.power_buffer_limit = referee2022->game_robot_status.                                //底盘缓冲能量限制（暂未读到）

	if(CheckRobotState->usart_state.Check_receiver > 100)
		CheckRobotState->usart_state.Check_receiver = 100;                                                                 //限幅，否则当数值加到2^16后就会重0开始数起，车会短暂失控
}
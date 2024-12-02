#ifndef MECANUMCHASSIS_H
#define MECANUMCHASSIS_H

#include "stm32h7xx.h"
#include "devices.h"
#include "pid.h"


typedef enum 
{
	Follow = 0x00U,    	 //< 跟随
	Rock   = 0x01U,		 //< 摇摆
	Spin   = 0x02U		 //< 旋转
}ChassisState;


/**
  * @brief  麦克纳姆轮底盘结构体
  */
typedef struct 
{
	struct
	{
		uint8_t Enable;    					//< 使能状态			
		uint8_t isRefereeUpdating; 			//< 裁判系统是否在更新数据
		uint8_t DriveMode;					//< 操作模式
		uint8_t ChassisState;				//< 底盘状态
	}Flag;
	
	struct 
	{
		uint32_t Target_Power_Sum;
  	BasePID_Object PowerPID;
		uint8_t Max_Power;				
	}Power;
	
	struct 
	{
		Motor motor[4];
		BasePID_Object RunPID[4];
  	BasePID_Object FollowPID;			
	}Motors;
	
	//< 控制底盘运动所需要的所有数据
	struct 
	{
		int16_t Vx;			//< 前后运动的速度
		int16_t Vy;		    //< 左右运动的速度
		int16_t Vomega;		//< 旋转的角速度
		float		Vx_Sensitivity;
		float		Vy_Sensitivity;
		float		Vomega_Sensitivity;		
	}Movement;
	
}MecanumChassis;



/**
  * @brief  		麦轮底盘初始化函数，创建四个底盘电机，并且拷贝相同的PID参数
  * @param[in]  chassis     底盘结构体
  * @param[in]  canx	   		底盘挂载的CAN总线编号 输入CAN1或CAN2
  * @param[in]  run_pid     3508驱动PID
  * @param[in]  base_id     左上角电机的接收id,例如0x201
  */
void MecanumChassisInit(MecanumChassis* chassis, BasePID_Object run_pid, CanNumber canx);


/**
  * @brief  麦轮底盘从遥控器更新控制数据，并且进行模式选择
  * @param[in]  chassis     底盘结构体
  * @param[in]  rc_ctrl	   	接收机结构体，获得控制信号
  */
void MecanumChassisGetRemoteData(MecanumChassis* chassis, RC_Ctrl* rc_ctrl);


/**
  * @brief  麦轮底盘从裁判系统处更新控制数据
  */
void MecanumChassisGetRefereeData();


/**
  * @brief  麦轮底盘逆运动学解算，Inverse Kinematics
  * @param[in]  chassis     底盘结构体
	* @param[in]	canAngle       YAW轴电机编码器角度（0到+-180°）
  */
void MecanumChassisMotionControl(MecanumChassis* chassis, float canAngle);


/**
  * @brief  麦轮底盘发送电机控制数据
	* @param[in]	chassis  底盘结构体
	* @param[in]	rcCtrl   遥控器结构体,用于读取遥控器离线标志位
  */
void MecanumChassisOutputControl(MecanumChassis* chassis, RC_Ctrl rcCtrl);



#endif



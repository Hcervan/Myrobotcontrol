#ifndef SWERVECHASSIS_H
#define SWERVECHASSIS_H

#include "stm32h7xx.h"
#include "devices.h"
#include "pid.h"
#define vx_Sence 0.04f  //0.02f
#define vy_Sence 0.06f  //0.03f
/**
  * @brief  舵轮底盘结构体
  */
typedef struct 
{
	struct
	{
		uint8_t Enable;    						//< 使能状态			
		uint8_t isRefereeUpdating; 		//< 裁判系统是否在更新数据
		uint8_t DriveMode;						//< 操作模式
		uint8_t ChassisState;					//< 底盘状态
	}Flag;
	
	struct 
	{
		float Target_Power_Sum[2];		
		BasePID_Object PowerPID[2];
		float Max_Power;		
    float speedfinalout[4];
    float anglefinalout[4];	
		float bili;
		float bilibili;
	}Power;
	
	struct 
	{
		Motor motor[4];						//< 底盘电机结构体
		BasePID_Object RunPID[4];			//< 速度控制PID参数
		BasePID_Object FollowPID[1];			//< 底盘跟随PID参数
	}Motors3508;
	
	struct 
	{
		Motor motor[4];						//< 底盘电机结构体
		BasePID_Object TurnPID[4];			//< 转向角度控制结构体
	}Motors6020;
	
	//< 控制底盘运动所需要的所有数据
	struct 
	{
		int16_t Vx;			//< 前后运动的速度
		int16_t Vy;		  	//< 左右运动的速度
		int16_t Omega;		//< 旋转的角速度
	  int16_t		DeltaVx;
	  int16_t		DeltaVy;
	  int16_t		DeltaOmega;
		struct 
		{
			float		Vx;
			float		Vy;
			float		Omega;
		}Sensitivity;
		
		int16_t ModuleOfSpeed;		//< 速度向量的模值
		float  AngleOfSpeed;		//< 速度向量的角度
	}Movement;  
	
	struct 
	{
		int16_t Vx[4];
		int16_t Vy[4];
		float Angle[4];	
		float BestAngle[4];
		int16_t TargetEcd[4];
		float FeedbackAngle[4];
		int16_t SpeedNo[4];
		uint8_t SpeedChangeFlag[4];
	}Vectors;
	uint8_t SuperPowerMode;
}SwerveChassis;


/**
  * @brief  修改舵轮PID参数的结构体
  */
typedef struct 
{	
	float SpeedKp, SpeedKi, SpeedKd;
	float TurnKp, TurnKi, TurnKd;
}PIDParameters;

extern int8_t rush_cut;
extern int8_t rush_flag;
extern uint8_t fly_flag;

/**
  * @brief  修改舵轮底盘PID参数的接口函数
  * @param[in]  chassis     底盘结构体
  * @param[in]  pid     		底盘6020和3508的角度和速度控制PID参数
  */
void ChangeChassisPID(SwerveChassis* chassis, PIDParameters pid);


/**
  * @brief  舵轮底盘初始化,创建八个电机，赋予编码器初值和PID参数
  * @param[in]  chassis     底盘结构体
  * @param[in]  run_pid     3508驱动PID
  * @param[in]  turn_pid    6020转向PID
  * @param[in]  canx	   		舵轮挂载的CAN总线编号 输入CAN1或CAN2
  */
void SwerveChassisInit(SwerveChassis* chassis, BasePID_Object run_pid, BasePID_Object turn_pid, CanNumber canx);


/**
  * @brief  		读取接收机控制数据，根据舵轮运动逻辑解算
	* @param[in]	chassis  底盘结构体
	* @param[in]	rc_ctrl  遥控器结构体,用于更新控制数据
	* @param[in]	canAngle YAW轴电机编码器角度（0到+-180°）
  */
void SwerveChassisGetRemoteData(SwerveChassis* chassis, RC_Ctrl* rc_ctrl,Chassis_Attitude_Info* Swerve, float canAngle);

void SwerveAngleChassisInit(SwerveChassis* chassis,CanNumber canx);
/**
  * @brief  在舵轮逆运动学解算后被调用，计算3508和6020的输出电流值，并填入CAN数据到0x200和0x1ff两个数据帧
	* @param[in]	chassis  底盘结构体
  */
void SwerveChassisMotionControl(SwerveChassis* chassis , RC_Ctrl* rc_ctrl ,Holder_t* holder);


/**
	* @brief 根据接收机是否离线，判断是否发送正常底盘控制信号。控制0x1ff和0x200两个CAN数据帧的发送情况
	* @param[in]	chassis  底盘结构体
	* @param[in]	rcCtrl   遥控器结构体,用于读取遥控器离线标志位
	*/

void SwerveChassisSetFollowPID(SwerveChassis* chassis, BasePID_Object follow_pid);

void SwervePowerConrolInit(SwerveChassis* chassis, BasePID_Object base_pid, BasePID_Object power_pid);

#endif

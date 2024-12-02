#include "mecanum_chassis.h"

#define AtR 0.0174532f	              //<  3.1415 /180 角度制 转化为弧度制	


/**
  * @brief  麦轮底盘初始化函数，创建四个底盘电机，并且拷贝相同的PID参数，
  */
void MecanumChassisInit(MecanumChassis* chassis, BasePID_Object run_pid, CanNumber canx)
{
	for(int i = 1;i <= 4; i++)  //< 初始化四个电机和对应的pid结构体
	{
		MotorInit(&chassis->Motors.motor[i - 1], 0, Motor3508, canx, (0x200 + i)); 	//< 麦轮底盘的代码是默认挂载在0x200上的
		BasePID_Init(&chassis->Motors.RunPID[i - 1], run_pid.Kp, run_pid.Ki, run_pid.Kd, run_pid.KiPartDetachment);
	}	
	chassis->Movement.Vomega_Sensitivity = 1;
	chassis->Movement.Vx_Sensitivity     = 1;
	chassis->Movement.Vy_Sensitivity     = 1;
}


/**
  * @brief  设置底盘跟随的PID参数
  */
void MecanumChassisSetFollowPID(MecanumChassis* chassis, BasePID_Object follow_pid)
{
		BasePID_Init(&chassis->Motors.FollowPID, follow_pid.Kp, follow_pid.Ki, follow_pid.Kd, follow_pid.KiPartDetachment);
}



/**
  * @brief  麦轮底盘从遥控器更新控制数据，并且进行模式选择
  */
void MecanumChassisGetRemoteData(MecanumChassis* chassis, RC_Ctrl* rc_ctrl)
{
	chassis->Movement.Vx      = (rc_ctrl->rc.ch1 - 1024) * chassis->Movement.Vx_Sensitivity;	
	chassis->Movement.Vy		  = (rc_ctrl->rc.ch0 - 1024) * chassis->Movement.Vy_Sensitivity;	
	chassis->Movement.Vomega  = (rc_ctrl->rc.ch2 - 1024) * chassis->Movement.Vomega_Sensitivity; 
}



/**
  * @brief  麦轮底盘逆运动学解算，Inverse Kinematics ,根据chassis结构体中的movement结构体解算转速。
  * @notec  处理过后将未进行功率控制的电流参数填写到Motor结构体中。 
  */
void MecanumChassisSetSpeed(MecanumChassis* chassis, float canAngle)
{
	float angle = canAngle * AtR;
	float rotated_vy = (chassis->Movement.Vx * sin(angle) + chassis->Movement.Vy * cos(angle));
	float rotated_vx = (chassis->Movement.Vx * cos(angle) - chassis->Movement.Vy * sin(angle));
	
	chassis->Motors.motor[0].Data.Target = (-1) * rotated_vx +   1  * rotated_vy + 1 * chassis->Movement.Vomega;
	chassis->Motors.motor[1].Data.Target =   1  * rotated_vx +   1  * rotated_vy + 1 * chassis->Movement.Vomega;
	chassis->Motors.motor[2].Data.Target =   1  * rotated_vx + (-1) * rotated_vy + 1 * chassis->Movement.Vomega;
	chassis->Motors.motor[3].Data.Target = (-1) * rotated_vx + (-1) * rotated_vy + 1 * chassis->Movement.Vomega;	
	
	for(int i=0;i<4;i++)  //< 计算底盘电机
	{   
		if( chassis->Motors.motor[i].Data.Target >  7800)   chassis->Motors.motor[i].Data.Target =  7800;
		if( chassis->Motors.motor[i].Data.Target < -7800)   chassis->Motors.motor[i].Data.Target = -7800;
		chassis->Motors.motor[i].Data.Output = BasePID_SpeedControl((BasePID_Object*)(chassis->Motors.RunPID + i), chassis->Motors.motor[i].Data.Target, chassis->Motors.motor[i].Data.SpeedRPM);
		MotorFillData(&chassis->Motors.motor[i], chassis->Motors.motor[i].Data.Output);
	}
	 
}


/**
  * @brief  麦轮底盘发送电机控制数据
  */
void MecanumChassisOutputControl(MecanumChassis* chassis, RC_Ctrl rcCtrl)
{
	if(rcCtrl.isOnline == 1) 
	{
		MotorCanOutput(can2, 0x1ff);  
		MotorCanOutput(can2, 0x200);		
	}
	else 
	{ //< 接收机离线，电机输出为0
		MotorFillData(&chassis->Motors.motor[0],0);
		MotorFillData(&chassis->Motors.motor[1],0);
		MotorFillData(&chassis->Motors.motor[2],0);
		MotorFillData(&chassis->Motors.motor[3],0);
		MotorCanOutput(can2, 0x1ff);
		MotorCanOutput(can2, 0x200);
	}
}
	






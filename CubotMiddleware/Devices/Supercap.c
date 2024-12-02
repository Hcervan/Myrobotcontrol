#include "Supercap.h"
#include "fdcan.h"
#include "driver_can.h"
#include "dr16.h"
#include "motor.h"
#include "referee.h"
#include "hardware_config.h"
CAN_TxBuffer txBuffer0x6FFforCAN1={
	.Identifier = 0x6ff
};


uint8_t Supercap_rxCallBack(CAN_Object canx, CAN_RxBuffer rxBuffer ,Supercap* Cap)
{
	if(rxBuffer.Header.Identifier == 0x601)
	{
	Cap->cap_state.Voltage = ((rxBuffer.Data[0]<<8)+rxBuffer.Data[1])*0.01f;
	Cap->cap_state.Current = ((rxBuffer.Data[2]<<8)+rxBuffer.Data[3]);
	}
}

void SupercapControl(CAN_Object can, RC_Ctrl* rc_ctrl , Supercap* Cap)
{
	if(Cap->cap_state.Supercap_Mode == 1 && Cap->cap_state.Voltage > 16.2)
	{
		txBuffer0x6FFforCAN1.Data[0] = 1;
		Cap->cap_state.Supercap_Flag = 1;
	}
	else
	{
		txBuffer0x6FFforCAN1.Data[0] = 0;
		Cap->cap_state.Supercap_Flag = 0;
	}
	if(Cap->cap_state.Supercap_Charge_mode==0)
		Cap->cap_state.Supercap_Charge=0;
	else if (Cap->cap_state.Supercap_Charge_mode==1)
		Cap->cap_state.Supercap_Charge=1;
	txBuffer0x6FFforCAN1.Data[1] = referee2022.game_robot_status.chassis_power_limit;
	txBuffer0x6FFforCAN1.Data[2] = Cap->cap_state.Supercap_Charge;
	txBuffer0x6FFforCAN1.Data[3] = 0;
	txBuffer0x6FFforCAN1.Data[4] = 0;
	txBuffer0x6FFforCAN1.Data[5] = 0;
	txBuffer0x6FFforCAN1.Data[6] = 0;
	txBuffer0x6FFforCAN1.Data[7] = 0;
	
	CAN_Send(&can, &txBuffer0x6FFforCAN1);   
}
//#include "Super_Cap.h"
//#include "Remote.h"
//#include "Chassis_Ctrl.h"
//#include "referee_2021.h"

///*Sub Board Function*/

//Sup_Cap_t Super_Cap=
//{
//	.smode=2,
//};
//Supper_Cap_Info cap_my=
//{
//	0

//};


///******************************************************************
//函数名；Check_Key
//功能：检测超级电容开启
//参数；RC_Ctl_t * RC,Sup_Cap_t * Sup
//返回值：无
//处理结果；Use_Super_Cap
//上级函数；Super_Cap_Ctrl
//下级函数：无
//******************************************************************/
//void Check_Key(RC_Ctl_t * RC,Sup_Cap_t * Sup)
//{
//	if(RC->key.shift_flag==1)
//	{
//		if(Sup->Voltage>16)
//			Sup->Use_Super_Cap=1;
//		else
//			Sup->Use_Super_Cap=0;
//	}
//	else
//		Sup->Use_Super_Cap=0;
//}
///******************************************************************
//函数名；Charge_Ctrl
//功能：电容充电控制
//参数；Sup_Cap_t * Sup,Hero_Chassis_t * Chassis
//返回值：无
//处理结果；Charge
//上级函数；Super_Cap_Ctrl
//下级函数：无
//******************************************************************/
//void Charge_Ctrl(Sup_Cap_t * Sup,Hero_Chassis_t * Chassis)
//{
//	int i=0;
//	int16_t speed_sum;
//	
//	for(i=0;i<4;i++)
//	{
//		speed_sum+=abs(Chassis->wheel.Feedback_Speed[i]);
//	}
//	
//	if(Chassis->wheel.Target_Speed_Sum>6000*4&&speed_sum<2000*4)
//		Sup->Charge=1;
//	else
//		Sup->Charge=0;
//	
//	if(Chassis->Flag.Rock==1)
//		Sup->Charge=1;
//	if(Sup->Use_Super_Cap==1) 
//		Sup->Charge=0;
//	
//	
//	if(Sup->Use_Super_Cap==1)
//		Sup->smode=1;
//	else if(Sup->Charge==1)
//		Sup->smode=2;
//	else
//		Sup->smode=2;
//}
///******************************************************************
//函数名；Super_Cap_Ctrl
//功能：电容控制
//参数；无
//返回值：无
//处理结果；电容控制信息
//上级函数；Hero_Sys_Run
//下级函数：Check_Key、Charge_Ctrl
//******************************************************************/
//void Super_Cap_Ctrl(void)
//{
////	Charge_Ctrl(&Super_Cap,&Hero_Chassis);
////	if(Super_Cap.Voltage<17)
////		Super_Cap.Use_Super_Cap=0;
//	
//	if(Super_Cap.Use_Super_Cap==1&&Super_Cap.Voltage>=20)
//		Super_Cap.smode=1;
//	if(Super_Cap.Use_Super_Cap==0||Super_Cap.Voltage<16)
//		Super_Cap.smode=2;
//}
///******************************************************************
//函数名；Super_Cap

//自动开关电容

//自动调节超电容的充电功率
//******************************************************************/
//int16_t zxj_sup_50w_limit=0;
//int16_t zxj_sup_80w_limit=0;
//int16_t zxj_sup_100w_limit=0;
//int16_t zxj_sup_120w_limit=0;
//int16_t zxj_sup_300w_limit=0;



//void Super_Cap_zxj(void)
//{
//	
//	
//	
//	
//	
//		if(RC_Ctl.key.shift_flag==1)
//	{

//		if(cap_my.voltage>16)
//			cap_my.Use_Super_Cap=1;
//		else
//			cap_my.Use_Super_Cap=0;
//		
//		
//	}
//	else
//	{
//		cap_my.Use_Super_Cap=0;
//	}
//	int16_t sum=0;
//	int8_t i=0;
//	for(i=0;i<4;i++)
//	{
//		
//		   sum=sum+Hero_Chassis.wheel.Output[i];

//	}
//	
////>现在需要的功能是根据地盘电机输出来确定  超级电容充电功率
///*
//	思路：给个时间常量，一直减，时间常量代表超级电容充电时间
//	然后可以来一个  第一次初始化
//	
//	然后根据不同的  功率限制，不同的电机输出来选择
//	超级电容的充电时间
//	
//	
//	
//	
//	
//	*/
//	
////	if(game_robot_state.chassis_power_limit==0)
////	{i=0;
////	}
////	


////	
////	
////	
////	

//	
//	
//	
//	
//	

//}
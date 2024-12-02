#include "hardware_config.h"
#include "control_logic.h"
#include "motor.h"
#include "dr16.h"
#include "usart.h"
#include "driver_timer.h"
//< TIM14的触发频率在CubeMX中被配置为1000Hz

float target_speed=0;
/**
  * @brief  定时器中断回调
  */
void TIM14_Task(void)
{
	tim14.ClockTime++;
	if(tim14.ClockTime>=500)
	{
		target_speed=(rc_Ctrl.rc.ch1-1024);
		MotorFillData(&motor3508,target_speed);
		MotorCanOutput(can2, 0x200);
	}
}


/**
  * @brief  CAN1接收中断回调
  */
uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(can1, (*rxBuffer)); 
	return 0;
}

/**
  * @brief  CAN2接收中断回调
  */
uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(can2, (*rxBuffer)); 	
	return 0;
}

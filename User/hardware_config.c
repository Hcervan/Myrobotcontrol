#include "hardware_config.h"
#include "control_logic.h"
#include "motor.h"
#include "dr16.h"
#include "driver_timer.h"

Motor motor3508;
/**
  * @brief  初始化指令合集
  */
void HardwareConfig(void)
{
	DR16Init(&rc_Ctrl);
	
	MotorInit(&motor3508, 0, Motor3508, CAN2, 0X201);
	
	UARTx_Init(&huart1, DR16_callback);
	UART_ENABLE_IT(&uart1, &uart1_buffer);
	
	CANx_Init(&hfdcan2, CAN2_rxCallBack);
	CAN_Open (&can2 );
	
	TIMx_Init(&htim14, TIM14_Task);
	TIM_Open(&tim14);
	
	UART_Receive_DMA(&uart1, &uart1_buffer);
	
}









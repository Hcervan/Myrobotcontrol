#ifndef CONTROLLOGIC_H_
#define CONTROLLOGIC_H_
#include "stm32h7xx_hal.h"

#include "motor.h"
	

uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer);

uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer);

void TIM14_Task(void);

#endif




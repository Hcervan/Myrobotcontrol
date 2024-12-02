#ifndef DRIVERTIMER_H
#define DRIVERTIMER_H
#include "tim.h"


typedef void (*TIM_ElapsedCallback)(void); 


typedef struct
{
	TIM_HandleTypeDef* 	Handle;
//	uint32_t ClockTime;             			//< 任务定时器的计数变量
	int32_t ClockTime;
	TIM_ElapsedCallback		ElapCallback;		//< 定时器溢出中断
	uint16_t HolderTime;
	uint16_t ErrorTime;
	uint16_t UI_Time;
}TIM_Object;

/**
  * @brief 初始化定时器的用户回调
  */
void TIMx_Init(TIM_HandleTypeDef* handle, TIM_ElapsedCallback callback);


/**
  * @brief     开启定时器溢出中断
  */
void TIM_Open(TIM_Object* tim);



extern TIM_Object tim14;;


#endif


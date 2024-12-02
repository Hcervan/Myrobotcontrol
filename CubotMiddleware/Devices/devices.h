#ifndef __DEVICES_H__
#define __DEVICES_H__

#include "stm32h7xx_hal.h"
#include "motor.h"
#include "dr16.h"
#include "linux_list.h" 


typedef enum 
{ 
	DJIMotor    = 0x01U,
	SuperCap    = 0x02U,
	SlaveBoard  = 0x03U,
	MasterBoard = 0x03U
}DeviceType;




typedef struct 
{
	list_t   		list;
	DeviceType  type;

}Device;



#endif // __DEVICES_H__

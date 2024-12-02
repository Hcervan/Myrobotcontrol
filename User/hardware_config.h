#ifndef HARDWARE_CONFIG_H_
#define HARDWARE_CONFIG_H_
#include "stm32h7xx_hal.h"
#include "motor.h"
/**
  * @brief  完成剩余的驱动器配置，开启驱动器
  */
void HardwareConfig(void);

extern Motor motor3508;
#endif




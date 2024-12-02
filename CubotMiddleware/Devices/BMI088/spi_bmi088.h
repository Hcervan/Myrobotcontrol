/*! \file bmi088_stm32.h
 \STM32 specific SPI functions */

/*********************************************************************/
/* header files */
#ifndef __SPI_BMI088_H_
#define __SPI_BMI088_H_
#include "main.h"

extern int8_t stm32_spi_write(GPIO_TypeDef * cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len);

extern int8_t stm32_spi_read(GPIO_TypeDef * cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len);

#endif

/**
*/


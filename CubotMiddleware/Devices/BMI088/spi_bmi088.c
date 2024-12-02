///**\mainpage
// * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright
// * notice, this list of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright
// * notice, this list of conditions and the following disclaimer in the
// * documentation and/or other materials provided with the distribution.
// *
// * Neither the name of the copyright holder nor the names of the
// * contributors may be used to endorse or promote products derived from
// * this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
// * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
// * OR CONTRIBUTORS BE LIABLE FOR ANY
// * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
// * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
// * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// * ANY WAY OUT OF THE USE OF THIS
// * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
// *
// * The information provided is believed to be accurate and reliable.
// * The copyright holder assumes no responsibility
// * for the consequences of use
// * of such information nor for any infringement of patents or
// * other rights of third parties which may result from its use.
// * No license is granted by implication or otherwise under any patent or
// * patent rights of the copyright holder.
// *
// * @file        bmi088.c
// * @date        24 Aug 2018
// * @version     1.2.0
// *
// */

///*! \file bmi088_stm32.c
// \STM32 specific SPI functions */
///****************************************************************************/
///**\name        Header files
// ****************************************************************************/

//#include "stm32h7xx_hal.h"
//#include "spi_bmi088.h"

//extern SPI_HandleTypeDef hspi1;
///**
//* @brief spi方式下对bmi088的读写操作
//*	@param
//	##cs_pin:片选引脚
//	##reg_addr:所需读/写的寄存器地址
//	##*data:读/写的数据缓存区
//	##len：读写的字节长度
//*/
//int8_t stm32_spi_write(GPIO_TypeDef * cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len)
//{
//	reg_addr &= 0x7f;//0111 1111写操作
//	HAL_GPIO_WritePin(cs_pin, GPIO_PIN_4, GPIO_PIN_RESET);//进行片选
//	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 50);
//	//对BMI088进行读写操作时，需要先对其发生一个8Bits的数据，bit#0代表读/写(1/0),bit#1~7代表reg_addr
//	HAL_SPI_Transmit(&hspi1, data, len, 50);
//	//写：Transmit；读：Receive
//	HAL_GPIO_WritePin(cs_pin, GPIO_PIN_4, GPIO_PIN_SET);//取消片选
//	return 0;
//}


//int8_t stm32_spi_read(GPIO_TypeDef * cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len)
//{
//	reg_addr |=0X80;//1000 0000读操作
//	HAL_GPIO_WritePin(cs_pin, GPIO_PIN_4, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 50);
//	HAL_SPI_Receive(&hspi1, data, len, 50);
//	HAL_GPIO_WritePin(cs_pin, GPIO_PIN_4, GPIO_PIN_SET);
//	return 0;
//}
///** @}*/

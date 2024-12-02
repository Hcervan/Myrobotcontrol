#ifndef DRV_CAN_H_
#define DRV_CAN_H_
#include "stm32h7xx_hal.h"
#include "fdcan.h"	
#include "linux_list.h"


/**
	* @brief	CAN设备号枚举
	*/
typedef enum 
{
	CAN1 = 0x01U,
	CAN2 = 0x02U
}CanNumber;


/**
	* @brief	CAN接收缓冲区
	*/
typedef struct
{
	FDCAN_RxHeaderTypeDef	Header;
	uint8_t								Data[8];
}CAN_RxBuffer;


/**
	* @brief	CAN发送缓冲区
	*/
typedef struct
{
	uint32_t							Identifier;
	uint8_t								Data[8];
}CAN_TxBuffer;


/**
  * @brief   CAN用户回调函数
	* @param[in] rxBuffer 	CAN接收缓冲区
  */
typedef uint8_t(*CAN_RxCpltCallback)(CAN_RxBuffer* rxBuffer);


/**
	* @brief	CAN设备对象
	*/
typedef struct
{
	FDCAN_HandleTypeDef* 	Handle;
	list_t								DevicesList;
	CAN_RxCpltCallback		RxCpltCallback;
}CAN_Object;


/**
  * @brief  CAN初始化，将句柄和接收回调拷贝至CAN结构体
  * @param[in]  handle		        串口句柄
  */
void CANx_Init(FDCAN_HandleTypeDef* handle, CAN_RxCpltCallback rxCallback);


/**
  * @brief 打开CAN设备，配置过滤器为空，使能fifo0接收到新信息中断，注册用户回调
  * @param[in]	can	CAN设备
  */
void CAN_Open(CAN_Object* can);


/**
  * @brief 通过CAN设备发送数据
  *	@param[in] txBuffer CAN的发送缓冲区
	* 
  */
uint8_t CAN_Send(CAN_Object* can, CAN_TxBuffer* txBuffer);


/**
  * @brief CAN设备弱函数回调
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* handle, uint32_t RxFifo0ITs);



extern CAN_Object can1;
extern CAN_Object can2;


#endif



///*从库里偷来的 接收信息中断的判断*/
//#define FDCAN_RX_FIFO0_MASK (FDCAN_IR_RF0L | FDCAN_IR_RF0F | FDCAN_IR_RF0W | FDCAN_IR_RF0N)




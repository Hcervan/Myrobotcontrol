/**@file  driver_usart.c
* @brief   驱动层，串口管理器配置文件，用户回调重定义
* @details  主要包括构建串口管理器，提供串口初始化和用户回调重定义
* @author      RyanJiao  any question please send mail to 1095981200@qq.com						 
* @date        2021-8-23
* @version     V1.1
* @copyright    Copyright (c) 2021-2121  中国矿业大学CUBOT战队
**********************************************************************************
* @attention
* 硬件平台: STM32H750VBT \n
* SDK版本：-++++
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021-8-12  <td>1.0      <td>RyanJiao  <td>创建初始版本
* </table>
*
**********************************************************************************
 ==============================================================================
                          How to use this driver  
 ==============================================================================
 
	添加driver_can.h

	1. 调UARTx_Init() 将 句柄 和 用户定义的接收回调函数 拷贝至UART结构体  （回调函数中对接收到的数据进行 ID识别 和 合并解算）

  2. 用户编写 UART_RxBuffer，填入 目标缓存区地址 和 数据长度。

	3. 调用UART_Open() 传入 UART_Object 和 用户编写 UART_RxBuffer。
	
	4. 将 UART_Idle_Handler 添加到 stm32H7xx_it.c 的 USARTx_IRQHandler() 中，调用用户编写的同一个 UART_RxBuffer 。
	
	5. 应用层编写 UART_TxBuffer （发送缓存区结构体），填入待发送字节数组首地址和字节长度
	
	6. 调UART_Send()传入 UART设备结构体 和 UART_TxBuffer结构体，将数据发送出去

  ********************************************************************************
	* @attention
	* 硬件平台: STM32H750VBT \n
	* SDK版本：-++++
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  ********************************************************************************
	对DMA中NDTR寄存器描述：
	This register can be written only
	when the stream is disabled. When the stream is enabled, this register is read-only,
	indicating the remaining data items to be transmitted. This register decrements after each
	DMA transfer.
	指明了DMA中待传输的剩余数据个数 每次DMA传输完成后自动减一
	参考手册中对idle空闲中断触发条件的描述：
	
*/
#include "driver_usart.h"
//#include "referee.h"
#include "usart.h"
//#include "vision.h"
#include "hardware_config.h"
#include "stdio.h"
UART_Object uart1;
UART_Object uart2;
UART_Object uart3;
UART_Object uart4;
UART_Object uart5;
UART_Object uart6;


/**
  * @brief   串口初始化，将句柄和接收回调拷贝至串口结构体
  */
void UARTx_Init(UART_HandleTypeDef* handle, UART_RxIdleCallback rxIdleCallback)
{
	// 初始化uart1
	if (handle->Instance == USART1)
	{
		uart1.Handle = handle;
		uart1.RxIdleCallback = rxIdleCallback;
	}
	
	// 初始化uart2
	if (handle->Instance == USART2)
	{
		uart2.Handle = handle;
		uart2.RxIdleCallback = rxIdleCallback;
	}
	
	// 初始化uart3
	if (handle->Instance == USART3)
	{
		uart3.Handle = handle;
		uart3.RxIdleCallback = rxIdleCallback;
	}
	
	// 初始化uart4
	if (handle->Instance == UART4)
	{
		uart4.Handle = handle;
		uart4.RxIdleCallback = rxIdleCallback;
	}
	
	// 初始化uart5
	if (handle->Instance == UART5)
	{
		uart5.Handle = handle;
		uart5.RxIdleCallback = rxIdleCallback;
	}
	
	// 初始化uart6
	if (handle->Instance == USART6)
	{
		uart6.Handle = handle;
		uart6.RxIdleCallback = rxIdleCallback;
	}
}


/**
  * @brief  串口管理器结构体参数已经预先填写好的串口设备初始化
  */
void UART_Receive_DMA(UART_Object* uart, UART_RxBuffer* rxBuffer)
{
	HAL_UART_Receive_DMA(uart->Handle, rxBuffer->Data, rxBuffer->Size);
//	__HAL_UART_ENABLE_IT(uart->Handle, UART_IT_IDLE);   		//<开启中断前分配回调比较好
} 


/**
  * @brief  串口管理器结构体参数已经预先填写好的串口设备初始化     //舵轮步兵分开使用，延后DMA接收
  */
void UART_ENABLE_IT(UART_Object* uart, UART_RxBuffer* rxBuffer)
{
//	HAL_UART_Receive_DMA(uart->Handle, rxBuffer->Data, rxBuffer->Size);
	__HAL_UART_ENABLE_IT(uart->Handle, UART_IT_IDLE);   		//<开启中断前分配回调比较好
} 

/**
  * @brief  串口调用dma发送数据，数据需拆分为字节
  */
uint32_t UART_Send(UART_Object* uart,UART_TxBuffer* txBuffer)
{
	return HAL_UART_Transmit_DMA(uart->Handle,txBuffer->Data, txBuffer->DataSize);
}



/**
  * @brief  串口设备中断函数，执行中断DMA操作，调用串口用户回调函数 
  */
void UART_Idle_Handler(UART_Object* uart, UART_RxBuffer* rxBuffer)
{
	assert_param(uart != NULL);
	
	uint16_t usart_rx_num;
	
	if((__HAL_UART_GET_FLAG(uart->Handle, UART_FLAG_IDLE) != RESET))
	{	
		HAL_UART_DMAStop(uart->Handle);																																					//< 关闭DMA，防止解算过程中数据更新，造成丢失数据															
		__HAL_UART_CLEAR_IDLEFLAG(uart->Handle);																																//< 清楚idle标志位，防止再次进入中断			
		__HAL_UART_CLEAR_OREFLAG(uart->Handle);	
//		if ((((DMA_Stream_TypeDef*)uartManage->huart.hdmarx->Instance)->NDTR) == uartManage->rxBufferSize)		//< 判断缓存区长度 是否等于 DMA中未传输数据长度。若为等于，数据更新完一次。 
		usart_rx_num = rxBuffer->Size - ((DMA_Stream_TypeDef*)uart->Handle->hdmarx->Instance)->NDTR;  			//< 
		{																																																				//< 如果发送方先启动，则接收方开始接收后是否需要匹配头字节和尾字节																																														
			if((*uart).RxIdleCallback!=NULL)
			  uart->RxIdleCallback(rxBuffer->Data, usart_rx_num);            				 		   												//<用户回调
		}
		HAL_UART_DMAResume(uart->Handle);		
		HAL_UART_Receive_DMA(uart->Handle, rxBuffer->Data, rxBuffer->Size);
	}
}
//void USART2_IDLE_CALLBACK()
//{
//uint16_t temp,rx_len;
//	if(( __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE)!= RESET))//idle标志被置位
//	{ 	
//	//	HAL_UART_DMAStop(&huart2); 
//	  __HAL_DMA_DISABLE(&hdma_usart2_rx);
//		__HAL_UART_CLEAR_IDLEFLAG(&huart2);//清除标志位
//		__HAL_UART_CLEAR_OREFLAG(&huart2);			 
//		
//		temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);// 获取DMA中未传输的数据个数
//		rx_len = 6 - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
//	  Run_User_Vision_Task(&rc_Ctrl,&Holder,&Vision_Info);
//		__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,6);
////	__HAL_DMA_CLEAR_FLAG(&hdma_usart2_rx,DMA_FLAG_TCIF2_6|DMA_FLAG_HTIF2_6);
////		DMA1_Stream2->NDTR = (uint32_t)7;
//		__HAL_DMA_ENABLE(&hdma_usart2_rx);		
//		HAL_UART_Receive_DMA(&huart2,Vision_meta,6);//重新打开DMA接收  		
//	}
//}
//void USART3_IDLE_CALLBACK()
//{
//	uint8_t this_time_length;
//	if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) != RESET))
//	{  
//		HAL_UART_DMAStop(&huart3);
//		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
//		__HAL_UART_CLEAR_OREFLAG(&huart3);
//		this_time_length = BSP_USART3_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
//		Referee_Data_Diapcak(meta_data,this_time_length);
//		HAL_UART_Receive_DMA(&huart3,meta_data,BSP_USART3_DMA_RX_BUF_LEN);
//	}

//}

//uint8_t data_to_send_V6[20]__attribute__((at(0x24002000)));
//void ANO_V6_Send_Up_Computer(int16_t user1,int16_t user2,int16_t user3,int16_t user4,int16_t user5,int16_t user6)
//{
//	uint8_t _cnt=0;
//	int16_t _temp;
//	
//	uint8_t sum = 0;
//	uint8_t i=0;
//		/*   协议为V6版本 注意观察区别   */
//	data_to_send_V6[_cnt++]=0xAA;
//	data_to_send_V6[_cnt++]=0x05; //或许00也可以 05是匿名拓空者飞控
//	data_to_send_V6[_cnt++]=0xAF; //AF是上位机识别码
//	data_to_send_V6[_cnt++]=0xF1; 
//	data_to_send_V6[_cnt++]=0;
//	
//	_temp = user1;    
//	data_to_send_V6[_cnt++]=BYTE1(_temp);
//	data_to_send_V6[_cnt++]=BYTE0(_temp);
//	_temp = user2;
//	data_to_send_V6[_cnt++]=BYTE1(_temp);
//	data_to_send_V6[_cnt++]=BYTE0(_temp);
//	_temp = user3;
//	data_to_send_V6[_cnt++]=BYTE1(_temp);
//	data_to_send_V6[_cnt++]=BYTE0(_temp);
//	_temp = user4;    
//	data_to_send_V6[_cnt++]=BYTE1(_temp);
//	data_to_send_V6[_cnt++]=BYTE0(_temp);
//	_temp = user5;
//	data_to_send_V6[_cnt++]=BYTE1(_temp);
//	data_to_send_V6[_cnt++]=BYTE0(_temp);
//	_temp = user6;
//	data_to_send_V6[_cnt++]=BYTE1(_temp);
//	data_to_send_V6[_cnt++]=BYTE0(_temp);
//	
//	data_to_send_V6[6] = _cnt-5;
//	
//	sum = 0;
//	for(i=0;i<_cnt;i++)
//			sum += data_to_send_V6[i];
//	data_to_send_V6[_cnt++] = sum;
//	
//	HAL_UART_Transmit_DMA(&huart6,data_to_send_V6,_cnt);
//}
//ano_tc_data[0] = Vision_Info.X_Error;
//ano_tc_data[1] = gimbal.down.yaw.target_angle ;
//ano_tc_data[2] = gimbal.down.yaw.feedback_angle;
//ano_tc_data[3] = Pid_Yaw.Ep_Part;
//ano_tc_data[4] = Pid_Yaw.Ei_Part;	
//ano_tc_data[5] = Pid_Yaw.Ed_Part;
//ano_tc_data[6] = Pid_Yaw.Out;
//ano_tc_data[7] = gimbal.down.yaw.feedback_speed;
//SendToANO(ano_tc_data,sizeof(ano_tc_data));


/**
  * @brief  整型转换成字符串ASCii码 
  */
uint8_t* itoa(int  num, uint8_t* str, int radix)
{
    uint8_t index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"; //<索引表
    unsigned unum;																	  			//<存放要转换的整数的绝对值,转换的整数可能是负数
    int i=0,j,k;																					  //<i用来指示设置字符串相应位，转换之后i其实就是字符串的长度；转换后顺序是逆序的，有正负的情况，k用来指示调整顺序的开始位置;j用来指示调整顺序时的交换。
																														//<获取要转换的整数的绝对值
    if(radix==10&&num<0)																		//<要转换成十进制数并且是负数
    {
        unum=(unsigned)-num;																//<将num的绝对值赋给unum
        str[i++]='-';																				//<在字符串最前面设置为'-'号，并且索引加1
    }
    else unum=(unsigned)num;																//<若是num为正，直接赋值给unum
		
    do																							  			//<转换部分，注意转换后是逆序的
    {
        str[i++]=index[unum%(unsigned)radix];							//<取unum的最后一位，并设置为str对应位，指示索引加1
        unum/=radix;																				//<unum去掉最后一位
 
    }while(unum);																						//<直至unum为0退出循环
 
    str[i]='\0';																						//<在字符串最后添加'\0'字符，c语言字符串以'\0'结束。
 
																														//<将顺序调整过来
    if(str[0]=='-') k=1;																		//<如果是负数，符号不用调整，从符号后面开始调整
    else k=0;																								//<不是负数，全部都要调整
 
    uint8_t temp;																						//<临时变量，交换两个值时用到
    for(j=k;j<=(i-1)/2;j++)																	//<头尾一一对称交换，i其实就是字符串的长度，索引最大值比长度少1
    {
        temp=str[j];																				//<头部赋值给临时变量
        str[j]=str[i-1+k-j];																//<尾部赋值给头部
        str[i-1+k-j]=temp;																	//<将临时变量的值(其实就是之前的头部值)赋给尾部
    }
    return str;																							//<返回转换后的字符串
 
}

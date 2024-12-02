	/**@file  dr16.c
	* @brief    �豸��
	* @details  ��Ҫ�����������ڹ��������ṩ���ڳ�ʼ�����û��ص��ض���
	* @author      RyanJiao  any question please send mail to 1095981200@qq.com
	* @date        2021-10-9
	* @version     V1.0
	* @copyright    Copyright (c) 2021-2121  �й���ҵ��ѧCUBOTս��
	**********************************************************************************
	* @attention
	* Ӳ��ƽ̨: STM32H750VBT \n
	* SDK�汾��-++++
	* @par �޸���־:
	* <table>
	* <tr><th>Date        <th>Version  <th>Author    <th>Description
	* <tr><td>2021-8-12  <td>1.0      <td>RyanJiao  <td>������ʼ�汾
	* </table>
	*
	**********************************************************************************
	 ==============================================================================
														How to use this driver  
	 ==============================================================================
	 

		********************************************************************************
		* @attention
		* Ӳ��ƽ̨: STM32H750VBT \n
		* SDK�汾��-++++
		* if you had modified this file, please make sure your code does not have many 
		* bugs, update the version NO., write dowm your name and the date, the most
		* important is make sure the users will have clear and definite understanding 
		* through your new brief.
		********************************************************************************
	*/

	#include "dr16.h"
	#include "hardware_config.h"
//	#include "control_logic.h"

	#define myABS(x) ( (x)>0?(x):-(x) )
	RC_Ctrl rc_Ctrl={
		.isUnpackaging = 0,
		.isOnline = 0
		
	};

	/**
		* @brief  ����dr16�Ľ��ջ���������
		*/
	uint8_t DR16_recData[DR16_rxBufferLengh]__attribute__((at(0x24006000)));


	/**
		* @brief  ����dr16���ڻ��������ݽṹ
		*/
	UART_RxBuffer uart1_buffer={
		.Data = DR16_recData,
		.Size = DR16_rxBufferLengh
	};

	/**
		* @brief  ��ʼ�����ջ��������͵����ݣ��������Ͱ�����Ϣ����
		*/
	void DR16Init(RC_Ctrl* RC_Ctl)
	{
		RC_Ctl->rc.ch0=1024;
		RC_Ctl->rc.ch1=1024;
		RC_Ctl->rc.ch2=1024;
		RC_Ctl->rc.ch3=1024;
		RC_Ctl->rc.s1=3;
		RC_Ctl->rc.s2=3;
		RC_Ctl->rc.sw=1024;
		RC_Ctl->mouse.x=0;
		RC_Ctl->mouse.y=0;
		RC_Ctl->mouse.z=0;		
		RC_Ctl->OneShoot = 0;
		RC_Ctl->key_Q_flag=0;
		RC_Ctl->key_E_flag=0;    //< �ϵ�ص���
		RC_Ctl->key_R_flag=0;
		RC_Ctl->key_F_flag=0;
		RC_Ctl->key_G_flag=0;
		RC_Ctl->key_Z_flag=0;
		RC_Ctl->key_X_flag=0;
		RC_Ctl->key_C_flag=0;
		RC_Ctl->key_V_flag=0;
		RC_Ctl->key_B_flag=0;
		RC_Ctl->Chassis_Y_Integ=0;//б�»��ֱ���
		RC_Ctl->Chassis_X_Integ=0;
		RC_Ctl->ShootNumber=9;
		RC_Ctl->Cruise_Mode = 0;
	}


	/**
		* @brief  ����dr16�Ľ��ջ���������
		*/
	uint8_t DR16_callback(uint8_t * recBuffer, uint16_t len)
	{
		DR16_DataUnpack(&rc_Ctrl, recBuffer, len);  //< callback�����ɸ�ʽ����
		return 0;
	}


	/**
		* @brief  ����dr16�Ľ��ջ���������, ����ȫ�ֱ���rc_Ctrl��ֵ���Թ�������������
		*/
	void DR16_DataUnpack(RC_Ctrl* rc_ctrl, uint8_t * recBuffer, uint16_t len )
	{ 
		rc_ctrl->isUnpackaging = 1;					//< �����ڼ䲻�����ȡ����
		uint8_t correct_num=0;	
		correct_num=0;
		if(((recBuffer[0] | (recBuffer[1] << 8)) & 0x07ff)<=1684 && ((recBuffer[0] | (recBuffer[1] << 8)) & 0x07ff)>=364)
			correct_num++;
		if((((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff)<=1684 && (((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff)>=364)
			correct_num++;
		if((((recBuffer[2] >> 6) | (recBuffer[3] << 2) |(recBuffer[4] << 10)) & 0x07ff)<=1684 && (((recBuffer[2] >> 6) | (recBuffer[3] << 2) |(recBuffer[4] << 10)) & 0x07ff)>=364)
			correct_num++;
		if((((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff)<=1684 && (((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff)>=364)
			correct_num++;
		if((((recBuffer[5] >> 4)& 0x000C) >> 2)==1 || (((recBuffer[5] >> 4)& 0x000C) >> 2)==2 || (((recBuffer[5] >> 4)& 0x000C) >> 2)==3)
			correct_num++;
		if(((recBuffer[5] >> 4)& 0x0003)==1 || ((recBuffer[5] >> 4)& 0x0003)==2 || ((recBuffer[5] >> 4)& 0x0003)==3)
			correct_num++;
		if(correct_num==6)																																												//< ������������֤ 
		{
			rc_ctrl->rc.ch0 = (recBuffer[0]| (recBuffer[1] << 8)) & 0x07ff; 																					//< Channel 0   ��8λ���3λ
			rc_ctrl->rc.ch1 = ((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff; 																	//< Channel 1   ��5λ���6λ
			rc_ctrl->rc.ch2 = ((recBuffer[2] >> 6) | (recBuffer[3] << 2) |(recBuffer[4] << 10)) & 0x07ff; 						//< Channel 2
			rc_ctrl->rc.ch3 = ((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff; 																	//< Channel 3
			rc_ctrl->rc.s1 = ((recBuffer[5] >> 4)& 0x000C) >> 2; 																											//!< Switch left
			rc_ctrl->rc.s2 = ((recBuffer[5] >> 4)& 0x0003); 																													//!< Switch right
			rc_ctrl->rc.sw=(uint16_t)(recBuffer[16]|(recBuffer[17]<<8))&0x7ff; 
				
			if((rc_ctrl->rc.ch0>1020)&&(rc_ctrl->rc.ch0<1028))          //ң������Ʈ
				rc_ctrl->rc.ch0=1024;
			if((rc_ctrl->rc.ch1>1020)&&(rc_ctrl->rc.ch1<1028))
				rc_ctrl->rc.ch1=1024;
			if((rc_ctrl->rc.ch2>1020)&&(rc_ctrl->rc.ch2<1028))
				rc_ctrl->rc.ch2=1024;
			if((rc_ctrl->rc.ch3>1020)&&(rc_ctrl->rc.ch3<1028))
				rc_ctrl->rc.ch3=1024;
			}
		}


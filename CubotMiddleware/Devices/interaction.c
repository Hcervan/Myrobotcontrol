#include "interaction.h"
#include "string.h"
#include "usart.h"
#include "brain.h"





/*

			 ***************代码解析*****************
	Data_Pack[n]用于存放我们发给裁判系统的数据，该数据通过串口3发送给裁判系统，裁判系统
	依据协议将在操作手界面上显示用户自定义UI界面。
	Data_Pack[0]-Data_Pack[4]是帧头，其中的数据含义在裁判系统协议手册里开头给出，我们需
	要改的就只是Data_Pack[1]和Data_Pack[2]这两位，这两位用于存放数据长度，绘制不同的图形
	所需的数据长度是不一样的：

	内容ID				数据长度				功能
	0x0101					21				绘制一个图形
	0x0102					36				绘制两个图形
	0x0103					81				绘制五个图形
	0x0104					111				绘制七个图形
	0x0110					51				绘制字符图形

	其中，每个图形所占数据位为15位，字符占45位，数据段ID内容、发送者ID、接收者ID各占2位，也就是数据
	的前六位用于存放ID信息，所以21=6+15,36=6+15*2,依次类推。

	首先建立一个图形结构体用于存放图形数据，在定义相关功能函数将结构体发送出去，完成UI界面的显示


				**************使用方法****************
	将想要显示的图形函数放在函数Client_Display（）里，并且在主任务函数里不停调用Client_Display（）函数
 
 */
 
 uint8_t Data_Pack[128]__attribute__((at(0x24065000)));

UI_t UI;

game_state_t each_state={
	1,1,1,0,1,1
};

color_test_t color_test={
	YELLOW,YELLOW,RED_BLUE,WHITE,YELLOW,ORANGE,YELLOW,RED_BLUE,YELLOW,YELLOW,RED_BLUE,ORANGE
};


/*测试下发敌方位置小地图*/   //还没成（应该和雷达配合吧）
void draw_client_map_information(uint8_t robot_id, float x, float y)
{
	uint16_t crc16_temp;
	UI.ext_client_map_command.frame_header.SOF = 0xA5;
	UI.ext_client_map_command.frame_header.data_length = 10;   //< sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)
	UI.ext_client_map_command.frame_header.seq = 0;
	UI.ext_client_map_command.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_map_command.frame_header,4,0xFF); 
	UI.ext_client_map_command.cmd_id = 0x0305;
	UI.ext_client_map_command.target_robot_ID = robot_id;
	UI.ext_client_map_command.target_position_x = x;
	UI.ext_client_map_command.target_position_y = y;

	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_map_command, 17, 0xFFFF);
	UI.ext_client_map_command.CRC16[0] = crc16_temp & 0xFF;
	UI.ext_client_map_command.CRC16[1] = crc16_temp >> 8;
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&UI.ext_client_map_command,19);
}



/**
  * @brief  机器人间交互数据（步兵->哨兵）（成功）
	*/
void draw_interaction(uint8_t robot_id, uint16_t data)
{
	int i;
	uint16_t crc16_temp;
	UI.robot_interactive_data.frame_header.SOF = 0xA5;
	UI.robot_interactive_data.frame_header.data_length = 21;
	UI.robot_interactive_data.frame_header.seq = 0;
	UI.robot_interactive_data.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.robot_interactive_data.frame_header,4,0xFF); 
	UI.robot_interactive_data.cmd_id = 0x0301;
	UI.robot_interactive_data.ext_student_interactive_header_data.data_cmd_id = 0x0201;
	UI.robot_interactive_data.ext_student_interactive_header_data.sender_ID = robot_id;
	UI.robot_interactive_data.ext_student_interactive_header_data.receiver_ID = RobotRedSentry;
	UI.robot_interactive_data.data[0]=data;
	UI.robot_interactive_data.data[1]=data>>8;	
	for(i=2;i<15;i++)
	UI.robot_interactive_data.data[i]=0;
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.robot_interactive_data, 28, 0xFFFF);
	UI.robot_interactive_data.CRC16[0] = crc16_temp & 0xFF;
	UI.robot_interactive_data.CRC16[1] = crc16_temp >> 8;
	
	memcpy(Data_Pack,(unsigned char*)&UI.robot_interactive_data,sizeof(UI.robot_interactive_data));
	
	HAL_UART_Transmit_DMA(&huart3,Data_Pack,sizeof(UI.robot_interactive_data));
	
//	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&UI.robot_interactive_data,sizeof(UI.robot_interactive_data));
}


void graphic_data_modify(graphic_data_struct_t* graphic_data_struct, 
												 uint8_t name,
												 uint32_t operation,
												 uint32_t type,
												 uint32_t layer,
												 uint32_t color,
												 uint32_t size,
												 uint32_t number,		//< 小数点后位数
												 uint32_t wide,
											   uint32_t start_x,  //开始X坐标
												 uint32_t start_y,  //开始Y坐标
												 uint32_t radius,   //半径或字体大小
												 uint32_t end_x,    //终止X坐标
												 uint32_t end_y)    //终止Y坐标
{
		graphic_data_struct->operate_tpye = operation;
	
}


/*测试删除图层（应该能成，还没测）*/
void referee_graphic_delete(uint8_t del_layer, uint8_t operation, uint8_t robot_id) 
{
	uint16_t crc16_temp;
	UI.ext_client_graphic_delete.frame_header.SOF = 0xA5;
	UI.ext_client_graphic_delete.frame_header.data_length = 8;   //< sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)
	UI.ext_client_graphic_delete.frame_header.seq = 0;
	UI.ext_client_graphic_delete.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_graphic_delete.frame_header,4,0xFF); 
	UI.ext_client_graphic_delete.cmd_id = 0x0301;
	UI.ext_client_graphic_delete.ext_student_interactive_header_data.data_cmd_id = 0x0100;
	UI.ext_client_graphic_delete.ext_student_interactive_header_data.sender_ID = robot_id;
	UI.ext_client_graphic_delete.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	UI.ext_client_graphic_delete.ext_client_custom_graphic_delete.operate_tpye=operation;
	UI.ext_client_graphic_delete.ext_client_custom_graphic_delete.layer=del_layer;
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_graphic_delete, 15, 0xFFFF);
	UI.ext_client_graphic_delete.CRC16[0] = crc16_temp & 0xFF;
	UI.ext_client_graphic_delete.CRC16[1] = crc16_temp >> 8;
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&UI.ext_client_graphic_delete,17);
}


/**
  * @brief  在0号图层画字符的程序，最长30个字符
	*/
void referee_draw_string(uint8_t robot_id,char *string,uint8_t string_dex,uint8_t control_way,uint8_t color,uint8_t on_layer, uint16_t x,uint16_t y)
{
	uint16_t crc16_temp;
	UI.ext_client_custom_character.frame_header.SOF = 0xA5;
	UI.ext_client_custom_character.frame_header.data_length = 51;   //< sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)
	UI.ext_client_custom_character.frame_header.seq = 0;
	UI.ext_client_custom_character.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_character.frame_header,4,0xFF); 
	UI.ext_client_custom_character.cmd_id = 0x0301;
	UI.ext_client_custom_character.ext_student_interactive_header_data.data_cmd_id = 0x0110;
	UI.ext_client_custom_character.ext_student_interactive_header_data.sender_ID = robot_id;
	UI.ext_client_custom_character.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	UI.ext_client_custom_character.grapic_data_struct.graphic_name[0]=string_dex;
	UI.ext_client_custom_character.grapic_data_struct.graphic_name[1]=0x0;
	UI.ext_client_custom_character.grapic_data_struct.graphic_name[2]=0x0;
	UI.ext_client_custom_character.grapic_data_struct.operate_tpye = control_way;
	UI.ext_client_custom_character.grapic_data_struct.graphic_tpye = 0x07;         //< 表明字符型
	UI.ext_client_custom_character.grapic_data_struct.layer = on_layer;
	UI.ext_client_custom_character.grapic_data_struct.color = color;
	UI.ext_client_custom_character.grapic_data_struct.start_angle = 18;
	UI.ext_client_custom_character.grapic_data_struct.end_angle = 20;
	UI.ext_client_custom_character.grapic_data_struct.width = 2;
	UI.ext_client_custom_character.grapic_data_struct.start_x = x;
	UI.ext_client_custom_character.grapic_data_struct.start_y = y;
	UI.ext_client_custom_character.grapic_data_struct.radius = 0;
	UI.ext_client_custom_character.grapic_data_struct.end_x = 0;
	UI.ext_client_custom_character.grapic_data_struct.end_y = 0;
	memset(UI.ext_client_custom_character.data, 0, 30);					//< 每次绘制前清除上次字符串缓存
	strcpy((char *)UI.ext_client_custom_character.data, string);
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_character, 58, 0xFFFF);
	UI.ext_client_custom_character.CRC16[0] = crc16_temp & 0xFF;
	UI.ext_client_custom_character.CRC16[1] = crc16_temp >> 8;
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&UI.ext_client_custom_character,60);
}


/**
  * @brief  在1号图层画浮点数的程序，保留三位小数
	*/
int32_t float_temp;
void referee_draw_float(uint8_t robot_id, float float_data,uint8_t float_index, uint8_t control_way,uint8_t color, uint8_t layer, uint16_t x,uint16_t y)
{
	uint16_t crc16_temp;
	float_temp =(int32_t)(float_data * 1000);
	UI.ext_client_custom_graphic_float.frame_header.SOF = 0xA5;
	UI.ext_client_custom_graphic_float.frame_header.data_length = 21;
	UI.ext_client_custom_graphic_float.frame_header.seq = 0;
	UI.ext_client_custom_graphic_float.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_float.frame_header,4,0xFF); 
	UI.ext_client_custom_graphic_float.cmd_id = 0x0301;
	UI.ext_client_custom_graphic_float.ext_student_interactive_header_data.data_cmd_id = 0x0101;
	UI.ext_client_custom_graphic_float.ext_student_interactive_header_data.sender_ID = robot_id;
	UI.ext_client_custom_graphic_float.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	UI.ext_client_custom_graphic_float.grapic_data_struct.graphic_name[0]=float_index;
	UI.ext_client_custom_graphic_float.grapic_data_struct.graphic_name[1] = 0x0;  //< "A" 97
	UI.ext_client_custom_graphic_float.grapic_data_struct.graphic_name[2]= 0x0;    //< "A"
	UI.ext_client_custom_graphic_float.grapic_data_struct.operate_tpye = control_way;
	UI.ext_client_custom_graphic_float.grapic_data_struct.graphic_tpye = 5;
	UI.ext_client_custom_graphic_float.grapic_data_struct.layer = layer;
	UI.ext_client_custom_graphic_float.grapic_data_struct.color = color;
	UI.ext_client_custom_graphic_float.grapic_data_struct.start_angle = 18;
	UI.ext_client_custom_graphic_float.grapic_data_struct.end_angle = 2;      								 //< 小数位有效个数
	UI.ext_client_custom_graphic_float.grapic_data_struct.width = 2;
	UI.ext_client_custom_graphic_float.grapic_data_struct.start_x = x;
	UI.ext_client_custom_graphic_float.grapic_data_struct.start_y = y;

	UI.ext_client_custom_graphic_float.grapic_data_struct.radius = float_temp & 0x3FF;        //< 低10位
	UI.ext_client_custom_graphic_float.grapic_data_struct.end_x =  (float_temp>>10) & 0x7FF;	 //< 中11位
	UI.ext_client_custom_graphic_float.grapic_data_struct.end_y =  float_temp>>21;						 //< 高11位
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_float, 28, 0xFFFF);
	UI.ext_client_custom_graphic_float.CRC16[0] = crc16_temp & 0xFF;
	UI.ext_client_custom_graphic_float.CRC16[1] = crc16_temp >> 8;
	
	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_float,sizeof(UI.ext_client_custom_graphic_float));
	
	HAL_UART_Transmit_DMA(&huart3,Data_Pack,sizeof(UI.ext_client_custom_graphic_float));
	
//	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&UI.ext_client_custom_graphic_float,30);
}


/**
  * @brief  在8号图层画整数的程序
	*/
void referee_draw_int(uint8_t robot_id,int32_t int_data,uint8_t int_index, uint8_t control_way, uint8_t color, uint16_t x, uint16_t y)
{
	uint16_t crc16_temp;
	UI.ext_client_custom_graphic_int.frame_header.SOF = 0xA5;
	UI.ext_client_custom_graphic_int.frame_header.data_length = 21;
	UI.ext_client_custom_graphic_int.frame_header.seq = 0;
	UI.ext_client_custom_graphic_int.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_int.frame_header,4,0xFF); 
	UI.ext_client_custom_graphic_int.cmd_id = 0x0301;
	UI.ext_client_custom_graphic_int.ext_student_interactive_header_data.data_cmd_id = 0x0101;
	UI.ext_client_custom_graphic_int.ext_student_interactive_header_data.sender_ID = robot_id;
	UI.ext_client_custom_graphic_int.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	UI.ext_client_custom_graphic_int.grapic_data_struct.graphic_name[0]=int_index;
	UI.ext_client_custom_graphic_int.grapic_data_struct.graphic_name[1] = 0x0;     //< "A" 97
	UI.ext_client_custom_graphic_int.grapic_data_struct.graphic_name[2]= 0x0;      //< "A"
	UI.ext_client_custom_graphic_int.grapic_data_struct.operate_tpye = control_way;
	UI.ext_client_custom_graphic_int.grapic_data_struct.graphic_tpye = 0x06;				//< 整形为06
	UI.ext_client_custom_graphic_int.grapic_data_struct.layer = 8;
	UI.ext_client_custom_graphic_int.grapic_data_struct.color = color;
	UI.ext_client_custom_graphic_int.grapic_data_struct.start_angle = 18;
	UI.ext_client_custom_graphic_int.grapic_data_struct.end_angle = 0x0;      			//< 小数位有效个数
	UI.ext_client_custom_graphic_int.grapic_data_struct.width = 2;
	UI.ext_client_custom_graphic_int.grapic_data_struct.start_x = x;
	UI.ext_client_custom_graphic_int.grapic_data_struct.start_y = y;
	UI.ext_client_custom_graphic_int.grapic_data_struct.radius = int_data & 0x3FF;        //< 低10位
	UI.ext_client_custom_graphic_int.grapic_data_struct.end_x =  (int_data>>10) & 0x7FF;	 //< 中11位
	UI.ext_client_custom_graphic_int.grapic_data_struct.end_y =  int_data>>21;						 //< 高11位

	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_int, 28, 0xFFFF);
	UI.ext_client_custom_graphic_int.CRC16[0] = crc16_temp & 0xFF;
	UI.ext_client_custom_graphic_int.CRC16[1] = crc16_temp >> 8;
	
	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_int,sizeof(UI.ext_client_custom_graphic_int));
	
	HAL_UART_Transmit_DMA(&huart3,Data_Pack,sizeof(UI.ext_client_custom_graphic_int));
	
//	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&UI.ext_client_custom_graphic_int,30);
}


/**
  * @brief  在5号图层画线的程序
	*/
void referee_draw_line(uint8_t robot_id, uint8_t line_index, uint8_t control_way,uint8_t color,uint16_t x,uint16_t y,uint16_t end_x,uint16_t end_y)
{
	uint16_t crc16_temp;
	UI.ext_client_custom_graphic_line.frame_header.SOF = 0xA5;
	UI.ext_client_custom_graphic_line.frame_header.data_length = 21;
	UI.ext_client_custom_graphic_line.frame_header.seq = 0;
	UI.ext_client_custom_graphic_line.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_line.frame_header,4,0xFF); 
	UI.ext_client_custom_graphic_line.cmd_id = 0x0301;
	UI.ext_client_custom_graphic_line.ext_student_interactive_header_data.data_cmd_id = 0x0101;
	UI.ext_client_custom_graphic_line.ext_student_interactive_header_data.sender_ID = robot_id;
	UI.ext_client_custom_graphic_line.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	UI.ext_client_custom_graphic_line.grapic_data_struct.graphic_name[0]=line_index;
	UI.ext_client_custom_graphic_line.grapic_data_struct.graphic_name[1]=0x0;
	UI.ext_client_custom_graphic_line.grapic_data_struct.graphic_name[2]=0x0;
	UI.ext_client_custom_graphic_line.grapic_data_struct.operate_tpye = control_way;
	UI.ext_client_custom_graphic_line.grapic_data_struct.graphic_tpye = 0x00;
	UI.ext_client_custom_graphic_line.grapic_data_struct.layer = 5;
	UI.ext_client_custom_graphic_line.grapic_data_struct.color = color;
	UI.ext_client_custom_graphic_line.grapic_data_struct.start_angle = 0x0;
	UI.ext_client_custom_graphic_line.grapic_data_struct.end_angle = 0x0;     //< 小数位有效个数
	UI.ext_client_custom_graphic_line.grapic_data_struct.width = 2;
	UI.ext_client_custom_graphic_line.grapic_data_struct.start_x = x;  
	UI.ext_client_custom_graphic_line.grapic_data_struct.start_y = y;
	UI.ext_client_custom_graphic_line.grapic_data_struct.radius = 0x0;        //< 低10位
	UI.ext_client_custom_graphic_line.grapic_data_struct.end_x =  end_x;	     //< 中11位
	UI.ext_client_custom_graphic_line.grapic_data_struct.end_y =  end_y;			 //< 高11位
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_line, 28, 0xFFFF);
	UI.ext_client_custom_graphic_line.CRC16[0] = crc16_temp & 0xFF;
	UI.ext_client_custom_graphic_line.CRC16[1] = crc16_temp >> 8;
	
	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_line,sizeof(UI.ext_client_custom_graphic_line));
	
	HAL_UART_Transmit_DMA(&huart3,Data_Pack,sizeof(UI.ext_client_custom_graphic_line));
	
//	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_custom_graphic_line,30);
}

/**
  * @brief  在2号图层画圆的程序
	*/
void referee_draw_circle(uint8_t robot_id,uint8_t circle_index, uint8_t control_way,uint8_t color, uint8_t layer, uint16_t x,uint16_t y, uint16_t R)
{
	uint16_t crc16_temp;
	UI.ext_client_custom_graphic_circle.frame_header.SOF = 0xA5;
	UI.ext_client_custom_graphic_circle.frame_header.data_length = 21;
	UI.ext_client_custom_graphic_circle.frame_header.seq = 0;
	UI.ext_client_custom_graphic_circle.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_circle.frame_header,4,0xFF); 
	UI.ext_client_custom_graphic_circle.cmd_id = 0x0301;
	UI.ext_client_custom_graphic_circle.ext_student_interactive_header_data.data_cmd_id = 0x0101;
	UI.ext_client_custom_graphic_circle.ext_student_interactive_header_data.sender_ID = robot_id;
	UI.ext_client_custom_graphic_circle.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	UI.ext_client_custom_graphic_circle.grapic_data_struct.graphic_name[0]=circle_index;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.graphic_name[1] = 0x0;  //< "A" 97
	UI.ext_client_custom_graphic_circle.grapic_data_struct.graphic_name[2]= 0x0;    //< "A"
	UI.ext_client_custom_graphic_circle.grapic_data_struct.operate_tpye = control_way;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.graphic_tpye = 2;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.layer = layer;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.color = color;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.start_angle = 18;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.end_angle = 2;      								 //< 小数位有效个数
	UI.ext_client_custom_graphic_circle.grapic_data_struct.width = 2;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.start_x = x;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.start_y = y;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.radius = R;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.end_x = 0;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.end_y = 0;
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_circle, 28, 0xFFFF);
	UI.ext_client_custom_graphic_circle.CRC16[0] = crc16_temp & 0xFF;
	UI.ext_client_custom_graphic_circle.CRC16[1] = crc16_temp >> 8;
	
	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_circle,sizeof(UI.ext_client_custom_graphic_circle));
	
	HAL_UART_Transmit_DMA(&huart3,Data_Pack,sizeof(UI.ext_client_custom_graphic_circle));
	
//	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_custom_graphic_circle,30);
}


/**
  * @brief  在2号图层画圆的程序
	*/
void referee_draw_circle_shoot(uint8_t robot_id,uint8_t circle_index, uint8_t control_way, uint16_t x,uint16_t y)
{
	uint16_t crc16_temp;
	UI.ext_client_custom_graphic_circle.frame_header.SOF = 0xA5;
	UI.ext_client_custom_graphic_circle.frame_header.data_length = 21;
	UI.ext_client_custom_graphic_circle.frame_header.seq = 0;
	UI.ext_client_custom_graphic_circle.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_circle.frame_header,4,0xFF); 
	UI.ext_client_custom_graphic_circle.cmd_id = 0x0301;
	UI.ext_client_custom_graphic_circle.ext_student_interactive_header_data.data_cmd_id = 0x0101;
	UI.ext_client_custom_graphic_circle.ext_student_interactive_header_data.sender_ID = robot_id;
	UI.ext_client_custom_graphic_circle.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	UI.ext_client_custom_graphic_circle.grapic_data_struct.graphic_name[0]=circle_index;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.graphic_name[1] = 0x0;  //< "A" 97
	UI.ext_client_custom_graphic_circle.grapic_data_struct.graphic_name[2]= 0x0;    //< "A"
	UI.ext_client_custom_graphic_circle.grapic_data_struct.operate_tpye = control_way;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.graphic_tpye = 2;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.layer = 7;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.color = YELLOW;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.start_angle = 18;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.end_angle = 2;      								 //< 小数位有效个数
	UI.ext_client_custom_graphic_circle.grapic_data_struct.width = 2;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.start_x = x;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.start_y = y;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.radius = 16;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.end_x = 0;
	UI.ext_client_custom_graphic_circle.grapic_data_struct.end_y = 0;
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_circle, 28, 0xFFFF);
	UI.ext_client_custom_graphic_circle.CRC16[0] = crc16_temp & 0xFF;
	UI.ext_client_custom_graphic_circle.CRC16[1] = crc16_temp >> 8;
	
	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_circle,sizeof(UI.ext_client_custom_graphic_circle));
	
	HAL_UART_Transmit_DMA(&huart3,Data_Pack,sizeof(UI.ext_client_custom_graphic_circle));
	
//	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&UI.ext_client_custom_graphic_circle,30);
}


/**
  * @brief  在2号图层画矩形的程序
	*/
void referee_draw_rectangle(uint8_t robot_id,uint8_t circle_index, uint8_t control_way,uint8_t color, uint8_t layer, uint16_t x,uint16_t y, uint16_t end_x, uint16_t end_y)
{
	uint16_t crc16_temp;
	UI.ext_client_custom_graphic_rectangle.frame_header.SOF = 0xA5;
	UI.ext_client_custom_graphic_rectangle.frame_header.data_length = 21;
	UI.ext_client_custom_graphic_rectangle.frame_header.seq = 0;
	UI.ext_client_custom_graphic_rectangle.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_rectangle.frame_header,4,0xFF); 
	UI.ext_client_custom_graphic_rectangle.cmd_id = 0x0301;
	UI.ext_client_custom_graphic_rectangle.ext_student_interactive_header_data.data_cmd_id = 0x0101;
	UI.ext_client_custom_graphic_rectangle.ext_student_interactive_header_data.sender_ID = robot_id;
	UI.ext_client_custom_graphic_rectangle.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	UI.ext_client_custom_graphic_rectangle.grapic_data_struct.graphic_name[0]=circle_index;
	UI.ext_client_custom_graphic_rectangle.grapic_data_struct.graphic_name[1] = 0x0;  //< "A" 97
	UI.ext_client_custom_graphic_rectangle.grapic_data_struct.graphic_name[2]= 0x0;    //< "A"
	UI.ext_client_custom_graphic_rectangle.grapic_data_struct.operate_tpye = control_way;
	UI.ext_client_custom_graphic_rectangle.grapic_data_struct.graphic_tpye = 1;
	UI.ext_client_custom_graphic_rectangle.grapic_data_struct.layer = layer;
	UI.ext_client_custom_graphic_rectangle.grapic_data_struct.color = color;
	UI.ext_client_custom_graphic_rectangle.grapic_data_struct.start_angle = 18;
	UI.ext_client_custom_graphic_rectangle.grapic_data_struct.end_angle = 2;      								 //< 小数位有效个数
	UI.ext_client_custom_graphic_rectangle.grapic_data_struct.width = 2;
	UI.ext_client_custom_graphic_rectangle.grapic_data_struct.start_x = x;
	UI.ext_client_custom_graphic_rectangle.grapic_data_struct.start_y = y;
	UI.ext_client_custom_graphic_rectangle.grapic_data_struct.radius = 16;
	UI.ext_client_custom_graphic_rectangle.grapic_data_struct.end_x = end_x;
	UI.ext_client_custom_graphic_rectangle.grapic_data_struct.end_y = end_y;
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_rectangle, 28, 0xFFFF);
	UI.ext_client_custom_graphic_rectangle.CRC16[0] = crc16_temp & 0xFF;
	UI.ext_client_custom_graphic_rectangle.CRC16[1] = crc16_temp >> 8;
	
	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_rectangle,sizeof(UI.ext_client_custom_graphic_rectangle));
	
	HAL_UART_Transmit_DMA(&huart3,Data_Pack,sizeof(UI.ext_client_custom_graphic_rectangle));
//	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&UI.ext_client_custom_graphic_rectangle,30);
}

/**
  * @brief  画椭圆
	*/
void referee_draw_ellipe(uint8_t robot_id,uint8_t circle_index, uint8_t control_way,uint8_t color, uint8_t layer, uint16_t x,uint16_t y)
{
	uint16_t crc16_temp;
	UI.ext_client_custom_graphic_arc.frame_header.SOF = 0xA5;
	UI.ext_client_custom_graphic_arc.frame_header.data_length = 21;
	UI.ext_client_custom_graphic_arc.frame_header.seq = 0;
	UI.ext_client_custom_graphic_arc.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_arc.frame_header,4,0xFF); 
	UI.ext_client_custom_graphic_arc.cmd_id = 0x0301;
	UI.ext_client_custom_graphic_arc.ext_student_interactive_header_data.data_cmd_id = 0x0101;
	UI.ext_client_custom_graphic_arc.ext_student_interactive_header_data.sender_ID = robot_id;
	UI.ext_client_custom_graphic_arc.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	UI.ext_client_custom_graphic_arc.grapic_data_struct.graphic_name[0]=circle_index;
  UI.ext_client_custom_graphic_arc.grapic_data_struct.graphic_name[1] = 0x0;  //< "A" 97
	UI.ext_client_custom_graphic_arc.grapic_data_struct.graphic_name[2]= 0x0;    //< "A"
	UI.ext_client_custom_graphic_arc.grapic_data_struct.operate_tpye = control_way;
	UI.ext_client_custom_graphic_arc.grapic_data_struct.graphic_tpye = 3;
	UI.ext_client_custom_graphic_arc.grapic_data_struct.layer = layer;
	UI.ext_client_custom_graphic_arc.grapic_data_struct.color = color;
	UI.ext_client_custom_graphic_arc.grapic_data_struct.start_angle = 18;
	UI.ext_client_custom_graphic_arc.grapic_data_struct.end_angle = 2;      								 //< 小数位有效个数
	UI.ext_client_custom_graphic_arc.grapic_data_struct.width = 2;
	UI.ext_client_custom_graphic_arc.grapic_data_struct.start_x = x;
	UI.ext_client_custom_graphic_arc.grapic_data_struct.start_y = y;
	UI.ext_client_custom_graphic_arc.grapic_data_struct.radius = 16;
	UI.ext_client_custom_graphic_arc.grapic_data_struct.end_x = 0;
	UI.ext_client_custom_graphic_arc.grapic_data_struct.end_y = 0;
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_arc, 28, 0xFFFF);
	UI.ext_client_custom_graphic_arc.CRC16[0] = crc16_temp & 0xFF;
	UI.ext_client_custom_graphic_arc.CRC16[1] = crc16_temp >> 8;
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&UI.ext_client_custom_graphic_arc,30);
}

/**
  * @brief  画圆弧
	*/
void referee_draw_arc(uint8_t robot_id,uint8_t circle_index, uint8_t control_way,uint8_t color, uint8_t layer, uint16_t x,uint16_t y)
{
	uint16_t crc16_temp;
	UI.ext_client_custom_graphic_ellipe.frame_header.SOF = 0xA5;
	UI.ext_client_custom_graphic_ellipe.frame_header.data_length = 21;
	UI.ext_client_custom_graphic_ellipe.frame_header.seq = 0;
	UI.ext_client_custom_graphic_ellipe.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_ellipe.frame_header,4,0xFF); 
	UI.ext_client_custom_graphic_ellipe.cmd_id = 0x0301;
	UI.ext_client_custom_graphic_ellipe.ext_student_interactive_header_data.data_cmd_id = 0x0101;
	UI.ext_client_custom_graphic_ellipe.ext_student_interactive_header_data.sender_ID = robot_id;
	UI.ext_client_custom_graphic_ellipe.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	UI.ext_client_custom_graphic_ellipe.grapic_data_struct.graphic_name[0]=circle_index;
	UI.ext_client_custom_graphic_ellipe.grapic_data_struct.graphic_name[1] = 0x0;  //< "A" 97
	UI.ext_client_custom_graphic_ellipe.grapic_data_struct.graphic_name[2]= 0x0;    //< "A"
	UI.ext_client_custom_graphic_ellipe.grapic_data_struct.operate_tpye = control_way;
	UI.ext_client_custom_graphic_ellipe.grapic_data_struct.graphic_tpye = 4;
	UI.ext_client_custom_graphic_ellipe.grapic_data_struct.layer = layer;
	UI.ext_client_custom_graphic_ellipe.grapic_data_struct.color = color;
	UI.ext_client_custom_graphic_ellipe.grapic_data_struct.start_angle = 180;
	UI.ext_client_custom_graphic_ellipe.grapic_data_struct.end_angle = 0;      								 //< 小数位有效个数
	UI.ext_client_custom_graphic_ellipe.grapic_data_struct.width = 2;
	UI.ext_client_custom_graphic_ellipe.grapic_data_struct.start_x = x;
	UI.ext_client_custom_graphic_ellipe.grapic_data_struct.start_y = y;
	UI.ext_client_custom_graphic_ellipe.grapic_data_struct.radius = 0;
	UI.ext_client_custom_graphic_ellipe.grapic_data_struct.end_x = 20;
	UI.ext_client_custom_graphic_ellipe.grapic_data_struct.end_y = 8;
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_ellipe, 28, 0xFFFF);
	UI.ext_client_custom_graphic_ellipe.CRC16[0] = crc16_temp & 0xFF;
	UI.ext_client_custom_graphic_ellipe.CRC16[1] = crc16_temp >> 8;
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&UI.ext_client_custom_graphic_ellipe,30);
}

/**
  * @brief  发送组合图形所需的函数
	*/
void draw_graph(graphic_data_struct_t *patterning, uint16_t index, uint8_t control_way, uint8_t graph, uint8_t layer, uint8_t color, uint8_t Sa, uint8_t Ea, uint8_t With,  uint16_t x,uint16_t y, uint8_t R, uint16_t Ex, uint16_t Ey)
{
	(*patterning).graphic_name[0]=index;
	(*patterning).graphic_name[1]=0;
	(*patterning).graphic_name[2]=0;
	(*patterning).operate_tpye=control_way;
	(*patterning).graphic_tpye=graph;
	(*patterning).layer=layer;
	(*patterning).color=color;
	(*patterning).start_angle=Sa;
	(*patterning).end_angle=Ea;
	(*patterning).width=With;
	(*patterning).start_x=x;
	(*patterning).start_y=y;
	(*patterning).radius=R;
	(*patterning).end_x=Ex;
	(*patterning).end_y=Ey;
}



/**
  * @brief  关于NUC和弹舱的图形
	*/
void referee_draw_nuc(uint8_t robot_id, uint8_t control_line2, uint8_t control_rectangle, uint8_t control_line, uint8_t control_hatch)
{
	uint16_t crc16_temp;
	UI.ext_client_custom_graphic_patterning.frame_header.SOF = 0xA5;
	UI.ext_client_custom_graphic_patterning.frame_header.data_length =  81;
	UI.ext_client_custom_graphic_patterning.frame_header.seq = 0;
	UI.ext_client_custom_graphic_patterning.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning.frame_header,4,0xFF); 
	UI.ext_client_custom_graphic_patterning.cmd_id = 0x0301;
	UI.ext_client_custom_graphic_patterning.ext_student_interactive_header_data.data_cmd_id = 0x0103;
	UI.ext_client_custom_graphic_patterning.ext_student_interactive_header_data.sender_ID = robot_id;
	UI.ext_client_custom_graphic_patterning.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	
	draw_graph(&UI.ext_client_custom_graphic_patterning.grapic_data_struct[0],97,control_line2,LINE,2,color_test.NUC_color1,0,0,2,1770,780,0,1770,860);           //NUC自瞄模式
	draw_graph(&UI.ext_client_custom_graphic_patterning.grapic_data_struct[1],96,control_rectangle,RECTANGLE,2,color_test.NUC_color2,0,0,2,1770,810,0,1785,860);  //NUC自瞄模式
	draw_graph(&UI.ext_client_custom_graphic_patterning.grapic_data_struct[2],95,control_line,LINE,2,color_test.NUC_color3,0,0,2,1720,820,0,1820,820);          //NUC自瞄模式
	draw_graph(&UI.ext_client_custom_graphic_patterning.grapic_data_struct[3],94,control_hatch,RECTANGLE,2,color_test.hatch_color,0,0,4,1770,690,0,1840,660);       //弹舱开关
	draw_graph(&UI.ext_client_custom_graphic_patterning.grapic_data_struct[4],93,ADD,RECTANGLE,2,CYAN_BLUE,0,0,2,1770,870,0,1820,900);             //NUC相机
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning, 88, 0xFFFF);
	UI.ext_client_custom_graphic_patterning.CRC16[0] = crc16_temp & 0xFF;
	UI.ext_client_custom_graphic_patterning.CRC16[1] = crc16_temp >> 8;
	
	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_patterning,sizeof(UI.ext_client_custom_graphic_patterning));
	
	HAL_UART_Transmit_DMA(&huart3,Data_Pack,90);
}


/**
  * @brief  关于电容的图形
	*/
void referee_draw_cap(uint8_t robot_id, uint8_t control_line_cap, uint8_t control_line_charge, uint8_t control_cap_lenghth, float VOLT)
{
	uint16_t crc16_temp;
	UI.ext_client_custom_graphic_patterning_second.frame_header.SOF = 0xA5;
	UI.ext_client_custom_graphic_patterning_second.frame_header.data_length =  81;
	UI.ext_client_custom_graphic_patterning_second.frame_header.seq = 0;
	UI.ext_client_custom_graphic_patterning_second.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_second.frame_header,4,0xFF); 
	UI.ext_client_custom_graphic_patterning_second.cmd_id = 0x0301;
	UI.ext_client_custom_graphic_patterning_second.ext_student_interactive_header_data.data_cmd_id = 0x0103;
	UI.ext_client_custom_graphic_patterning_second.ext_student_interactive_header_data.sender_ID = robot_id;
	UI.ext_client_custom_graphic_patterning_second.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	
	draw_graph(&UI.ext_client_custom_graphic_patterning_second.grapic_data_struct[0],92,control_line_cap,LINE,2,color_test.capmode_color,0,0,20,1600,450,0,1650,450);   //电容开关//更改成重力补偿准星
	draw_graph(&UI.ext_client_custom_graphic_patterning_second.grapic_data_struct[1],91,control_line_charge,LINE,2,color_test.charge_color,0,0,20,1700,450,0,1750,450);  //充电开关//要更改成重力补偿准星
	draw_graph(&UI.ext_client_custom_graphic_patterning_second.grapic_data_struct[2],90,ADD,RECTANGLE,2,color_test.cap_rectangle_color,0,0,2,1600,400,0,1800,430);  //电容框
	if(VOLT<50)
		draw_graph(&UI.ext_client_custom_graphic_patterning_second.grapic_data_struct[3],89,control_cap_lenghth,LINE,2,color_test.cap_rectangle_color,0,0,30,1600,415,0,(1600+(VOLT-LowVolt)*200.0/(HighVolt-LowVolt)),415);     //电容百分比
	draw_graph(&UI.ext_client_custom_graphic_patterning_second.grapic_data_struct[4],88,ADD,RECTANGLE,2,CYAN_BLUE,0,0,2,1770,900,0,1785,905);    //相机
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_second, 88, 0xFFFF);
	UI.ext_client_custom_graphic_patterning_second.CRC16[0] = crc16_temp & 0xFF;
	UI.ext_client_custom_graphic_patterning_second.CRC16[1] = crc16_temp >> 8;
	
	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_patterning_second,sizeof(UI.ext_client_custom_graphic_patterning_second));
	
	HAL_UART_Transmit_DMA(&huart3,Data_Pack,90);
}


/**
  * @brief  关于准星的图形
	*/
void referee_draw_shoot(uint8_t robot_id)
{
	uint16_t crc16_temp;
	UI.ext_client_custom_graphic_patterning_third.frame_header.SOF = 0xA5;
	UI.ext_client_custom_graphic_patterning_third.frame_header.data_length =  111;
	UI.ext_client_custom_graphic_patterning_third.frame_header.seq = 0;
	UI.ext_client_custom_graphic_patterning_third.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_third.frame_header,4,0xFF); 
	UI.ext_client_custom_graphic_patterning_third.cmd_id = 0x0301;
	UI.ext_client_custom_graphic_patterning_third.ext_student_interactive_header_data.data_cmd_id = 0x0104;
	UI.ext_client_custom_graphic_patterning_third.ext_student_interactive_header_data.sender_ID = robot_id;
	UI.ext_client_custom_graphic_patterning_third.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	
	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[0],78,ADD,LINE,2,color_test.shoot_color,0,0,2,500,0,0,660,200);      //车间线
	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[1],77,ADD,LINE,2,color_test.shoot_color,0,0,2,660,200,0,1250,200);      //车间线
	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[2],76,ADD,LINE,2,color_test.shoot_color,0,0,2,1250,200,0,1400,0);      //车间线
	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[3],75,ADD,LINE,2,color_test.shoot_color,0,0,2,960,650,0,960,400);  	 	 //准线
	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[4],74,ADD,LINE,2,color_test.shoot_color,0,0,2,932,390,0,982,390);      //准线
	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[5],73,ADD,LINE,2,color_test.shoot_color,0,0,2,932,490,0,982,490);			 //准线
	draw_graph(&UI.ext_client_custom_graphic_patterning_third.grapic_data_struct[6],72,ADD,LINE,2,color_test.shoot_color,0,0,2,932,440,0,982,440);      //准线
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_third, 118, 0xFFFF);
	UI.ext_client_custom_graphic_patterning_third.CRC16[0] = crc16_temp & 0xFF;
	UI.ext_client_custom_graphic_patterning_third.CRC16[1] = crc16_temp >> 8;
	
	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_patterning_third,sizeof(UI.ext_client_custom_graphic_patterning_third));
	
	HAL_UART_Transmit_DMA(&huart3,Data_Pack,120);
}


/**
  * @brief  关于准星的图形
	*/
void referee_draw_robot(uint8_t robot_id, uint8_t level, uint8_t state, uint8_t danger, uint8_t control_level, uint8_t control_state, uint8_t control_danger)
{
	uint16_t crc16_temp;
	UI.ext_client_custom_graphic_patterning_fourth.frame_header.SOF = 0xA5;
	UI.ext_client_custom_graphic_patterning_fourth.frame_header.data_length =  111;
	UI.ext_client_custom_graphic_patterning_fourth.frame_header.seq = 0;
	UI.ext_client_custom_graphic_patterning_fourth.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_fourth.frame_header,4,0xFF); 
	UI.ext_client_custom_graphic_patterning_fourth.cmd_id = 0x0301;
	UI.ext_client_custom_graphic_patterning_fourth.ext_student_interactive_header_data.data_cmd_id = 0x0104;
	UI.ext_client_custom_graphic_patterning_fourth.ext_student_interactive_header_data.sender_ID = robot_id;
	UI.ext_client_custom_graphic_patterning_fourth.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	
	draw_graph(&UI.ext_client_custom_graphic_patterning_fourth.grapic_data_struct[0],70,control_level,INT,8,RED_BLUE,18,0,2,1720,860,level,0,0);      //车间线
	draw_graph(&UI.ext_client_custom_graphic_patterning_fourth.grapic_data_struct[1],69,control_state,INT,8,RED_BLUE,18,0,2,1740,860,state,0,0);      //车间线
	draw_graph(&UI.ext_client_custom_graphic_patterning_fourth.grapic_data_struct[2],68,ADD,INT,8,RED_BLUE,18,0,2,1620,780,1,0,0);      //车间线
	draw_graph(&UI.ext_client_custom_graphic_patterning_fourth.grapic_data_struct[3],67,ADD,INT,8,RED_BLUE,18,0,2,1670,780,2,0,0);  	 	 //准线
	draw_graph(&UI.ext_client_custom_graphic_patterning_fourth.grapic_data_struct[4],66,ADD,INT,8,RED_BLUE,18,0,2,1720,780,3,0,0);      //准线
	draw_graph(&UI.ext_client_custom_graphic_patterning_fourth.grapic_data_struct[5],65,ADD,INT,8,RED_BLUE,18,0,2,1770,780,4,0,0);			 //准线
	draw_graph(&UI.ext_client_custom_graphic_patterning_fourth.grapic_data_struct[6],64,control_danger,INT,8,RED_BLUE,18,0,2,1820,780,danger,0,0);      //准线
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&UI.ext_client_custom_graphic_patterning_fourth, 118, 0xFFFF);
	UI.ext_client_custom_graphic_patterning_fourth.CRC16[0] = crc16_temp & 0xFF;
	UI.ext_client_custom_graphic_patterning_fourth.CRC16[1] = crc16_temp >> 8;
	
	memcpy(Data_Pack,(unsigned char*)&UI.ext_client_custom_graphic_patterning_fourth,sizeof(UI.ext_client_custom_graphic_patterning_fourth));
	
	HAL_UART_Transmit_DMA(&huart3,Data_Pack,120);
}

/**
  * @brief  画NUC
	*/
void referee_draw_NUC_data(uint32_t cnt , uint8_t robot_id, uint8_t camera_state, uint8_t target_state, uint16_t mode, uint8_t hatch_state)
{
	char *fight="Fingting";
	
	char*manual="MANUAL";                 //手动把射击模式
	char*shootone="SHOOTONE";                //优先攻打1号车
	char*shoottwo="SHOOTTWO";               //优先攻打2号车
	char*shootthree="SHOOTTHREE";              //优先攻打3号车
	char*shootfour="SHOOTFOUR";              //优先攻打4号车
	char*shootfive="SHOOTFIVE";               //优先攻打5号车
	char*shootsentry="SHOOTSENTRY";            //优先攻打哨兵
	char*shootoutpoat="SHOOTOUTPOST";           //优先攻打前哨站
	char*shootbase="SHOOTBASE";               //优先攻打基地
	char*automatichit="AUTOMATICHIT";            //自动射击模式
	char*curvedfireoutpost="CURVEDFIREOUTPOST";      //吊射前哨站
	char*curvedfirebase="CURVEDFIREBASE";         //吊射基地
	char*shootsmallbuff="SHOOTSMALLBUFF";         //打小符
	char*shootlarebuff="SHOOTLARGEBUFF";         //打大符
	
	if(cnt%100 == 0 && (cnt < 3000))
	{
		referee_draw_circle(robot_id, 87, ADD, CYAN_BLUE, 0, 1805, 885, 8);
	}
	if(cnt%100 == 10 && (cnt < 3000))
	{
		referee_draw_int(robot_id, mode, 86, ADD, RED_BLUE, 1600, 890);   
	}
	if(cnt%100==70&&(cnt < 3000))
	{
		referee_draw_string(robot_id, automatichit, 57,ADD, YELLOW, 3, 1400, 890);
	}
	
		
	
	if(cnt%500 == 50 && (cnt > 3000))
	{		
		if(camera_state==0)                                         						//0关相机
		  referee_draw_circle(robot_id, 87, DELETE, CYAN_BLUE, 0, 1805, 885, 8);
		else if(camera_state==1)                                   							//1开相机
		  referee_draw_circle(robot_id, 87, ADD, CYAN_BLUE, 0, 1805, 885, 8);
	}
	if(cnt%100==20&&(cnt>3000))
	{
		if(target_state==1&&hatch_state==0)                                        						 //1打符
			referee_draw_nuc(robot_id,ADD,ADD,DELETE,DELETE);
		if(target_state==1&&hatch_state==1)
			referee_draw_nuc(robot_id,ADD,ADD,DELETE,ADD);
		if(target_state==2&&hatch_state==0)                                   						 //2自瞄
			referee_draw_nuc(robot_id,ADD,DELETE,ADD,DELETE);
		if(target_state==2&&hatch_state==1)
			referee_draw_nuc(robot_id,ADD,DELETE,ADD,ADD);
		if(target_state!=1&&target_state!=2&&hatch_state==0)
			referee_draw_nuc(robot_id,DELETE,DELETE,DELETE,DELETE);
		if(target_state!=1&&target_state!=2&&hatch_state==1)
			referee_draw_nuc(robot_id,DELETE,DELETE,DELETE,ADD);
	}

	if(cnt%100==10&&(cnt>3000))
			referee_draw_int(robot_id, mode, 86, MODIFY, RED_BLUE, 1600, 890);
	if(cnt%100 == 70 && (cnt > 3000))
	{
		if(mode==0)        																													 //击打1号机器人
			referee_draw_string(robot_id, manual, 57, MODIFY, RED_BLUE,3, 1400, 890);
		else if(mode==1)        																													 //击打1号机器人
			referee_draw_string(robot_id, shootone, 57, MODIFY, RED_BLUE, 3, 1400, 890);
		else if(mode==2)																													  	 //击打2号机器人
			referee_draw_string(robot_id, shoottwo, 57, MODIFY, RED_BLUE, 3, 1400, 890);
		else if(mode==3) 																														//击打3号机器人
			referee_draw_string(robot_id, shootthree, 57, MODIFY, RED_BLUE,3, 1400, 890);
		else if(mode==4)																															//击打4号机器人
			referee_draw_string(robot_id, shootfour, 57, MODIFY, RED_BLUE, 3, 1400, 890);
		else if(mode==5)																															//击打5号机器人
			referee_draw_string(robot_id, shootfive, 57, MODIFY, RED_BLUE, 3, 1400, 890);
		else if(mode==6)																															//击打哨兵机器人
			referee_draw_string(robot_id, shootsentry, 57, MODIFY, RED_BLUE, 3, 1400, 890);
		else if(mode==7)																															//射击前哨战
			referee_draw_string(robot_id, shootoutpoat, 57, MODIFY, RED_BLUE,3, 1400, 890);
		else if(mode==8)																															//射击基地 
			referee_draw_string(robot_id, shootbase, 57, MODIFY, RED_BLUE,3, 1400, 890);
		else if(mode==9)																															//吊射前哨战
			referee_draw_string(robot_id, automatichit, 57, MODIFY, RED_BLUE, 3, 1400, 890);
		else if(mode==10)																															//吊射基地
			referee_draw_string(robot_id, curvedfireoutpost, 57, MODIFY, RED_BLUE, 3, 1400, 890);
		else if(mode==11)
			referee_draw_string(robot_id, curvedfirebase, 57, MODIFY, RED_BLUE, 3, 1400, 890);
		else if(mode==12)
			referee_draw_string(robot_id, shootsmallbuff, 57, MODIFY, RED_BLUE, 3, 1400, 890);
		else if(mode==13)
			referee_draw_string(robot_id, shootlarebuff, 57, MODIFY, RED_BLUE, 3, 1400, 890);
	}
}



/**
  * @brief  画发射机构
	*/
void referee_draw_Load_data(uint32_t cnt , uint8_t robot_id, uint8_t hatch_state)
{	
	if(cnt%100 == 30 && (cnt > 3000))   //< 发送频率被降低为200Hz
	{
		if(hatch_state==0)                                                   //0是关弹舱
			referee_draw_nuc(robot_id, NONE, NONE, NONE, DELETE);
		else if(hatch_state==1)																							 //1是开弹舱
			referee_draw_nuc(robot_id, NONE, NONE, NONE, ADD);
	}
}


/**
  * @brief  画底盘
	*/
void referee_draw_chassis_data(uint32_t cnt , uint8_t robot_id, uint8_t move_state, uint8_t power_state, uint8_t heat_state, float pitch_angle)
{
	char *pitch = "Pitch";
		
	if(cnt%100 == 40 && (cnt > 3000))
	{
		if(move_state==0)                          	                          //0是自旋
			referee_draw_circle(robot_id, 85, ADD, color_test.rotate_color, 2, 1770, 500, 25);
		else 
			referee_draw_circle(robot_id, 85, DELETE, color_test.rotate_color, 2, 1770, 500, 25);
	}
	
	if(cnt%100==50){
		referee_draw_string(robot_id, pitch, 60,ADD, YELLOW, 3, 1200, 400);
	}
	if(cnt%100==60 && cnt<3000){
		referee_draw_float(robot_id, pitch_angle, 63, ADD, YELLOW, 4, 1300,400);
	}
		
	
	if(cnt%100==60 && cnt>3000){
		referee_draw_float(robot_id, pitch_angle, 63, MODIFY, YELLOW, 4, 1300,400);
	}
	
}


/**
  * @brief  画电容
	*/
void referee_draw_supercap_data(uint32_t cnt, uint8_t robot_id, uint8_t cap_state, uint8_t charge_state, float capvolt)
{
	if(cnt%100 == 80 && (cnt < 3000))
	{
		referee_draw_cap(robot_id, NONE, NONE, ADD, capvolt);
	}


	if(cnt%100 == 80 && (cnt >= 3000))
	{
	if(cap_state==0 && charge_state==0)                                                 //关电容不充电
			referee_draw_cap(robot_id, DELETE, DELETE, MODIFY, capvolt);
		else if(cap_state==0 && charge_state==1)                                            //关电容充电
			referee_draw_cap(robot_id, DELETE, ADD, MODIFY, capvolt);
		else if(cap_state==1 && charge_state==0) 																						//开电容不充电
			referee_draw_cap(robot_id, ADD, DELETE, MODIFY, capvolt);
		else if(cap_state==1 && charge_state==1)  																					//开电容充电
			referee_draw_cap(robot_id, ADD, ADD, MODIFY, capvolt);	
  }		
}


void referee_draw_shoot_data(uint32_t cnt , uint8_t robot_id, uint8_t mode, uint16_t x,uint16_t y)
{
//	if(cnt%250==0&&cnt<1000)
//	{
///*PPT以前比赛辅助瞄准*/		
////		referee_draw_circle(robot_id, 71, ADD, YELLOW, 0, 930, 400, 8);
///*PPT以前比赛辅助瞄准*/

//		
///*测试自瞄准度由视觉下发数据*/
////		referee_draw_circle_shoot(robot_id, 71, ADD, x, y);
///*测试自瞄准度由视觉下发数据*/
//		
//	}
	if(cnt%250==0)
	{
		
		referee_draw_shoot(robot_id);
/*PPT以前比赛辅助瞄准*/
//	if(mode==1)
//		referee_draw_circle(robot_id, 71, MODIFY, YELLOW, 0, 930, 460, 8);
//	else if(mode==2)
//		referee_draw_circle(robot_id, 71, MODIFY, YELLOW, 0, 920, 560, 16);
//	else if(mode==3)
//		referee_draw_circle(robot_id, 71, MODIFY, YELLOW, 0, 1120, 460, 32);
/*PPT以前比赛辅助瞄准*/
		
		
/*测试自瞄准度由视觉下发数据*/
//		referee_draw_circle_shoot(robot_id, 71, MODIFY, x, y);
/*测试自瞄准度由视觉下发数据*/
	}
}


/*PPT临时添加，不用*/
void draw_robotdata(uint32_t cnt , uint8_t robot_id, uint8_t level, uint8_t state, uint8_t danger, float float_data)
{
//	if(cnt%350==0&&cnt<1000)
//		referee_draw_robot(robot_id, level, state, danger, ADD,ADD,ADD);
//	
//	if(cnt%450==0&&cnt<1000)
//		referee_draw_float(robot_id, float_data, 63, ADD, RED_BLUE, 4, 1720,810);
//	
//	if(cnt%350==0&&cnt>1000)
//		referee_draw_robot(robot_id, level, state, danger, MODIFY,MODIFY,MODIFY);
	
	if(cnt%200==90&&cnt<1000)
		referee_draw_float(robot_id, float_data, 59, ADD, color_test.capvolt_color, 4, 300,820);
	
	if(cnt%200==90&&cnt>1000)
		referee_draw_float(robot_id, float_data, 59, MODIFY, color_test.capvolt_color, 4, 300,820);
}

	
	
/**
  * @brief  关于刷新后图形
	*/
void update(uint32_t cnt, uint8_t robot_id, uint8_t change)
{
	if(change)
	{
		if(cnt%350==0){
		referee_graphic_delete(0, ALL_delete, robot_id);
		cnt=0;
		}
	}
}


uint16_t referee_get_receiver_ID(uint32_t sender_id)
{
	switch(sender_id)
	{
		case RobotRedHero: return ClientRedHero; 
		case RobotRedEngineer: return ClientRedEngineer;
		case RobotRedInfantryNO3: return ClientRedInfantryNO3;
		case RobotRedInfantryNO4: return ClientRedInfantryNO4; 
		case RobotRedInfantryNO5: return ClientRedInfantryNO5; 
		case RobotRedAerial: return ClientRedAerial;
		case RobotBlueHero: return ClientBlueHero;
		case RobotBlueEngineer: return ClientBlueEngineer; 
		case RobotBlueInfantryNO3: return ClientBlueInfantryNO3;
		case RobotBlueInfantryNO4: return ClientBlueInfantryNO4; 
		case RobotBlueInfantryNO5: return ClientBlueInfantryNO5; 
		case RobotBlueAerial: return ClientBlueAerial; 
		default: return 0;
	}
}




const unsigned char CRC8__INIT = 0xff;
const unsigned char CRC8__TAB[256] =//8位最多256个
{
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
unsigned char Get_CRC8_Check(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)
{
	unsigned char ucIndex;                     //与0异或保持不变，与1异或反转
	while (dwLength--)   //ucCRC8是什么??
	{
		ucIndex = ucCRC8^(*pchMessage++);//第一次是取反?? 
		ucCRC8 = CRC8__TAB[ucIndex];//余式表             
	}                                                   
	return(ucCRC8);
}

uint16_t CRC__INIT = 0xffff;
const uint16_t wCRC__Table[256] =
{
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
	uint8_t chData;
	if (pchMessage == NULL)//无效地址
	{
		return 0xFFFF;
	}
	while(dwLength--)
	{
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC__Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) &0x00ff];
	}
	return wCRC;
}



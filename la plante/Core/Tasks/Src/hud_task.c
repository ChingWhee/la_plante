/*
 * hud_task.c
 *
 *  Created on: Jul 7, 2023
 *      Author: wx
 */

#include "board_lib.h"
#include "bsp_queue.h"
#include "bsp_referee.h"
#include "bsp_usart.h"
#include "hud_task.h"
#include "referee_msgs.h"
#include "robot_config.h"
#include "rtos_g_vars.h"

static uint16_t g_client_id = 0;
extern int g_spinspin_mode;
extern uint8_t remote_raw_data[18];
extern TaskHandle_t referee_processing_task_handle;
extern uint8_t g_ref_tx_seq;
extern uint8_t control_mode;
extern uint8_t safety_toggle;
extern uint8_t launcher_safety_toggle;
extern speed_shift_t gear_speed;
static uint8_t tx_buffer[256];

extern int diagonal_mode;
extern uint8_t is_diagonal;

extern ref_game_robot_data_t ref_robot_data;
ref_inter_robot_data_t graphic_header;
graphic_data_struct_t graphic_data;
ref_frame_header_t send_header;
uint8_t char_buffer[30];

void hud_task(void *argument) {

	enum drawings {
		spinspin, diagonal, robot_state, motor_fault, refresh
	};

	while (ref_robot_data.robot_id == 0) {
		vTaskDelay(10);
	}
	uint32_t refresh_timer = HAL_GetTick();

	uint8_t draw_state = spinspin;
	//draw all images
	vTaskDelay(1000);
	clear_hud();
	vTaskDelay(150);
	draw_spinspin(0);
	vTaskDelay(150);
	draw_diagonal(0);
	vTaskDelay(150);
	while (1) {
		if (HAL_GetTick() - refresh_timer > 5000){
			draw_state = refresh;
			refresh_timer = HAL_GetTick();
		}
		switch (ref_robot_data.robot_id) {
		case 1:
			g_client_id = 0x101;
			break;
		case 2:
			g_client_id = 0x102;
			break;
		case 3:
			g_client_id = 0x103;
			break;
		case 4:
			g_client_id = 0x104;
			break;
		case 5:
			g_client_id = 0x105;
			break;
		case 6:
			g_client_id = 0x106;
			break;

		case 101:
			g_client_id = 0x165;
			break;
		case 102:
			g_client_id = 0x166;
			break;
		case 103:
			g_client_id = 0x167;
			break;
		case 104:
			g_client_id = 0x168;
			break;
		case 105:
			g_client_id = 0x169;
			break;
		case 106:
			g_client_id = 0x16A;
			break;
		default:
			g_client_id = 0;
			break;
		}


		switch (draw_state) {
		case spinspin:
			draw_spinspin(1);
			draw_state = diagonal;
			break;
		case diagonal:
			draw_diagonal(1);
			draw_state = spinspin;
			break;
		case robot_state:
			break;
		case motor_fault:
			break;
		case refresh:
			clear_hud();
			vTaskDelay(150);
			draw_spinspin(0);
			vTaskDelay(150);
			draw_diagonal(0);
			vTaskDelay(150);
			draw_state = spinspin;
			break;
		default:
			break;
		}
		vTaskDelay(150);
	}
}

void draw_spinspin(uint8_t modify) {
	uint32_t curr_pos = 0;
	uint8_t char_len = 0;
	curr_pos = 0;
	if (g_spinspin_mode == 0) {
		graphic_data.color = 4; //purplish red
		char_len = snprintf((char*) char_buffer, 30, "SPIN OFF");
	} else {
		graphic_data.color = 3; //orange
		char_len = snprintf((char*) char_buffer, 30, "SPIN ON");
	}
	send_header.start_frame = 0xA5;
	send_header.cmd_id = REF_ROBOT_COMMS_CMD_ID;
	send_header.seq = g_ref_tx_seq++;
	send_header.data_length = sizeof(ref_inter_robot_data_t)
			+ sizeof(graphic_data_struct_t) + char_len;
	send_header.seq = g_ref_tx_seq++;
	memcpy(tx_buffer + curr_pos, &send_header, 7);
	curr_pos += sizeof(ref_frame_header_t);
	append_CRC8_check_sum(tx_buffer, 5);

	//for drawing 1 character graphic
	graphic_header.cmd_ID = 0x110;
	//send to self
	graphic_header.send_ID = ref_robot_data.robot_id;
	graphic_header.receiver_ID = g_client_id;
	memcpy(tx_buffer + curr_pos, &graphic_header,
			sizeof(ref_inter_robot_data_t));
	curr_pos += sizeof(ref_inter_robot_data_t);
	//self set number for identification purposes only
	graphic_data.graphic_name[0] = 0;
	graphic_data.graphic_name[1] = 0;
	graphic_data.graphic_name[2] = 1;
	graphic_data.layer = 0;
	//draw number
	if (modify == 1) {
		graphic_data.operation_type = 2; //0 = no operation, 1 = add, 2= modify, 3 = delete
	} else {
		graphic_data.operation_type = 1; //0 = no operation, 1 = add, 2= modify, 3 = delete
	}
	graphic_data.graphic_type = 7; // char
	graphic_data.start_angle = 30; // font size
	graphic_data.end_angle = char_len; //number of decimal places
	graphic_data.width = 7; //line width
	graphic_data.layer = 0;
	//assuming 1920x1080? need check
	graphic_data.start_x = 50;
	graphic_data.start_y = 600;
	memcpy(tx_buffer + curr_pos, &graphic_data, sizeof(graphic_data_struct_t));
	curr_pos += sizeof(graphic_data_struct_t);
	memcpy(tx_buffer + curr_pos, char_buffer, char_len);
	curr_pos += char_len;

	append_CRC16_check_sum(tx_buffer, curr_pos + 2);
	while (huart6.gState != HAL_UART_STATE_READY) {
		vTaskDelay(1);
	}
	HAL_UART_Transmit_DMA(&huart6, tx_buffer, curr_pos + 2);
}

void draw_diagonal(uint8_t modify) {
	draw_diagonal_mode(modify);
	vTaskDelay(150);
	draw_is_diagonal(modify);
}

void draw_diagonal_mode(uint8_t modify) {
	uint32_t curr_pos = 0;
	uint8_t char_len = 0;
	curr_pos = 0;
	if (!diagonal_mode) {
		graphic_data.color = 4; // purplish red
		char_len = snprintf((char*) char_buffer, 30, "DIAGONAL OFF");
	} else {
		graphic_data.color = 3; // orange
		char_len = snprintf((char*) char_buffer, 30, "DIAGONAL ON");
	}
	send_header.start_frame = 0xA5;
	send_header.cmd_id = REF_ROBOT_COMMS_CMD_ID;
	send_header.seq = g_ref_tx_seq++;
	send_header.data_length = sizeof(ref_inter_robot_data_t)
			+ sizeof(graphic_data_struct_t) + char_len;
	send_header.seq = g_ref_tx_seq++;
	memcpy(tx_buffer + curr_pos, &send_header, 7);
	curr_pos += sizeof(ref_frame_header_t);
	append_CRC8_check_sum(tx_buffer, 5);

	//for drawing 1 character graphic
	graphic_header.cmd_ID = 0x110;
	//send to self
	graphic_header.send_ID = ref_robot_data.robot_id;
	graphic_header.receiver_ID = g_client_id;
	memcpy(tx_buffer + curr_pos, &graphic_header,
			sizeof(ref_inter_robot_data_t));
	curr_pos += sizeof(ref_inter_robot_data_t);
	//self set number for identification purposes only
	graphic_data.graphic_name[0] = 0;
	graphic_data.graphic_name[1] = 0;
	graphic_data.graphic_name[2] = 2;
	graphic_data.layer = 0;
	//draw number
	if (modify == 1) {
		graphic_data.operation_type = 2; //0 = no operation, 1 = add, 2= modify, 3 = delete
	} else {
		graphic_data.operation_type = 1; //0 = no operation, 1 = add, 2= modify, 3 = delete
	}
	graphic_data.graphic_type = 7; // char
	graphic_data.start_angle = 30; // font size
	graphic_data.end_angle = char_len; //number of decimal places
	graphic_data.width = 7; //line width
	graphic_data.layer = 0;
	//assuming 1920x1080? need check
	graphic_data.start_x = 50;
	graphic_data.start_y = 700;
	memcpy(tx_buffer + curr_pos, &graphic_data, sizeof(graphic_data_struct_t));
	curr_pos += sizeof(graphic_data_struct_t);
	memcpy(tx_buffer + curr_pos, char_buffer, char_len);
	curr_pos += char_len;

	append_CRC16_check_sum(tx_buffer, curr_pos + 2);
	while (huart6.gState != HAL_UART_STATE_READY) {
		vTaskDelay(1);
	}
	HAL_UART_Transmit_DMA(&huart6, tx_buffer, curr_pos + 2);
}

void draw_is_diagonal(uint8_t modify) {
	uint32_t curr_pos = 0;
	uint8_t char_len = 0;
	curr_pos = 0;
	if (!is_diagonal) {
		graphic_data.color = 5; //pink
		char_len = snprintf((char*) char_buffer, 30, "NOT DIAGONAL");
	} else {
		graphic_data.color = 2; //green
		char_len = snprintf((char*) char_buffer, 30, "DIAGONAL");
	}
	send_header.start_frame = 0xA5;
	send_header.cmd_id = REF_ROBOT_COMMS_CMD_ID;
	send_header.seq = g_ref_tx_seq++;
	send_header.data_length = sizeof(ref_inter_robot_data_t)
			+ sizeof(graphic_data_struct_t) + char_len;
	send_header.seq = g_ref_tx_seq++;
	memcpy(tx_buffer + curr_pos, &send_header, 7);
	curr_pos += sizeof(ref_frame_header_t);
	append_CRC8_check_sum(tx_buffer, 5);

	//for drawing 1 character graphic
	graphic_header.cmd_ID = 0x110;
	//send to self
	graphic_header.send_ID = ref_robot_data.robot_id;
	graphic_header.receiver_ID = g_client_id;
	memcpy(tx_buffer + curr_pos, &graphic_header,
			sizeof(ref_inter_robot_data_t));
	curr_pos += sizeof(ref_inter_robot_data_t);
	//self set number for identification purposes only
	graphic_data.graphic_name[0] = 0;
	graphic_data.graphic_name[1] = 0;
	graphic_data.graphic_name[2] = 3;
	graphic_data.layer = 0;
	//draw number
	if (modify == 1) {
		graphic_data.operation_type = 2; //0 = no operation, 1 = add, 2= modify, 3 = delete
	} else {
		graphic_data.operation_type = 1; //0 = no operation, 1 = add, 2= modify, 3 = delete
	}
	graphic_data.graphic_type = 7; // char
	graphic_data.start_angle = 20; // font size
	graphic_data.end_angle = char_len; //number of decimal places
	graphic_data.width = 7; //line width
	graphic_data.layer = 0;
	//assuming 1920x1080? need check
	graphic_data.start_x = 50;
	graphic_data.start_y = 650;
	memcpy(tx_buffer + curr_pos, &graphic_data, sizeof(graphic_data_struct_t));
	curr_pos += sizeof(graphic_data_struct_t);
	memcpy(tx_buffer + curr_pos, char_buffer, char_len);
	curr_pos += char_len;

	append_CRC16_check_sum(tx_buffer, curr_pos + 2);
	while (huart6.gState != HAL_UART_STATE_READY) {
		vTaskDelay(1);
	}
	HAL_UART_Transmit_DMA(&huart6, tx_buffer, curr_pos + 2);
}

void draw_lines() {
	uint32_t curr_pos = 0;
	curr_pos = 0;
	send_header.start_frame = 0xA5;
	send_header.cmd_id = REF_ROBOT_COMMS_CMD_ID;
	send_header.seq = g_ref_tx_seq++;
	send_header.data_length = sizeof(ref_inter_robot_data_t)
			+ sizeof(graphic_data_struct_t) + sizeof(graphic_data_struct_t);//+ sizeof(graphic_data_struct_t);
	send_header.seq = g_ref_tx_seq++;
	memcpy(tx_buffer + curr_pos, &send_header, 7);
	curr_pos += sizeof(ref_frame_header_t);
	append_CRC8_check_sum(tx_buffer, 5);

	//for drawing 2 graphicS
	graphic_header.cmd_ID = 0x102;
	//send to self
	graphic_header.send_ID = ref_robot_data.robot_id;
	graphic_header.receiver_ID = g_client_id;
	memcpy(tx_buffer + curr_pos, &graphic_header,
			sizeof(ref_inter_robot_data_t));
	curr_pos += sizeof(ref_inter_robot_data_t);
	//self set number for identification purposes only
	//graphic name sets unique id for the graphic
	graphic_data.graphic_name[0] = 0;
	graphic_data.graphic_name[1] = 0;
	graphic_data.graphic_name[2] = 5;
	graphic_data.layer = 0;

	graphic_data.operation_type = 1; // add
	graphic_data.color = 5; // pink
	graphic_data.graphic_type = 0; // straight line
	graphic_data.layer = 0;
	graphic_data.width = 5;
	//assuming 1920x1080? need check
	graphic_data.start_x = 0;
	graphic_data.start_y = 0;
	graphic_data.end_x = 900;
	graphic_data.end_y = 900;
	memcpy(tx_buffer + curr_pos, &graphic_data, sizeof(graphic_data_struct_t));
	curr_pos += sizeof(graphic_data_struct_t);
	graphic_data.graphic_name[0] = 0;
	graphic_data.graphic_name[1] = 0;
	graphic_data.graphic_name[2] = 6;

	graphic_data.layer = 0;
	graphic_data.start_x = 1920;
	graphic_data.start_y = 0;
	graphic_data.end_x = 1020;
	graphic_data.end_y = 900;
	memcpy(tx_buffer + curr_pos, &graphic_data, sizeof(graphic_data_struct_t));
	curr_pos += sizeof(graphic_data_struct_t);

	append_CRC16_check_sum(tx_buffer, curr_pos + 2);
	while (huart6.gState != HAL_UART_STATE_READY) {
		vTaskDelay(1);
	}
	HAL_UART_Transmit_DMA(&huart6, tx_buffer, curr_pos + 2);
}


void clear_hud(){
	uint32_t curr_pos = 0;
	uint8_t char_len = 0;
	curr_pos = 0;
	//char_len = snprintf((char*) char_buffer, 30, "GEAR %d", gear_speed.curr_gear);

	send_header.start_frame = 0xA5;
	send_header.cmd_id = REF_ROBOT_COMMS_CMD_ID;
	send_header.seq = g_ref_tx_seq++;
	send_header.data_length = sizeof(ref_delete_graphic_t);
	send_header.seq = g_ref_tx_seq++;
	memcpy(tx_buffer + curr_pos, &send_header, 7);
	ref_delete_graphic_t ref_delete;
	ref_delete.cmd_ID = 0x100;
	ref_delete.graphic_layer = 9; // Number of graphic layers to delete
	ref_delete.graphic_operation = 2; // Delete all graphic layers
	ref_delete.receiver_ID = g_client_id;
	ref_delete.send_ID = ref_robot_data.robot_id;
	memcpy(tx_buffer + curr_pos, &ref_delete, sizeof(ref_delete_graphic_t));
	append_CRC16_check_sum(tx_buffer, curr_pos + 2);
	while (huart6.gState != HAL_UART_STATE_READY) {
		vTaskDelay(1);
	}
	HAL_UART_Transmit_DMA(&huart6, tx_buffer, curr_pos + 2);

}


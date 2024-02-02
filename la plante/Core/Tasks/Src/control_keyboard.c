/*
 * control_keyboard.c
 *
 *  Created on: 6 Jul 2023
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "control_input_task.h"
#include "control_keyboard.h"
#include "motor_control.h"

#define TRIG_DELAY 50

extern remote_cmd_t remote_cmd;
extern QueueHandle_t buzzing_task_msg;
extern orientation_data_t imu_heading;
extern chassis_control_t chassis_ctrl_data;
extern gun_control_t launcher_ctrl_data;
extern gimbal_control_t gimbal_ctrl_data;
extern uint8_t safety_toggle;
extern uint8_t launcher_safety_toggle;

extern int g_spinspin_mode;
extern float spin_movement_limit;

extern int diagonal_mode;
uint8_t diagonal_toggled = 0;
uint8_t is_diagonal = 0;
uint8_t diagonal_direction = 0;

extern uint8_t damage_taken;
uint8_t damage_taken_last = 0;
uint32_t q_trig_time = 0;
uint32_t e_trig_time = 0;

static float speed_limit = 1.0;

#define LOW_SPEED 0.5

void keyboard_control_input() {
	mouse_gimbal_input();
	keyboard_chassis_input();
	mouse_launcher_control_input();
}

void keyboard_chassis_input() {
	if (safety_toggle || remote_cmd.right_switch != ge_RSW_ALL_ON) {
		chassis_ctrl_data.enabled = 0;
		chassis_ctrl_data.horizontal = 0;
		chassis_ctrl_data.forward = 0;
		chassis_ctrl_data.yaw = 0;
	} else {
		if (remote_cmd.right_switch == ge_RSW_ALL_ON) {
			chassis_ctrl_data.enabled = 1;
			float horizontal_input = 0.0;
			float forward_input = 0.0;
			float yaw_input = 0.0;

			// Press and hold shift to decrease speed
			if (remote_cmd.keyboard_keys & KEY_OFFSET_SHIFT) {
				speed_limit = LOW_SPEED;
			} else {
				speed_limit = 1;
			}

			// Overriding the overheat protection
			if (remote_cmd.keyboard_keys & KEY_OFFSET_CTRL) {
				launcher_ctrl_data.override = 1;
			} else {
				launcher_ctrl_data.override = 0;
			}

			if (remote_cmd.keyboard_keys & KEY_OFFSET_W) {
				forward_input += KEYBD_MAX_SPD;
			}
			if (remote_cmd.keyboard_keys & KEY_OFFSET_S) {
				forward_input -= KEYBD_MAX_SPD;
			}
			if (remote_cmd.keyboard_keys & KEY_OFFSET_A) {
				horizontal_input -= KEYBD_MAX_SPD;
			}
			if (remote_cmd.keyboard_keys & KEY_OFFSET_D) {
				horizontal_input += KEYBD_MAX_SPD;
			}

			uint32_t curr_time = HAL_GetTick();
			static uint32_t q_time = 0;
			static uint32_t e_time = 0;

			if (remote_cmd.keyboard_keys & KEY_OFFSET_Q){
				if (curr_time - q_time > TRIG_DELAY){
					g_spinspin_mode = 1 - g_spinspin_mode;
				}
				q_time = HAL_GetTick();
			}

			if (remote_cmd.keyboard_keys & KEY_OFFSET_E){
				if (curr_time - e_time > TRIG_DELAY){
					diagonal_mode = 1 - diagonal_mode;
				}
				e_time = HAL_GetTick();
			}

			if (diagonal_mode && damage_taken) {
				is_diagonal = 1;
				if (damage_taken != damage_taken_last) {
					//diagonal_direction = HAL_GetTick() % 2;
					diagonal_direction = rand() % 2;
				}
			} else {
				is_diagonal = 0;
			}
			damage_taken_last = damage_taken;

			if (is_diagonal) {
				// turn the chassis 45 degrees either left or right
				if (diagonal_direction) {
					yaw_input = chassis_center_yaw(45.0 * PI / 180.0);
				} else {
					yaw_input = chassis_center_yaw(-45.0 * PI / 180.0);
				}
			}

			// beyblade mode will overwrite diagonal mode
			if (g_spinspin_mode) {
				yaw_input = SPIN_SPEED;
				spin_movement_limit = SPIN_MOVEMENT;
				is_diagonal = 0;
			}

			if (!g_spinspin_mode && !is_diagonal) {
				//center yaw motor such that yaw motor = 0
				yaw_input = chassis_center_yaw(0);
				spin_movement_limit = 1;
			}

			chassis_ctrl_data.horizontal = horizontal_input * spin_movement_limit * speed_limit;
			chassis_ctrl_data.forward = forward_input * spin_movement_limit * speed_limit;
			chassis_ctrl_data.yaw = yaw_input;
		}
	}
}

void mouse_gimbal_input() {
	if (safety_toggle || remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
		gimbal_ctrl_data.enabled = 0;
	} else {
		gimbal_ctrl_data.enabled = 1;
		float pitch_mouse = (float) remote_cmd.mouse_y * MOUSE_Y_INVERT
				* PITCH_INVERT * MOUSE_Y_SENSITIVITY / 32768;
		float yaw_mouse = (float) remote_cmd.mouse_x * MOUSE_X_INVERT
				* YAW_INVERT * MOUSE_X_SENSITIVITY / 32768;

		gimbal_turn_ang(pitch_mouse, yaw_mouse);
	}
}

void mouse_launcher_control_input() {
	if (safety_toggle || launcher_safety_toggle
			|| remote_cmd.right_switch == ge_RSW_SHUTDOWN
			|| remote_cmd.left_switch != ge_LSW_UNSAFE) {
		if (remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
			launcher_ctrl_data.enabled = 0;
		}
		launcher_ctrl_data.gun_feeding_speed = 0;
		launcher_ctrl_data.projectile_speed = 0;
		if (remote_cmd.left_switch != ge_LSW_UNSAFE) {
			launcher_safety_toggle = 0;
		}
	} else if (remote_cmd.left_switch == ge_LSW_UNSAFE) {
		launcher_ctrl_data.enabled = 1;
		if (remote_cmd.mouse_left) {
			launcher_ctrl_data.projectile_speed = 1;
			launcher_ctrl_data.gun_feeding_speed =1;

		} else {
			launcher_ctrl_data.projectile_speed = 0.5;
			launcher_ctrl_data.gun_feeding_speed = 0;
		}
	} else {
		launcher_ctrl_data.gun_feeding_speed = 0;
		launcher_ctrl_data.projectile_speed = 0;

	}
}

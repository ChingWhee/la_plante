/*
 * control_remote.c
 *
 *  Created on: 6 Jul 2023
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "control_input_task.h"
#include "control_remote.h"
#include "motor_control.h"


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
extern uint8_t damage_taken;

void remote_control_input() {
	remote_gimbal_input();
	remote_chassis_input();
	remote_launcher_control_input();
}

void remote_chassis_input() {
	if (safety_toggle || remote_cmd.right_switch != ge_RSW_ALL_ON) {
//		chassis_ctrl_data.enabled = 0;
		chassis_kill_ctrl();
	} else {
			chassis_ctrl_data.enabled = 1;
			float horizontal_input = 0.0;
			float forward_input = 0.0;
			float yaw_input = 0.0;

			forward_input = (float) remote_cmd.left_y / RC_LIMITS;
			horizontal_input = (float) remote_cmd.left_x / RC_LIMITS;
			if (damage_taken) {
				// turn the chassis 45 degrees
				yaw_input = chassis_center_yaw(45.0 * PI / 180.0);
			}

			if (g_spinspin_mode) {
				yaw_input = SPIN_SPEED;
				spin_movement_limit = SPIN_MOVEMENT;
			} else if (!g_spinspin_mode && !damage_taken) {
				//center yaw motor such that yaw motor = 0
				yaw_input = chassis_center_yaw(0);
				spin_movement_limit = 1;
			}

//yaw_input = (float) remote_cmd.right_x * CHASSIS_YAW_MAX_RPM /RC_LIMITS;
			chassis_set_ctrl(forward_input * spin_movement_limit,
					horizontal_input * spin_movement_limit, yaw_input);
	}
}


void remote_gimbal_input() {
	if (safety_toggle || remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
		gimbal_ctrl_data.enabled = 0;
	} else {
		gimbal_ctrl_data.enabled = 1;
		float pitch_remote = ((float) remote_cmd.right_y / 660) * PITCH_INVERT
				* REMOTE_PITCH_SPEED;
		float yaw_remote = ((float) remote_cmd.right_x / 660) * YAW_INVERT
				* REMOTE_YAW_SPEED;
		gimbal_turn_ang(pitch_remote, yaw_remote);
	}
}


void remote_launcher_control_input() {
	if (safety_toggle || remote_cmd.right_switch == ge_RSW_SHUTDOWN
			|| remote_cmd.left_switch != ge_LSW_UNSAFE) {
		if (remote_cmd.left_switch != ge_LSW_UNSAFE) {
			launcher_safety_toggle = 0;
		}
		if (remote_cmd.right_switch == ge_RSW_SHUTDOWN){
							launcher_ctrl_data.enabled = 0;
				}
//		launcher_ctrl_data.enabled = 0;
		launcher_ctrl_data.gun_feeding_speed = 0;
		launcher_ctrl_data.projectile_speed = 0;
	} else {
		launcher_ctrl_data.enabled = 1;
		launcher_ctrl_data.projectile_speed = 1;
		if (remote_cmd.left_switch == ge_LSW_UNSAFE) {
			launcher_ctrl_data.gun_feeding_speed = 1;
		} else {
			launcher_ctrl_data.gun_feeding_speed = 0;
		}
	}

}


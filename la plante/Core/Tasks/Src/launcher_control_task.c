/*
 * launcher_control_task.c
 *
 *  Created on: Jul 26, 2021
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "motor_control.h"
#include "launcher_control_task.h"

extern EventGroupHandle_t launcher_event_group;

extern ref_game_state_t ref_game_state;
extern motor_data_t can_motors[24];
extern gun_control_t launcher_ctrl_data;

extern remote_cmd_t remote_cmd;
extern referee_limit_t referee_limiters;

static uint8_t reached_rpm = 0;

static uint32_t start_time = 0;
static uint32_t clear_time = 0;
static uint8_t unjamming = 0;

static uint8_t overheat = 0;
static int16_t ammo_left = 0;

//static float friction_offset = FRICTION_OFFSET;

extern QueueHandle_t telem_motor_queue;

#define SPEED_THRESHOLD 100
#define BULLET_17_HEAT 10
#define BULLET_42_HEAT 100
#define HEAT_LIMIT 280
#define AMMO_LIMIT 6
#define AMMO_RELEASE 7

#ifdef BULLET_17
#define BULLET_ACTUAL_HEAT BULLET_17_HEAT
#endif

#ifdef BULLET_42
#define BULLET_ACTUAL_HEAT BULLET_42_HEAT
#endif

extern ref_game_robot_data_t ref_robot_data;
extern ref_robot_power_data_t ref_power_data;

extern uint32_t ref_power_data_txno;
extern ref_magazine_data_t ref_mag_data;
extern uint32_t ref_mag_data_txno;
static uint32_t prev_power_data_no = 0;
static uint32_t prev_mag_data_no = 0;

void launcher_control_task(void *argument) {
	TickType_t start_time;
	while (1) {
		//event flags!
		xEventGroupWaitBits(launcher_event_group, 0b111, pdTRUE, pdTRUE,
		portMAX_DELAY);
		status_led(4, on_led);
		start_time = xTaskGetTickCount();

		if (launcher_ctrl_data.enabled) {
			launcher_control(can_motors + LFRICTION_MOTOR_ID - 1,
					can_motors + RFRICTION_MOTOR_ID - 1,
					can_motors + FEEDER_MOTOR_ID - 1);

		} else {
			can_motors[LFRICTION_MOTOR_ID - 1].rpm_pid.output = 0;
			can_motors[RFRICTION_MOTOR_ID - 1].rpm_pid.output = 0;
			can_motors[FEEDER_MOTOR_ID - 1].rpm_pid.output = 0;
//			motor_send_can(can_motors, FEEDER_MOTOR_ID, LFRICTION_MOTOR_ID,
//			RFRICTION_MOTOR_ID, 0);
		}
		status_led(4, off_led);
		//vTaskDelay(CHASSIS_DELAY);
		xEventGroupClearBits(launcher_event_group, 0b111);
		vTaskDelayUntil(&start_time, CHASSIS_DELAY);
	}

}

void launcher_control(motor_data_t *left_friction_motor,
		motor_data_t *right_friction_motor, motor_data_t *feeder) {
	int16_t feeder_output = 0;
	static uint32_t overheat_time;
	static float target_ang;
	uint32_t curr_time = HAL_GetTick();
	static uint32_t overheat_start;
	static uint32_t last_fire;
	static uint8_t fired;
	int16_t firing_speed = launcher_ctrl_data.gun_feeding_speed
			* referee_limiters.feeding_speed / FEEDER_SPEED_RATIO;
	// gun feeding speed -> 1
	// feeding_speed -> LV1_FEEDER

	if (launcher_ctrl_data.gun_feeding_speed == 0) {
		reached_rpm = 0;
		feeder->rpm_pid.output = 0;
		speed_pid(0, feeder->raw_data.rpm, &feeder->rpm_pid);
		left_friction_motor->rpm_pid.output = 0;
		right_friction_motor->rpm_pid.output = 0;
	} else {
		// x = 1 - x
		// Toggle x between 0 and 1
		if (feeder->raw_data.torque > FEEDER_JAM_TORQUE) {
			unjamming = 1;
			start_time = curr_time;
		}
		if ((start_time + FEEDER_UNJAM_TIME < curr_time) && unjamming == 1) {
			unjamming = 0;
		}

		// Ammo left before overheating
		ammo_left = (HEAT_LIMIT - ref_power_data.shooter_heat0) / BULLET_17_HEAT;
		if (ammo_left < AMMO_LIMIT) {
			overheat = 1;
		} else if (ammo_left > AMMO_RELEASE){
			overheat = 0;
			launcher_ctrl_data.override = 0;
		}

		// Reverse feeder direction if it is jammed
		if (unjamming) {
			firing_speed = launcher_ctrl_data.gun_feeding_speed
				* FEEDER_UNJAM_SPD / FEEDER_SPEED_RATIO;
		}

		int16_t launcher_rpm = -referee_limiters.projectile_speed
				* FRICTION_INVERT * PROJECTILE_SPEED_RATIO;

		speed_pid(launcher_rpm, left_friction_motor->raw_data.rpm,
				&left_friction_motor->rpm_pid);
		speed_pid(-launcher_rpm, right_friction_motor->raw_data.rpm,
				&right_friction_motor->rpm_pid);

		// Make sure that both flywheels reach the target speed before starting feeder
		if (abs(launcher_rpm - left_friction_motor->raw_data.rpm) < SPEED_THRESHOLD &&
				abs(-launcher_rpm - right_friction_motor->raw_data.rpm) < SPEED_THRESHOLD) {
			reached_rpm = 1;
		}

		if (reached_rpm) {
			if (overheat && !unjamming && !launcher_ctrl_data.override) {
				feeder->rpm_pid.output = 0;
			} else {
				speed_pid(firing_speed * feeder->angle_data.gearbox_ratio,
						feeder->raw_data.rpm, &feeder->rpm_pid);
			}
		}
	}

	motor_send_can(can_motors, FEEDER_MOTOR_ID, LFRICTION_MOTOR_ID,
	RFRICTION_MOTOR_ID, 0);
}

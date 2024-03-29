/*
 * movement_control_task.c
 *
 *  Created on: Jan 19, 2021
 *      Author: Hans Kurnia
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "motor_control.h"
#include "arm_math.h"
#include "movement_control_task.h"

extern EventGroupHandle_t chassis_event_group;

extern chassis_control_t chassis_ctrl_data;
extern motor_data_t can_motors[24];
extern referee_limit_t referee_limiters;
extern speed_shift_t gear_speed;
float g_chassis_yaw = 0;

float motor_yaw_mult[4];
int8_t motor_x_mult[4];
int8_t motor_y_mult[4];

extern QueueHandle_t telem_motor_queue;
extern int g_spinspin_mode;

void movement_control_task(void *argument) {
	TickType_t start_time;
	//initialise in an array so it's possible to for-loop it later
	motor_yaw_mult[0] = FR_YAW_MULT;
	motor_yaw_mult[1] = FL_YAW_MULT;
	motor_yaw_mult[2] = BL_YAW_MULT;
	motor_yaw_mult[3] = BR_YAW_MULT;

	motor_x_mult[0] = FR_VX_MULT;
	motor_x_mult[1] = FL_VX_MULT;
	motor_x_mult[2] = BL_VX_MULT;
	motor_x_mult[3] = BR_VX_MULT;

	motor_y_mult[0] = FR_VY_MULT;
	motor_y_mult[1] = FL_VY_MULT;
	motor_y_mult[2] = BL_VY_MULT;
	motor_y_mult[3] = BR_VY_MULT;
	while (1) {

#ifndef CHASSIS_MCU

		EventBits_t motor_bits;
		//wait for all motors to have updated data before PID is allowed to run
		motor_bits = xEventGroupWaitBits(chassis_event_group, 0b1111, pdTRUE,
		pdTRUE,
		portMAX_DELAY);
		if (motor_bits == 0b1111) {
			status_led(3, on_led);
			start_time = xTaskGetTickCount();
			if (chassis_ctrl_data.enabled) {
				chassis_motion_control(can_motors + FR_MOTOR_ID - 1,
						can_motors + FL_MOTOR_ID - 1,
						can_motors + BL_MOTOR_ID - 1,
						can_motors + BR_MOTOR_ID - 1,
						can_motors + FR_ROTATE_MOTOR_ID - 1,
						can_motors + FL_ROTATE_MOTOR_ID - 1,
						can_motors + BL_ROTATE_MOTOR_ID - 1,
						can_motors + BR_ROTATE_MOTOR_ID - 1);
			} else {
				can_motors[FR_MOTOR_ID - 1].rpm_pid.output = 0;
				can_motors[FL_MOTOR_ID - 1].rpm_pid.output = 0;
				can_motors[BL_MOTOR_ID - 1].rpm_pid.output = 0;
				can_motors[BR_MOTOR_ID - 1].rpm_pid.output = 0;
				g_chassis_yaw = 0;

				//change CAN messages to a seperate task? so it doesn't fill up CAN transmitter
//				motor_send_can(can_motors, FR_MOTOR_ID, FL_MOTOR_ID,
//				BL_MOTOR_ID,
//				BR_MOTOR_ID);
			}
#else
		chassis_MCU_send_CAN();
#endif
			status_led(3, off_led);
		} else {
			//motor timed out
			can_motors[FR_MOTOR_ID - 1].rpm_pid.output = 0;
			can_motors[FL_MOTOR_ID - 1].rpm_pid.output = 0;
			can_motors[BL_MOTOR_ID - 1].rpm_pid.output = 0;
			can_motors[BR_MOTOR_ID - 1].rpm_pid.output = 0;
//			motor_send_can(can_motors, FR_MOTOR_ID, FL_MOTOR_ID, BL_MOTOR_ID,
//			BR_MOTOR_ID);
		}
		//clear bits if it's not already cleared
		xEventGroupClearBits(chassis_event_group, 0b1111);
		//delays task for other tasks to run
		vTaskDelayUntil(&start_time, CHASSIS_DELAY);
	}
	osThreadTerminate(NULL);
}

void chassis_motion_control(motor_data_t *motorfr, motor_data_t *motorfl,
		motor_data_t *motorbl, motor_data_t *motorbr,
		motor_data_t *rotatefr, motor_data_t *rotatefl,
		motor_data_t *rotatebl, motor_data_t *rotatebr) {
	static uint32_t prev_time;
	//get the angle between the gun and the chassis
	//so that movement is relative to gun, not chassis
	float rel_angle = -can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang;
	float translation_rpm[4] = { 0, };
	float translation_rpm_x[4] = { 0 };
	float translation_rpm_y[4] = { 0 };
	float rotation_angle[4] = { 0 };
	float yaw_rpm[4] = { 0, };
	float total_power = 0;

	int32_t chassis_rpm = LV1_MAX_SPEED;
	int32_t chassis_current = LV1_MAX_CURRENT;
	if (referee_limiters.robot_level == 1) {
		if (g_spinspin_mode) {
			chassis_rpm = LV1_SPIN_MAX_SPEED;
		} else {
			chassis_rpm = LV1_MAX_SPEED;
		}
		chassis_current = LV1_MAX_CURRENT;
	} else if (referee_limiters.robot_level == 2) {
		chassis_rpm = LV2_MAX_SPEED;
		chassis_current = LV2_MAX_CURRENT;
	} else if (referee_limiters.robot_level == 3) {
		chassis_rpm = LV3_MAX_SPEED;
		chassis_current = LV3_MAX_CURRENT;
	}
	chassis_rpm = (chassis_rpm > M3508_MAX_RPM) ? M3508_MAX_RPM : chassis_rpm;
	//rotate angle of the movement :)
	//MA1513/MA1508E is useful!!

	float rel_forward = (chassis_ctrl_data.horizontal * sin(-rel_angle)
			+ chassis_ctrl_data.forward * cos(-rel_angle));
	float rel_horizontal = ((chassis_ctrl_data.forward * sin(-rel_angle))
			+ (-chassis_ctrl_data.horizontal * cos(-rel_angle)));

	float rel_yaw = chassis_ctrl_data.yaw;

	// distance of the robot center to the wheel pivot (in mm)
	float motor_x_dist = MOTOR_X_DIST;
	float motor_y_dist = MOTOR_Y_DIST;
	// scale the x and y distance to < 1
	float dist_scale = (motor_x_dist > motor_y_dist) ? motor_x_dist : motor_y_dist;
	motor_x_dist /= dist_scale;
	motor_y_dist /= dist_scale;

	for (uint8_t a = 0; a < 4; a++) {
		translation_rpm_x[a] = rel_horizontal + motor_x_mult[a] * motor_x_dist * rel_yaw;
		translation_rpm_y[a] = rel_forward + motor_y_mult[a] * motor_y_dist * rel_yaw;
		translation_rpm[a] = sqrt(pow(translation_rpm_x[a], 2) + pow(translation_rpm_y[a], 2));
		rotation_angle[a] = atan(translation_rpm_y[a] / translation_rpm_x[a]);

		if (translation_rpm_y[a] < 0) {
			translation_rpm[a] *= -1;
		}
	}

    optimise_angle(rotatefr, &rotation_angle[0], &translation_rpm[0]);
    optimise_angle(rotatefl, &rotation_angle[1], &translation_rpm[1]);
    optimise_angle(rotatebl, &rotation_angle[2], &translation_rpm[2]);
    optimise_angle(rotatebr, &rotation_angle[3], &translation_rpm[3]);

	//if forward + horizontal + yaw > 1 for any wheel
	//scale all the RPM for all the wheels equally so that one wheel does not exceed max RPM
	float rpm_mult = 1;
	for (uint8_t i = 0; i < 4; i++) {
		if (fabs(translation_rpm[i]) > rpm_mult) {
			rpm_mult = fabs(translation_rpm[i]);
		}
	}

	for (uint8_t j = 0; j < 4; j++) {
		translation_rpm[j] = translation_rpm[j] * chassis_rpm / rpm_mult;
	}

	motorfr->rpm_pid.max_out = chassis_current;
	motorfl->rpm_pid.max_out = chassis_current;
	motorbl->rpm_pid.max_out = chassis_current;
	motorbr->rpm_pid.max_out = chassis_current;

	//calculate the outputs for each motor
	swerve_turn(motorfr, rotatefr, rotation_angle[0], translation_rpm[0]);
	total_power += fabs(motorfr->rpm_pid.output);
	total_power += fabs(rotatefr->rpm_pid.output);
	swerve_turn(motorfl, rotatefl, rotation_angle[1], translation_rpm[1]);
	total_power += fabs(motorfl->rpm_pid.output);
	total_power += fabs(rotatefl->rpm_pid.output);
	swerve_turn(motorbl, rotatebl, rotation_angle[2], translation_rpm[2]);
	total_power += fabs(motorbl->rpm_pid.output);
	total_power += fabs(motorfr->rpm_pid.output);
	swerve_turn(motorbr, rotatebr, rotation_angle[3], translation_rpm[3]);
	total_power += fabs(motorbr->rpm_pid.output);
	total_power += fabs(motorfr->rpm_pid.output);

//	speed_pid(translation_rpm[0], motorfr->raw_data.rpm, &motorfr->rpm_pid);
//	total_power += fabs(motorfr->rpm_pid.output);
//	speed_pid(translation_rpm[1], motorfl->raw_data.rpm, &motorfl->rpm_pid);
//	total_power += fabs(motorfl->rpm_pid.output);
//	speed_pid(translation_rpm[2], motorbl->raw_data.rpm, &motorbl->rpm_pid);
//	total_power += fabs(motorbl->rpm_pid.output);
//	speed_pid(translation_rpm[3], motorbr->raw_data.rpm, &motorbr->rpm_pid);
//	total_power += fabs(motorbr->rpm_pid.output);

	motor_send_can(can_motors, FR_ROTATE_MOTOR_ID, FL_ROTATE_MOTOR_ID,
			BL_ROTATE_MOTOR_ID, BR_ROTATE_MOTOR_ID);
	motor_send_can(can_motors, FR_MOTOR_ID, FL_MOTOR_ID, BL_MOTOR_ID,
			BR_MOTOR_ID);
}

void optimise_angle(motor_data_t *rotate_motor, float *rotation_angle, float *translation_rpm) {
	if (fabs(*rotation_angle - rotate_motor->angle_data.adj_ang) > (PI / 2)) {
		*rotation_angle += PI;
		*translation_rpm *= -1;
	}
}



void swerve_turn(motor_data_t *movement_motor, motor_data_t *rotate_motor, float rotation_angle, float translation_rpm) {
	speed_pid(translation_rpm, movement_motor->raw_data.rpm, &movement_motor->rpm_pid);
	angle_pid(rotation_angle, rotate_motor->angle_data.adj_ang, rotate_motor);
}

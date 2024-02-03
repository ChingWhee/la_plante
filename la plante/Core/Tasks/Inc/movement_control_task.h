/*
 * movement_control_task.h
 *
 *  Created on: 19 Jan 2021
 *      Author: Hans Kurnia
 */

#ifndef TASKS_INC_MOVEMENT_CONTROL_TASK_H_
#define TASKS_INC_MOVEMENT_CONTROL_TASK_H_


void movement_control_task(void *argument);
void chassis_motion_control();
void optimise_angle(motor_data_t *rotate_motor, float *rotation_angle, float *translation_rpm);
void swerve_turn(motor_data_t *movement_motor, motor_data_t *rotate_motor, float rotation_angle, float translation_rpm);
void chassis_pid_init();

#endif /* TASKS_INC_MOVEMENT_CONTROL_TASK_H_ */

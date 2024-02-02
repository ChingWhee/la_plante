/*
 * hud_task.h
 *
 *  Created on: Jul 7, 2023
 *      Author: wx
 */

#ifndef TASKS_INC_HUD_TASK_H_
#define TASKS_INC_HUD_TASK_H_


void hud_task(void *argument);
void draw_spinspin(uint8_t modify);
void draw_diagonal(uint8_t modify);
void draw_diagonal_mode(uint8_t modify);
void draw_is_diagonal(uint8_t modify);

void draw_lines();
void clear_hud();


#endif /* TASKS_INC_HUD_TASK_H_ */

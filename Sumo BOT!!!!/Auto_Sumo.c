/*
 * Auto_Sumo.c
 *
 *  Created on: Aug 9, 2022
 *      Author: tylermccue
 */

#include "Auto_Sumo.h"

tsk_fn_t task_list[] = {&move_across,
						&turn_at_edge};



void initialize(auto_sumo_t* bot){

	bot->state = 0;
	bot->next_time = HAL_GetTick() + 1500;
}

void automatic(auto_sumo_t* bot){

	task_list[bot->state](bot);

}

void move_across(auto_sumo_t* bot){

	set_level(&bot->lb_motor, 4999);
	set_level(&bot->lf_motor, 4999);
	set_level(&bot->rb_motor, 4999);
	set_level(&bot->rf_motor, 4999);
	if(HAL_GetTick() > bot->next_time){

		bot->state = 1;
		bot->next_time = HAL_GetTick() + 500;

	}
}

void turn_at_edge(auto_sumo_t* bot){

	set_level(&bot->lb_motor, -4999);
	set_level(&bot->lf_motor, -4999);
	set_level(&bot->rb_motor, 4999);
	set_level(&bot->rf_motor, 4999);
	if(HAL_GetTick() > bot->next_time){

		bot->state = 0;
		bot->next_time = HAL_GetTick() + 1500;

	}
}

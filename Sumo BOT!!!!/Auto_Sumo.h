/*
 * Auto_Sumo.h
 *
 *  Created on: Aug 9, 2022
 *      Author: tylermccue
 */

#ifndef SRC_AUTO_SUMO_H_
#define SRC_AUTO_SUMO_H_

#include "main.h"
#include "Motor_Driverz.h"

typedef struct auto_sumo{

	Motor_t lb_motor;
	Motor_t lf_motor;
	Motor_t rb_motor;
	Motor_t rf_motor;
	int32_t state;
	int32_t next_time;


} auto_sumo_t;

typedef void (*tsk_fn_t)(auto_sumo_t*);


void initialize(auto_sumo_t*);
void automatic(auto_sumo_t*);
void move_across(auto_sumo_t*);
void turn_at_edge(auto_sumo_t*);

#endif /* SRC_AUTO_SUMO_H_ */

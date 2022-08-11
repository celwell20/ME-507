/*
 * Sumo_Data.h
 *
 *  Created on: Aug 10, 2022
 *      Author: tylermccue
 */

#ifndef SRC_SUMO_DATA_H_
#define SRC_SUMO_DATA_H_

#include "main.h"
#include "Motor_Driverz.h"
#include "Magneto.h"

typedef struct sumo_data{

	Magnet_t mag;
	Motor_t mot;
	uint8_t state;
	ADC_ChannelConfTypeDef* sConfig;
	ADC_HandleTypeDef* adc;
	int32_t curr1;
	int32_t curr2;
	int32_t volt;
	int32_t ref1;
	int32_t ref2;
	int32_t curr1_chan;
	int32_t curr2_chan;
	int32_t volt_chan;
	int32_t ref1_chan;
	int32_t ref2_chan;

} sumo_data_t;

typedef void (*tsk_dfn_t)(sumo_data_t*);

void init_data(sumo_data_t*);
void run_data(sumo_data_t*);
void get_data(sumo_data_t*);
void check_data(sumo_data_t*);


#endif /* SRC_SUMO_DATA_H_ */

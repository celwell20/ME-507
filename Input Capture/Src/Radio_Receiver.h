/*
 * Radio_Receiver.h
 *
 *  Created on: Jul 24, 2022
 *      Author: Tyler McCue
 */

#ifndef SRC_RADIO_RECEIVER_H_
#define SRC_RADIO_RECEIVER_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

struct Radio{

	TIM_HandleTypeDef* htim;
	uint16_t tim_channel;

	uint32_t IC_Val1;
	uint32_t IC_Val2;
	uint32_t Difference;
	int Is_First_Captured;
	uint32_t usWidth;
	int radio_upper;
	int radio_lower;

} typedef Radio_t;

void Read_Start(Radio_t* rad);
void Initialize_Vals(Radio_t* rad);

#endif /* SRC_RADIO_RECEIVER_H_ */

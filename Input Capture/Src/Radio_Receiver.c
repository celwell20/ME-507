/*
 * Radio_Receiver.c
 *
 *  Created on: Jul 24, 2022
 *      Author: tylermccue
 */

#include "Radio_Receiver.h"

void Initialize_Vals(Radio_t* rad){

	rad->Difference = 0;
	rad->IC_Val1 = 0;
	rad->IC_Val2 = 0;
	rad->Is_First_Captured = 0;
	rad->frequency = 0;

}

void Read_Start(Radio_t* rad) {

	HAL_TIM_IC_Start_IT(rad->htim, rad->tim_channel);

}


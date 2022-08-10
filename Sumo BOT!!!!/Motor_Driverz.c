/*
 * Motor_Driverz.c
 *
 *  Created on: Aug 4, 2022
 *      Author: tylermccue
 */


#include "Motor_Driverz.h"


void enable(Motor_t* mot){
	HAL_GPIO_WritePin(mot->nSleepGPIO, mot->nSleep, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(mot->htim, mot->IN_pin);
}

void disable(Motor_t* mot){
	HAL_GPIO_WritePin(mot->nSleepGPIO, mot->nSleep, GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop(mot->htim, mot->IN_pin);
}


void set_level(Motor_t* mot, int32_t level){
	if(level > 0){
		HAL_GPIO_WritePin(mot->EN_GPIO, mot->EN_pin, mot->fwd);
		__HAL_TIM_SET_COMPARE(mot->htim, mot->IN_pin, level);
	}
	else if(level == 0){
		__HAL_TIM_SET_COMPARE(mot->htim, mot->IN_pin, 0);
	}
	else{
		HAL_GPIO_WritePin(mot->EN_GPIO, mot->EN_pin, mot->rev);
		__HAL_TIM_SET_COMPARE(mot->htim, mot->IN_pin, -1*level);
	}

}



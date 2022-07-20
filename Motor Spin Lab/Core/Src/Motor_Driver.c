/*
 * Motor_Driver.c
 *
 *  Created on: Jul 13, 2022
 *      Author: tjmccue
 */

#include "motor_driver.h"


void enable(Motor_t* mot){

	HAL_GPIO_WritePin(mot->EN_GPIO, mot->EN_pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(mot->htim, mot->IN1_pin);
	HAL_TIM_PWM_Start(mot->htim, mot->IN2_pin);

}

void disable(Motor_t* mot){

	HAL_GPIO_WritePin(mot->EN_GPIO, mot->EN_pin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop(mot->htim, mot->IN1_pin);
	HAL_TIM_PWM_Stop(mot->htim, mot->IN2_pin);

}


void set_level(Motor_t* mot, int32_t level){

	if(level > 0){

		__HAL_TIM_SET_COMPARE(mot->htim, mot->IN1_pin, level);
		__HAL_TIM_SET_COMPARE(mot->htim, mot->IN2_pin, 0);

	}
	else{
		__HAL_TIM_SET_COMPARE(mot->htim, mot->IN1_pin, 0);
		__HAL_TIM_SET_COMPARE(mot->htim, mot->IN2_pin, -level);

	}

}

/*
 * Motor_Driverz.h
 *
 *  Created on: Aug 4, 2022
 *      Author: tylermccue
 */

#ifndef SRC_MOTOR_DRIVERZ_H_
#define SRC_MOTOR_DRIVERZ_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

struct Motor{

	TIM_HandleTypeDef* htim;
	uint16_t IN_pin;
	GPIO_TypeDef* EN_GPIO;
	uint16_t EN_pin;
	GPIO_TypeDef* nSleepGPIO;
	uint16_t nSleep;
	uint16_t fwd;
	uint16_t rev;

} typedef Motor_t;

void enable(Motor_t* mot);
void disable(Motor_t* mot);
void set_level(Motor_t* mot, int32_t level);

#endif /* SRC_MOTOR_DRIVERZ_H_ */

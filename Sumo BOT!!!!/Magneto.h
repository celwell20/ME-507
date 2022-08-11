/*
 * Magneto.h
 *
 *  Created on: Aug 10, 2022
 *      Author: tylermccue
 */

#ifndef SRC_MAGNETO_H_
#define SRC_MAGNETO_H_

#include "stm32f4xx_hal.h"
#include "main.h"

struct Magnet{

	I2C_HandleTypeDef* hi2c;
	uint8_t dev_add;
	uint8_t Data_Start;
	uint8_t mode;
	uint8_t set;
	uint8_t check;
	uint8_t buffy[6];
	uint8_t calb[2];
	float XY_angle;

} typedef Magnet_t;

void intialize_mode(Magnet_t* mag);
void read_angle(Magnet_t* mag);



#endif /* SRC_MAGNETO_H_ */

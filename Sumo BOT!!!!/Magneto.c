/*
 * Magneto.c
 *
 *  Created on: Aug 10, 2022
 *      Author: tylermccue
 */


#include "Magneto.h"


void initialize_mode(Magnet_t* mag){


	HAL_I2C_Mem_Write(mag->hi2c, mag->dev_add, mag->set, 1, mag->calb[0], 1, 100);
	HAL_I2C_Mem_Write(mag->hi2c, mag->dev_add, mag->mode, 1, mag->calb[1], 1, 100);

}

void read_angle(Magnet_t* mag){

	HAL_I2C_Mem_Read(mag->hi2c, mag->dev_add, mag->check, 1, mag->buffy, 1, 100);
	if(mag->buffy[0]&0x01){

		HAL_I2C_Mem_Read(mag->hi2c, mag->dev_add, mag->Data_Start, 1, mag->buffy, 6, 100);
		uint16_t x_head = ((mag->buffy[1]<<8) | mag->buffy[0]);
	    uint16_t y_head = ((mag->buffy[3]<<8) | mag->buffy[2]);
	    uint16_t z_head = ((mag->buffy[5]<<8) | mag->buffy[4]);

	    float angle = (atan2f(x_head,y_head)*180)/3.1415;

		if (angle<0){

		  mag->XY_angle = angle + 360;

		}

	}
}


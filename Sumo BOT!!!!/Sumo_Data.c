/*
 * Sumo_Data.c
 *
 *  Created on: Aug 10, 2022
 *      Author: tylermccue
 */

#include "Sumo_Data.h"
#include "Magneto.h"
#include "Motor_Driverz.h"

tsk_dfn_t task_list_dat[] = {&get_data,
						&check_data};

void init_data(sumo_data_t* dat){

	dat->state = 0;
	dat->volt = 3010;

}

void run_data(sumo_data_t* dat){

	task_list_dat[dat->state](dat);

}

void get_data(sumo_data_t* dat){

	read_angle(&dat->mag);

	dat->sConfig->Channel = dat->volt_chan;
	dat->sConfig->Rank = 1;
	dat->sConfig->SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(dat->adc, dat->sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_ADC_Start(dat->adc);
	HAL_ADC_PollForConversion(dat->adc, HAL_MAX_DELAY);
	dat->volt = HAL_ADC_GetValue(dat->adc);

	dat->sConfig->Channel = dat->curr1_chan;
	dat->sConfig->Rank = 1;
	dat->sConfig->SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(dat->adc, dat->sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_ADC_Start(dat->adc);
	HAL_ADC_PollForConversion(dat->adc, HAL_MAX_DELAY);
	dat->curr1 = HAL_ADC_GetValue(dat->adc);

	dat->sConfig->Channel = dat->curr2_chan;
	dat->sConfig->Rank = 1;
	dat->sConfig->SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(dat->adc, dat->sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_ADC_Start(dat->adc);
	HAL_ADC_PollForConversion(dat->adc, HAL_MAX_DELAY);
	dat->curr2 = HAL_ADC_GetValue(dat->adc);

	dat->sConfig->Channel = dat->ref1_chan;
	dat->sConfig->Rank = 1;
	dat->sConfig->SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(dat->adc, dat->sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_ADC_Start(dat->adc);
	HAL_ADC_PollForConversion(dat->adc, HAL_MAX_DELAY);
	dat->ref1 = HAL_ADC_GetValue(dat->adc);

	dat->sConfig->Channel = dat->ref2_chan;
	dat->sConfig->Rank = 1;
	dat->sConfig->SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(dat->adc, dat->sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_ADC_Start(dat->adc);
	HAL_ADC_PollForConversion(dat->adc, HAL_MAX_DELAY);
	dat->ref2 = HAL_ADC_GetValue(dat->adc);

	dat->state = 1;

}

void check_data(sumo_data_t* dat){

	if(dat->volt < 3000){

		disable(&dat->mot);

	}
	else{

		dat->state = 0;

	}

}

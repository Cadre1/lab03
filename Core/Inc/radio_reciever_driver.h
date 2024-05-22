/*
 * radio_driver.h
 *
 *  Created on: May 17, 2024
 *      Author: cadre
 */

#ifndef INC_RADIO_RECIEVER_DRIVER_H_
#define INC_RADIO_RECIEVER_DRIVER_H_

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

struct rad_struct{
	TIM_HandleTypeDef* tim_handle;
	uint32_t tim_channel1;
	uint32_t tim_channel2;
	uint16_t IC_Rise;
	uint16_t IC_Fall;
	uint16_t IC_Diff;
	uint16_t pulse_width;
	uint16_t tol;
	int pulse_recieve_flag;
	int rise_flag;
} typedef RadioReciever_DriverTypeDef;


void enable_rad(RadioReciever_DriverTypeDef* rad);
void disable_rad(RadioReciever_DriverTypeDef* rad);
uint16_t get_pulse(RadioReciever_DriverTypeDef* rad);
void read_pulse(RadioReciever_DriverTypeDef* rad, TIM_HandleTypeDef* htim);

#endif /* INC_RADIO_RECIEVER_DRIVER_H_ */

/*
 * motor_driver.c
 *
 *  Created on: Apr 23, 2024
 *      Author: cadre
 */


#include "motor_driver.h"

void enable_mot(Motor_DriverTypeDef* mot, uint32_t motor_num){
	if(motor_num == 1){
		  HAL_TIM_PWM_Start(mot->tim_handle,mot->tim_channel1);
		  HAL_TIM_PWM_Start(mot->tim_handle,mot->tim_channel2);
	}
	if(motor_num == 2){
		  HAL_TIM_PWM_Start(mot->tim_handle,mot->tim_channel3);
		  HAL_TIM_PWM_Start(mot->tim_handle,mot->tim_channel4);
	}
}

void disable_mot(Motor_DriverTypeDef* mot, uint32_t motor_num){
	if(motor_num == 1){
		  HAL_TIM_PWM_Stop(mot->tim_handle,mot->tim_channel1);
		  HAL_TIM_PWM_Stop(mot->tim_handle,mot->tim_channel2);
	}
	if(motor_num == 2){
		  HAL_TIM_PWM_Stop(mot->tim_handle,mot->tim_channel3);
		  HAL_TIM_PWM_Stop(mot->tim_handle,mot->tim_channel4);
	}
}

void set_PWM_percent(Motor_DriverTypeDef* mot, uint32_t motor_num, int32_t duty_cycle){
	int32_t duty_count = (duty_cycle*(mot->pulse))/100;
	set_PWM(mot, motor_num, duty_count);
}

void set_PWM(Motor_DriverTypeDef* mot, uint32_t motor_num, int32_t duty_count){
	if (motor_num == 1){
		if (duty_count <= 0){
			(*(mot->tim_handle)).Instance->CCR1 = -duty_count;
			(*(mot->tim_handle)).Instance->CCR2 = 0;
			//__HAL_TIM_SET_COMPARE(mot->tim_handle,mot->tim_channel1,-duty_count);
			//__HAL_TIM_SET_COMPARE(mot->tim_handle,mot->tim_channel2,0);
		}
		else if (duty_count >= 0){
			(*(mot->tim_handle)).Instance->CCR1 = 0;
			(*(mot->tim_handle)).Instance->CCR2 = duty_count;
			//__HAL_TIM_SET_COMPARE(mot->tim_handle,mot->tim_channel1,0);
			//__HAL_TIM_SET_COMPARE(mot->tim_handle,mot->tim_channel2,duty_count);
		}
	}
	else if (motor_num == 2){
		if (duty_count <= 0){
			(*(mot->tim_handle)).Instance->CCR3 = -duty_count;
			(*(mot->tim_handle)).Instance->CCR4 = 0;
			//__HAL_TIM_SET_COMPARE(mot->tim_handle,mot->tim_channel3,-duty_count);
			//__HAL_TIM_SET_COMPARE(mot->tim_handle,mot->tim_channel4,0);
		}
		else if (duty_count >= 0){
			(*(mot->tim_handle)).Instance->CCR3 = 0;
			(*(mot->tim_handle)).Instance->CCR4 = duty_count;
			//__HAL_TIM_SET_COMPARE(mot->tim_handle,mot->tim_channel3,0);
			//__HAL_TIM_SET_COMPARE(mot->tim_handle,mot->tim_channel4,duty_count);
		}
	}
}

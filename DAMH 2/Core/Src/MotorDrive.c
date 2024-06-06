/*
 * MotorDrive.c
 *
 *  Created on: Feb 23, 2024
 *      Author: vutie
 */

#include "MotorDrive.h"
#include "stdlib.h"

void Drive(MotorDrive *motor,TIM_HandleTypeDef *htim2,int Input,unsigned int Channel1,unsigned int Channel2){
	motor->htim2 = htim2;
	motor->Pwm = abs(Input);
	motor->Channel1 = Channel1;
	motor->Channel2 = Channel2;

	if(Input<0){
		__HAL_TIM_SET_COMPARE(motor->htim2,motor->Channel1,0);
		__HAL_TIM_SET_COMPARE(motor->htim2,motor->Channel2,motor->Pwm);

	}
	else if(Input>0){
		__HAL_TIM_SET_COMPARE(motor->htim2,motor->Channel1,motor->Pwm);
		__HAL_TIM_SET_COMPARE(motor->htim2,motor->Channel2,0);
	}
	else{
		__HAL_TIM_SET_COMPARE(motor->htim2,motor->Channel1,0);
		__HAL_TIM_SET_COMPARE(motor->htim2,motor->Channel2,0);
	}

}
//void Drive(MotorDrive *motor,TIM_HandleTypeDef *htim4,uint16_t Mode,int Input,unsigned int Channel1,unsigned int Channel2)
//{
//	motor->htim4 = htim4;
//	motor->Mode = Mode;
//	motor->Pwm = abs(Input);
//	motor->Channel1 = Channel1;
//	motor->Channel2 = Channel2;
//
//	if(motor->Mode==0){
//		if(Input > 0)
//		{
//			__HAL_TIM_SET_COMPARE(motor->htim4,motor->Channel1,motor->Pwm);
//			__HAL_TIM_SET_COMPARE(motor->htim4,motor->Channel2,0);
//		}
//		else if(Input < 0)
//		{
//			__HAL_TIM_SET_COMPARE(motor->htim4,motor->Channel1,0);
//			__HAL_TIM_SET_COMPARE(motor->htim4,motor->Channel2,motor->Pwm);
//		}
//		else
//		{
//			__HAL_TIM_SET_COMPARE(motor->htim4,motor->Channel1,0);
//			__HAL_TIM_SET_COMPARE(motor->htim4,motor->Channel2,0);
//		}
//	}else{
//		if(Input > 0)
//		{
//			__HAL_TIM_SET_COMPARE(motor->htim4,motor->Channel1,1000-motor->Pwm);
//			__HAL_TIM_SET_COMPARE(motor->htim4,motor->Channel2,1000);
//		}
//		else if(Input < 0)
//		{
//			__HAL_TIM_SET_COMPARE(motor->htim4,motor->Channel1,1000);
//			__HAL_TIM_SET_COMPARE(motor->htim4,motor->Channel2,1000-motor->Pwm);
//		}
//		else
//		{
//			__HAL_TIM_SET_COMPARE(motor->htim4,motor->Channel1,1000);
//			__HAL_TIM_SET_COMPARE(motor->htim4,motor->Channel2,1000);
//		}
//	}
//}

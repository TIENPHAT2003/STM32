/*
 * MotorDrive.h
 *
 *  Created on: Feb 23, 2024
 *      Author: vutie
 */

#ifndef INC_DRIVEMOTOR_H_
#define INC_DRIVEMOTOR_H_

#include "main.h"

typedef struct MotorDrive{
	TIM_HandleTypeDef *htim1;
	TIM_HandleTypeDef *htim2;
	int Input;
	int8_t Dir;
	uint16_t Pwm;
	unsigned int Channel1;
	unsigned int Channel2;
}MotorDrive;
void Drive(MotorDrive *motor,TIM_HandleTypeDef *htim2,int Input,unsigned int Channel1,unsigned int Channel2);
#endif /* INC_MOTORDRIVE_H_ */

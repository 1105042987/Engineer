/**
  ******************************************************************************
  * File Name          : AuxDevice.h
  * Description        : IO、PWM外设驱动
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
  
#ifndef __AUXDEVICE_H
#define __AUXDEVICE_H
  
#include "includes.h"
  
#define AMFB_ANGLE_STEP -3.5
#define AMUD_ANGLE_STEP 8.4
#define GMANGLE_STEP 	1.5
  
//IO重命名
#define STEER_TIM 			&htim2
#define BYPASS_TIM 			&htim12
#define YAW_CHANNEL			TIM_CHANNEL_2
#define GIVESML_CHANNEL		TIM_CHANNEL_3
#define GIVEBIG_CHANNEL		TIM_CHANNEL_4
#define BDOOR_CLOSE 		1300
#define SDOOR_CLOSE 		1500
#define E_MAGNET_IO 		GPIOI, GPIO_PIN_2
#define M_VAVLE_FB_IO		GPIOI, GPIO_PIN_0
#define M_VAVLE_OC_IO		GPIOH, GPIO_PIN_12

#define Enable_ElectroMagnet()	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);
#define Disable_ElectroMagnet()	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);
#define Close_Gripper()			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_SET);
#define Open_Gripper()			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_RESET);
#define Open_SDOOR()			__HAL_TIM_SET_COMPARE(STEER_TIM, GIVESML_CHANNEL, SDOOR_CLOSE-900)
#define Close_SDOOR()			__HAL_TIM_SET_COMPARE(STEER_TIM, GIVESML_CHANNEL, SDOOR_CLOSE)
#define Open_BDOOR()			__HAL_TIM_SET_COMPARE(STEER_TIM, GIVESML_CHANNEL, BDOOR_CLOSE+900)
#define Close_BDOOR()			__HAL_TIM_SET_COMPARE(STEER_TIM, GIVESML_CHANNEL, BDOOR_CLOSE)


#define InitPWM()\
{\
	/*HAL_TIM_PWM_Start(STEER_TIM, YAW_CHANNEL);\
	__HAL_TIM_SET_COMPARE(STEER_TIM, YAW_CHANNEL,0);\
	HAL_TIM_PWM_Start(BYPASS_TIM, TIM_CHANNEL_1);*/\
	HAL_TIM_PWM_Start(STEER_TIM, GIVESML_CHANNEL);\
	__HAL_TIM_SET_COMPARE(STEER_TIM, GIVESML_CHANNEL, SDOOR_CLOSE);\
	HAL_TIM_PWM_Start(STEER_TIM, GIVEBIG_CHANNEL);\
	__HAL_TIM_SET_COMPARE(STEER_TIM, GIVEBIG_CHANNEL, BDOOR_CLOSE);\
}
  
  
#endif

/**
  ******************************************************************************
  * File Name          : GMControlTask.h
  * Description        : 云台电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 
	* 
  ******************************************************************************
  */
#ifndef GM_CONTROL_TASK_H
#define GM_CONTROL_TASK_H

#include "includes.h"

#define GMYAWReduction 96.0
#define GMPITCHReduction 36.0

extern float GSYAW_ZERO;

void ControlGSYAW(void);
void ControlGMYAW(void);
void ControlGMPITCH(void);
void setGMMotor(void);
void GMControlInit(void);

#endif //GM_CONTROL_TASK_H

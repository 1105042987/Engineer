/**
  ******************************************************************************
  * File Name          : AMControlTask.h
  * Description        : 取弹送弹机械臂电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 
	* 
  ******************************************************************************
  */
#ifndef AM_CONTROL_TASK_H
#define AM_CONTROL_TASK_H

#include "includes.h"

#define AMReduction 36.0
#define BYPASS_TIM htim12

void vice_controlLoop(void);
void AMControlInit(void);

#endif //AM_CONTROL_TASK_H

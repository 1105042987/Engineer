/**
  ******************************************************************************
  * File Name          : includes.h
  * Description        : 统一包含文件
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __INCLUDES_H
#define __INCLUDES_H

//#define DEBUG_MODE

#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "RemoteTask.h"
#include "pid_regulator.h"
#include "ControlTask.h"
#include "AMControlTask.h"
#include "IMUTask.h"
#include "CANTask.h"
#include "drivers_ramp.h"
#include "ManifoldTask.h"
#include "JudgeTask.h"
#include "UpperTask.h"


extern int16_t times;
//#include "visualscope.h"


//所有待调整参数，可以用查找找到位置
//AMUD1PositionPID =             			以及后跟随9项
//ANGLE_STEP													机械臂电机转动速度或是步进长度
//IGNORE_RANGE												摇杆在区间内不会触动2310电机，防止误触，摇到极限400多一些
//ROTATE_FACTOR												底盘旋转速度控制系数



//！！！！！！！！！强烈建议先在单个电机上调整好再烧入！！！！！！！！！！

//所有标记delete的内容是为了适配步兵调试底盘运动的，可以删去
//所有标记ATTENTION!!的内容是自己感觉可能有问题的

#endif /* __INCLUDES_H */

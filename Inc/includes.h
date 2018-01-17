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


//所有待调整参数，可以用查找找到位置
//IMURefreshInterval 									感觉积分时间间隔取得有点长（0.1）导致了第一次崩溃
//CHASSIS_MOTOR_SPEED_PID_DEFAULT  		不过好像底盘电机的PID参数可以不用调整
//AMUD1PositionPID =             			以及后跟随9项
//ANGLE_STEP													机械臂电机转动速度或是步进长度
//IGNORE_RANGE												摇杆在区间内不会触动2310电机，防止误触，摇到极限400多一些

//【非查找数据】更改TIM12PWM通道1的占空比，可以通过stm32cube访问，以及查找引号内“__HAL_TIM_SET_COMPARE(&BYPASS_TIM, TIM_CHANNEL_1,”
//更改后跟的参考值大小（我设置成1000了），让涵道电机正常工作

//STEPMODE														留着这一句是设置角度模式，去掉是速度模式，但是感觉角度更应该是对的
//																		这样做是以为现在有点不确定是PID问题导致的RealAngle追不上TargetAngle，还是运算逻辑导致的错误
//																		还有一个问题，不知道是不是debug的原因，遥控器任务经常被饿死
//																		建议问一下小枫学长饿死的问题


//！！！！！！！！！强烈建议先在单个电机上调整好再烧入！！！！！！！！！！

//所有标记delete的内容是为了适配步兵调试底盘运动的，可以删去
//所有标记ATTENTION!!的内容是自己感觉可能有问题的

#endif /* __INCLUDES_H */

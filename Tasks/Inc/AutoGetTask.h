/**
  ******************************************************************************
  * File Name          : AutoGetTask.h
  * Description        : 自动取弹控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __AUTO_GET_TASK_H
#define __AUTO_GET_TASK_H

#include "includes.h"

#define CHANGE_POINT 2200
//1v/5v 	27cm

#define FLAG_SET(val, flag)\
if(val>=CHANGE_POINT)\
{\
	flag = 1;\
}\
else\
{\
	flag = 0;\
}\

typedef __packed struct
{
	uint32_t vol_ref;
	int8_t flag;						//1阻断，0开放
}Distance_Sensor_t;

typedef __packed struct
{
	Distance_Sensor_t front;
	Distance_Sensor_t leftin;
	Distance_Sensor_t leftout;
	Distance_Sensor_t rightin;
	Distance_Sensor_t rightout;
	int8_t move_flags;
}Distance_Couple_t;

//move_flags 16进制：编码准则：lo li ri ro
//偏离   |对 准|    中 间    |对 准|   偏离
//0 1   3 2 6 4 c   d f b   3 2 6 4 c   8 0
//----------------------------------------> 'r'
//<---------------------------------------- 'l'

typedef enum
{
	NOAUTO_STATE,
	START_TEST,
	LEVEL_SHIFT,
	ARM_STRETCH,
	ERROR_HANDLE
}Engineer_State_e;

void AutoGet(char signal,uint8_t flag);

#endif //__AUTO_GET_TASK_H

/**
  ******************************************************************************
  * File Name          : JudgeTask.h
  * Description        : 裁判系统处理任务，得到裁判系统信息
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __JUDGETASK_H
#define __JUDGETASK_H

#include "includes.h"

typedef struct 
{
    uint32_t remainTime;
    uint16_t remainLifeValue;
    float realChassisOutV;
    float realChassisOutA;
    float remainPower;
}tGameInfo;

typedef struct 
{
	float data1;
  float data2;
  float data3;
	uint8_t mask;
}tUserData;

typedef struct 
{
	uint8_t sof;
	uint16_t data_length;
	uint8_t seq;
	uint8_t crc8;
}frame_header_t;

typedef struct
{
	frame_header_t head;
	uint16_t cmdID;
	tUserData data;
	uint16_t CRC16;
}SendData_t;

typedef enum
{
	ONLINE,
	OFFLINE
}JudgeState_e;

void judgeUartRxCpltCallback(void);
void InitJudgeUart(void);
void Judge_Refresh(void);
void Send_User_Data(tUserData *data, uint16_t len);

#endif /*__ JUDGETASK_H */

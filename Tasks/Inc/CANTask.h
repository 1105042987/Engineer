/**
  ******************************************************************************
  * File Name          : CANTask.h
  * Description        : CAN通信任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __CANTASK_H
#define __CANTASK_H

#include "includes.h"

#define CMFL_RXID 0x201u
#define CMFR_RXID 0x202u
#define CMBL_RXID 0x203u
#define CMBR_RXID 0x204u


typedef __packed struct{
	uint16_t angle;
	int16_t RotateSpeed;//RPM
}Motor820RRxMsg_t;

typedef __packed struct{
	uint16_t angle;
	int16_t RotateSpeed;//RPM
	uint16_t moment;
}ESCC6x0RxMsg_t;



extern ESCC6x0RxMsg_t CMFLRx;
extern ESCC6x0RxMsg_t CMFRRx;
extern ESCC6x0RxMsg_t CMBLRx;
extern ESCC6x0RxMsg_t CMBRRx;


extern uint8_t can1_update;
extern uint8_t can2_update;
extern uint8_t can1_type;
extern uint8_t can2_type;

void InitCanReception(void);

#endif /*__ CANTASK_H */

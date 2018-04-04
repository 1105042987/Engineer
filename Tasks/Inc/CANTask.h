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

#define CMMOTOR_CAN hcan1
#define AUXMOTOR_CAN hcan2
#define GMMOTOR_CAN hcan2

//RxID
//can1
#define CMFL_RXID 0x201u
#define CMFR_RXID 0x202u
#define CMBL_RXID 0x203u
#define CMBR_RXID 0x204u

//can2
#define AMUD1_RXID 0x201u
#define AMUD2_RXID 0x202u
#define AMFB_RXID 0x203u

#define GMYAW_RXID 0x205u
//			GSYAW 用PA2引脚
#define GMPITCH_RXID 0x206u


//TxID
//can1
#define CM_TXID 0x200u
//can2
#define AM_TXID 0x200u	

#define GM_TXID 0x1FFu

typedef struct{
	uint16_t angle;
	int16_t RotateSpeed;//RPM
}Motor820RRxMsg_t;



extern Motor820RRxMsg_t CMFLRx;
extern Motor820RRxMsg_t CMFRRx;
extern Motor820RRxMsg_t CMBLRx;
extern Motor820RRxMsg_t CMBRRx;
extern Motor820RRxMsg_t AMUD1Rx;
extern Motor820RRxMsg_t AMUD2Rx;
extern Motor820RRxMsg_t AMFBRx; 
extern Motor820RRxMsg_t GMYAWRx;
extern Motor820RRxMsg_t GMPITCHRx;

extern uint8_t can1_update;
extern uint8_t can2_update;
extern uint8_t can_type;

void InitCanReception(void);

#endif /*__ CANTASK_H */

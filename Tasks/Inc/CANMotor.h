/**
  ******************************************************************************
  * File Name          : CANMotor.h
  * Description        : CAN线电机控制
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __CANMOTOR_H
#define __CANMOTOR_H

#include "includes.h"

typedef __packed struct
{
	CAN_HandleTypeDef* 	CAN_TYPE;
	uint16_t 			TXID;
	uint16_t			RXID;
	float 				ReductionRate;
	ESCC6x0RxMsg_t		RxMsg;
	double 				TargetAngle;
	uint8_t				s_count;
	uint8_t 			FirstEnter;
	uint16_t 			lastRead;
	double 				RealAngle;
	fw_PID_Regulator_t 	positionPID;
	fw_PID_Regulator_t 	speedPID;
	uint16_t			Intensity;
}MotorINFO;

#define MOTORINFO_Init(can,txid,rxid,rdc,ppid,spid)\
{\
	can,txid,rxid,rdc,\
	{0,0,0},0,0,1,0,0,\
	ppid,spid,0\
}

extern MotorINFO UD1,UD2,GMP,GMY,AMR,CML,CMR;
extern MotorINFO *can1[8],*can2[8];

void InitMotor(MotorINFO *id);
void ControlMotor(MotorINFO *id);
void setCAN21(void);
void setCAN12(void);
void setCAN22(void);

#endif /*__ CANMOTOR_H */

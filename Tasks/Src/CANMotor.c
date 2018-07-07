/**
  ******************************************************************************
  * File Name          : CANMotot.c
  * Description        : CAN电机统一驱动任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

MotorINFO UD1 = MOTORINFO_Init( &hcan1,0x1ff,0x205,19.0,
								fw_PID_INIT(1200.0, 0.0, 0.0, 	15000.0, 15000.0, 15000.0, 15000.0),
								fw_PID_INIT(1, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0));
MotorINFO UD2 = MOTORINFO_Init( &hcan1,0x1ff,0x206,19.0,
								fw_PID_INIT(1200.0, 0.0, 0.0, 	15000.0, 15000.0, 15000.0, 15000.0),
								fw_PID_INIT(1, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0));


MotorINFO AMR = MOTORINFO_Init( &hcan2,0x200,0x203,19.0,
								fw_PID_INIT(1200.0, 0.0, 0.0, 	10000.0, 10000.0, 10000.0, 10000.0),
								fw_PID_INIT(1, 0.0, 0.0, 		10000.0, 10000.0, 10000.0, 4000.0));
								
								
MotorINFO GMP = MOTORINFO_Init( &hcan2,0x200,0x201,36.0,
								fw_PID_INIT(1200.0, 0.0, 0.0, 	10000.0, 10000.0, 10000.0, 10000.0),
								fw_PID_INIT(0.7, 0.0, 0.0, 		10000.0, 10000.0, 10000.0, 4000.0));
MotorINFO GMY = MOTORINFO_Init( &hcan2,0x200,0x202,36.0,
								fw_PID_INIT(1200.0, 0.0, 0.0, 	10000.0, 10000.0, 10000.0, 10000.0),
								fw_PID_INIT(0.7, 0.0, 0.0, 		10000.0, 10000.0, 10000.0, 4000.0));
								
								
MotorINFO CML = MOTORINFO_Init( &hcan2,0x1ff,0x205,19.0,
								fw_PID_INIT(1200.0, 0.0, 0.0, 	15000.0, 15000.0, 15000.0, 15000.0),
								fw_PID_INIT(1, 0.0, 1.0, 		15000.0, 15000.0, 15000.0, 15000.0));
MotorINFO CMR = MOTORINFO_Init( &hcan2,0x1ff,0x206,19.0,
								fw_PID_INIT(1200.0, 0.0, 0.0,	15000.0, 15000.0, 15000.0, 15000.0),
								fw_PID_INIT(1, 0.0, 1.0, 		15000.0, 15000.0, 15000.0, 15000.0));

MotorINFO* can1[8]={0,0,0,0,&UD1,&UD2,0,0};
MotorINFO* can2[8]={&GMP,&GMY,&AMR,0,&CML,&CMR,0,0};

void ControlMotor(MotorINFO* id)
{
	if(id==0) return;
	if(id->s_count == 1)
	{		
		uint16_t 	ThisAngle;	
		double 		ThisSpeed;	
		ThisAngle = id->RxMsg.angle;				//未处理角度
		if(id->FirstEnter==1) {id->lastRead = ThisAngle;id->FirstEnter = 0;return;}
		if(ThisAngle<=id->lastRead)
		{
			if((id->lastRead-ThisAngle)>3000)//编码器上溢
				id->RealAngle = id->RealAngle + (ThisAngle+8192-id->lastRead) * 360 / 8192.0 / id->ReductionRate;
			else//正常
				id->RealAngle = id->RealAngle - (id->lastRead - ThisAngle) * 360 / 8192.0 / id->ReductionRate;
		}
		else
		{
			if((ThisAngle-id->lastRead)>3000)//编码器下溢
				id->RealAngle = id->RealAngle - (id->lastRead+8192-ThisAngle) *360 / 8192.0 / id->ReductionRate;
			else//正常
				id->RealAngle = id->RealAngle + (ThisAngle - id->lastRead) * 360 / 8192.0 / id->ReductionRate;
		}
		ThisSpeed = id->RxMsg.RotateSpeed * 6;		//单位：度每秒
		
		id->Intensity = PID_PROCESS_Double(id->positionPID,id->speedPID,id->TargetAngle,id->RealAngle,ThisSpeed);
		
		id->s_count = 0;
		id->lastRead = ThisAngle;
	}
	else
	{
		id->s_count++;
	}		
}

void setCAN12()
{
	CanTxMsgTypeDef pData;
	hcan1.pTxMsg = &pData;
	
	hcan1.pTxMsg->StdId = 0x1ff;
	hcan1.pTxMsg->ExtId = 0;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can1[i+4]==0) {
			hcan1.pTxMsg->Data[i*2]   = 0;
			hcan1.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan1.pTxMsg->Data[i*2]   = (uint8_t)(can1[i+4]->Intensity >> 8);
			hcan1.pTxMsg->Data[i*2+1] = (uint8_t)can1[i+4]->Intensity;
		}
	}

	if(can1_update == 1 && can1_type == 2)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK)
		{
			Error_Handler();
		}
		can1_update = 0;
		can1_type = 1;
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
  }
}
void setCAN22()
{
	CanTxMsgTypeDef pData;
	hcan2.pTxMsg = &pData;
	
	hcan2.pTxMsg->StdId = 0x1ff;
	hcan2.pTxMsg->ExtId = 0;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can2[i+4]==0) {
			hcan2.pTxMsg->Data[i*2]   = 0;
			hcan2.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan2.pTxMsg->Data[i*2]   = (uint8_t)(can2[i+4]->Intensity >> 8);
			hcan2.pTxMsg->Data[i*2+1] = (uint8_t)can2[i+4]->Intensity;
		}
	}

	if(can2_update == 1 && can2_type == 2)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan2) != HAL_OK)
		{
			Error_Handler();
		}
		can2_update = 0;
		can2_type = 1;
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
  }
}
void setCAN21()
{
	CanTxMsgTypeDef pData;
	hcan2.pTxMsg = &pData;
	
	hcan2.pTxMsg->StdId = 0x200;
	hcan2.pTxMsg->ExtId = 0;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can2[i]==0) {
			hcan2.pTxMsg->Data[i*2]   = 0;
			hcan2.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan2.pTxMsg->Data[i*2]   = (uint8_t)(can2[i]->Intensity >> 8);
			hcan2.pTxMsg->Data[i*2+1] = (uint8_t)can2[i]->Intensity;
		}
	}

	if(can2_update == 1 && can2_type == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan2) != HAL_OK)
		{
			Error_Handler();
		}
		can2_update = 0;
		#ifdef CAN22
		can2_type = 2;
		#endif
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
  }
}

void InitMotor(MotorINFO *id)
{
	if(id==0) return;
	id->FirstEnter=1;
	id->lastRead=0;
	id->RealAngle=0;
	id->TargetAngle=0;
	
	ControlMotor(id);
}

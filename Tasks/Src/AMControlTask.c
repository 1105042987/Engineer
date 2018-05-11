/**
  ******************************************************************************
  * File Name          : AMControlTask.c
  * Description        : 机械臂控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

uint16_t AMUDIntensity=0;
extern uint16_t GMPITCHIntensity;

fw_PID_Regulator_t AMUDPositionPID = fw_PID_INIT(1200.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);

fw_PID_Regulator_t AMUDSpeedPID = fw_PID_INIT(1.5, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);



//机械臂电机实际物理角度值
double AMUDRealAngle = 0.0;

//机械臂电机上次物理角度值
uint16_t AMUDLastAngle = 0.0;

//是否初次进入
static uint8_t AMUDFirstEnter = 1;

//用于减小系统开销
static uint8_t s_AMUDCount = 0;

void setAGMMotor()
{
	CanTxMsgTypeDef pData;
	AGMOTOR_CAN.pTxMsg = &pData;
	
	AGMOTOR_CAN.pTxMsg->StdId = AGM_TXID;
	AGMOTOR_CAN.pTxMsg->ExtId = 0;
	AGMOTOR_CAN.pTxMsg->IDE = CAN_ID_STD;
	AGMOTOR_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	AGMOTOR_CAN.pTxMsg->DLC = 0x08;
	
	AGMOTOR_CAN.pTxMsg->Data[0] = (uint8_t)(AMUDIntensity >> 8);
	AGMOTOR_CAN.pTxMsg->Data[1] = (uint8_t)AMUDIntensity;
	AGMOTOR_CAN.pTxMsg->Data[2] = (uint8_t)(GMPITCHIntensity >> 8);
	AGMOTOR_CAN.pTxMsg->Data[3] = (uint8_t)GMPITCHIntensity;
	AGMOTOR_CAN.pTxMsg->Data[4] = (uint8_t)((-AMUDIntensity) >> 8);
	AGMOTOR_CAN.pTxMsg->Data[5] = (uint8_t)(-AMUDIntensity);
	AGMOTOR_CAN.pTxMsg->Data[6] = 0;
	AGMOTOR_CAN.pTxMsg->Data[7] = 0;

	if(can1_update == 1 && can_type == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&AGMOTOR_CAN) != HAL_OK)
		{
			Error_Handler();
		}
		can1_update = 0;
		can_type = 0;
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
//机械臂电机累计角度解算
void StandardlizeAMRealAngle(double* AMRealAngle,uint16_t AMThisAngle,uint16_t AMLastAngle)
{
	if(AMThisAngle<=AMLastAngle)
		{
			if((AMLastAngle-AMThisAngle)>3000)//编码器上溢
				*AMRealAngle = *AMRealAngle + (AMThisAngle+8192-AMLastAngle) * 360 / 8192.0 / AMReduction;
			else//反转
				*AMRealAngle = *AMRealAngle - (AMLastAngle - AMThisAngle) * 360 / 8192.0 / AMReduction;
		}
	else
		{
			if((AMThisAngle-AMLastAngle)>3000)//编码器下溢
				*AMRealAngle = *AMRealAngle - (AMLastAngle+8192-AMThisAngle) *360 / 8192.0 / AMReduction;
			else//正转
				*AMRealAngle = *AMRealAngle + (AMThisAngle - AMLastAngle) * 360 / 8192.0 / AMReduction;
		}
}


void ControlAMUD()
{
		if(s_AMUDCount == 1)
		{		
			uint16_t 	ThisAngle;	
			double 		ThisSpeed;	
			ThisAngle = AMUDRx.angle;							//未处理角度
			if(AMUDFirstEnter==1) {AMUDLastAngle = ThisAngle;AMUDFirstEnter = 0;return;}
			StandardlizeAMRealAngle(&AMUDRealAngle,ThisAngle,AMUDLastAngle);//处理
			ThisSpeed = AMUDRx.RotateSpeed * 6;		//单位：度每秒
			
			AMUDIntensity = PID_PROCESS_Double(AMUDPositionPID,AMUDSpeedPID,AMUDAngleTarget,AMUDRealAngle,ThisSpeed);
			
			s_AMUDCount = 0;
			AMUDLastAngle = ThisAngle;
		}
		else
		{
			s_AMUDCount++;
		}
}



//副控制循环
void vice_controlLoop()
{

		ControlAMUD();
	
		//ControlGMYAW();
		ControlGSYAW();
		ControlGMPITCH();
		
		setAGMMotor();
}

//机械臂控制初始化
void AMControlInit()
{
	AMUDFirstEnter = 1;
	AMUDRealAngle = 0.0;
	AMUDLastAngle = 0.0;
	
	ControlAMUD();
	HAL_TIM_PWM_Start(&BYPASS_TIM, TIM_CHANNEL_1);
}

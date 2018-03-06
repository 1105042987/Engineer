/**
  ******************************************************************************
  * File Name          : AMControlTask.c
  * Description        : 取弹送弹机械臂控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

uint16_t AMUD1Intensity=0,AMUD2Intensity=0,AMFBIntensity=0,AMSIDEIntensity=0,WINDIntensity=0;

fw_PID_Regulator_t AMUD1PositionPID = fw_PID_INIT(1200.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AMUD2PositionPID = fw_PID_INIT(1200.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AMFBPositionPID = fw_PID_INIT(500.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AMSIDEPositionPID = fw_PID_INIT(500.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t WINDPositionPID = fw_PID_INIT(500.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);

fw_PID_Regulator_t AMUD1SpeedPID = fw_PID_INIT(0.7, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t AMUD2SpeedPID = fw_PID_INIT(0.7, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t AMFBSpeedPID = fw_PID_INIT(0.7, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t AMSIDESpeedPID = fw_PID_INIT(0.7, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t WINDSpeedPID = fw_PID_INIT(0.7, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);



//机械臂电机实际物理角度值
double AMUD1RealAngle = 0.0;
double AMUD2RealAngle = 0.0;
double AMFBRealAngle = 0.0;
double AMSIDERealAngle = 0.0;
double WINDRealAngle = 0.0;

//机械臂电机上次物理角度值
uint16_t AMUD1LastAngle = 0.0;
uint16_t AMUD2LastAngle = 0.0;
uint16_t AMFBLastAngle = 0.0;
uint16_t AMSIDELastAngle = 0.0;
uint16_t WINDLastAngle = 0.0;

//是否初次进入
static uint8_t AMUD1FirstEnter = 1;
static uint8_t AMUD2FirstEnter = 1;
static uint8_t AMFBFirstEnter = 1;
static uint8_t AMSIDEFirstEnter = 1;
static uint8_t WINDFirstEnter = 1;

//用于减小系统开销
static uint8_t s_AMUD1Count = 0;
static uint8_t s_AMUD2Count = 0;
static uint8_t s_AMFBCount = 0;
static uint8_t s_AMSIDECount = 0;
static uint8_t s_WINDCount = 0;


//送弹机械臂电机CAN信号控制
void setSeldomAMMotor()
{
	CanTxMsgTypeDef pData;
	AUXMOTOR_CAN.pTxMsg = &pData;
	
	AUXMOTOR_CAN.pTxMsg->StdId = AMSELDOM_TXID;
	AUXMOTOR_CAN.pTxMsg->ExtId = 0;
	AUXMOTOR_CAN.pTxMsg->IDE = CAN_ID_STD;
	AUXMOTOR_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	AUXMOTOR_CAN.pTxMsg->DLC = 0x08;
	
	AUXMOTOR_CAN.pTxMsg->Data[0] = (uint8_t)(AMFBIntensity >> 8);
	AUXMOTOR_CAN.pTxMsg->Data[1] = (uint8_t)AMFBIntensity;
	AUXMOTOR_CAN.pTxMsg->Data[2] = 0;
	AUXMOTOR_CAN.pTxMsg->Data[3] = 0;
	AUXMOTOR_CAN.pTxMsg->Data[4] = 0;
	AUXMOTOR_CAN.pTxMsg->Data[5] = 0;
	AUXMOTOR_CAN.pTxMsg->Data[6] = 0;
	AUXMOTOR_CAN.pTxMsg->Data[7] = 0;

	if(can2_update == 1 && can_type == 0)
	{
		//CAN通信前关中断
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif

		if(HAL_CAN_Transmit_IT(&AUXMOTOR_CAN) != HAL_OK)
		{
			Error_Handler();
		}
		can2_update = 0;
		can_type = 1;
		
		//CAN通信后开中断，防止中断影响CAN信号发送
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

//取弹机械臂电机CAN信号控制
void setUsalAMMotor(void)
{
	CanTxMsgTypeDef pData;
	AUXMOTOR_CAN.pTxMsg = &pData;
	
	AUXMOTOR_CAN.pTxMsg->StdId = AMUSAL_TXID;
	AUXMOTOR_CAN.pTxMsg->ExtId = 0;
	AUXMOTOR_CAN.pTxMsg->IDE = CAN_ID_STD;
	AUXMOTOR_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	AUXMOTOR_CAN.pTxMsg->DLC = 0x08;
	
	AUXMOTOR_CAN.pTxMsg->Data[0] = (uint8_t)(AMUD1Intensity >> 8);
	AUXMOTOR_CAN.pTxMsg->Data[1] = (uint8_t)AMUD1Intensity;
	AUXMOTOR_CAN.pTxMsg->Data[2] = (uint8_t)(AMUD2Intensity >> 8);
	AUXMOTOR_CAN.pTxMsg->Data[3] = (uint8_t)AMUD2Intensity;
	AUXMOTOR_CAN.pTxMsg->Data[4] = (uint8_t)(AMSIDEIntensity >> 8);
	AUXMOTOR_CAN.pTxMsg->Data[5] = (uint8_t)AMSIDEIntensity;
	AUXMOTOR_CAN.pTxMsg->Data[6] = (uint8_t)(WINDIntensity >> 8);
	AUXMOTOR_CAN.pTxMsg->Data[7] = (uint8_t)WINDIntensity;

	if(can2_update == 1 && can_type == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&AUXMOTOR_CAN) != HAL_OK)
		{
			Error_Handler();
		}
		can2_update = 0;
		if(WorkState == GETBULLET_STATE || WorkState == BYPASS_STATE) can_type = 0;
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
//机械臂电机真实角度解算
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

//各个机械臂电机控制
void ControlAMFB()
{
		if(s_AMFBCount == 1)
		{		
		#ifdef EXACT_CONTROL
			uint16_t 	ThisAngle;	//当前电机角度
			double 		ThisSpeed;	//当前电机转速
			ThisAngle = AMFBRx.angle;//未处理角度
			if(AMFBFirstEnter==1) 
			{//初始化时，记录下当前编码器的值
				AMFBLastAngle = ThisAngle;
				AMFBFirstEnter = 0;
				return;
			}
			StandardlizeAMRealAngle(&AMFBRealAngle,ThisAngle,AMFBLastAngle);//处理
			ThisSpeed = AMFBRx.RotateSpeed * 6;		//单位：度每秒
			
			AMFBIntensity = PID_PROCESS_Double(AMFBPositionPID,AMFBSpeedPID,AMFBAngleTarget,AMFBRealAngle,ThisSpeed);
			
			s_AMFBCount = 0;
			AMFBLastAngle = ThisAngle;
		#endif
		#ifdef EASY_CONTROL
			AMFBIntensity = AMFBAngleTarget * 12;
		#endif
		}
		else
		{
			s_AMFBCount++;
		}
}
void ControlAMUD1()
{
		if(s_AMUD1Count == 1)
		{	
		#ifdef EXACT_CONTROL
			uint16_t 	ThisAngle;	
			double 		ThisSpeed;	
			ThisAngle = AMUD1Rx.angle;							//未处理角度
			if(AMUD1FirstEnter==1) {AMUD1LastAngle = ThisAngle;AMUD1FirstEnter = 0;return;}
			StandardlizeAMRealAngle(&AMUD1RealAngle,ThisAngle,AMUD1LastAngle);//处理
			ThisSpeed = AMUD1Rx.RotateSpeed * 6;		//单位：度每秒
			
			AMUD1Intensity = PID_PROCESS_Double(AMUD1PositionPID,AMUD1SpeedPID,AMUD1AngleTarget,AMUD1RealAngle,ThisSpeed);
			
			s_AMUD1Count = 0;
			AMUD1LastAngle = ThisAngle;
		#endif
		#ifdef EASY_CONTROL
			AMUD1Intensity = -800 + AMUD1AngleTarget * 9;
		#endif
		}
		else
		{
			s_AMUD1Count++;
		}
}
void ControlAMUD2()
{
		if(s_AMUD2Count == 1)
		{		
		#ifdef EXACT_CONTROL
			uint16_t 	ThisAngle;	
			double 		ThisSpeed;	
			ThisAngle = AMUD2Rx.angle;							//未处理角度
			if(AMUD2FirstEnter==1) {AMUD2LastAngle = ThisAngle;AMUD2FirstEnter = 0;return;}
			StandardlizeAMRealAngle(&AMUD2RealAngle,ThisAngle,AMUD2LastAngle);//处理
			ThisSpeed = AMUD2Rx.RotateSpeed * 6;		//单位：度每秒
			
			AMUD2Intensity = PID_PROCESS_Double(AMUD2PositionPID,AMUD2SpeedPID,AMUD2AngleTarget,AMUD2RealAngle,ThisSpeed);
			
			s_AMUD2Count = 0;
			AMUD2LastAngle = ThisAngle;
		#endif
		#ifdef EASY_CONTROL
			AMUD2Intensity = -800 + AMUD2AngleTarget * 9;
		#endif
		}
		else
		{
			s_AMUD2Count++;
		}
}
void ControlWIND()
{
		if(s_WINDCount == 1)
		{	
		#ifdef EXACT_CONTROL		
			uint16_t 	ThisAngle;	
			double 		ThisSpeed;	
			ThisAngle = WINDRx.angle;							//未处理角度
			if(WINDFirstEnter==1) {WINDLastAngle = ThisAngle;WINDFirstEnter = 0;return;}
			StandardlizeAMRealAngle(&WINDRealAngle,ThisAngle,WINDLastAngle);//处理
			ThisSpeed = WINDRx.RotateSpeed * 6;		//单位：度每秒
			
			WINDIntensity = PID_PROCESS_Double(WINDPositionPID,WINDSpeedPID,WINDAngleTarget,WINDRealAngle,ThisSpeed);
			
			s_WINDCount = 0;
			WINDLastAngle = ThisAngle;
		#endif
		#ifdef EASY_CONTROL
			WINDIntensity = 500 + WINDAngleTarget * 10;
		#endif
		}
		else
		{
			s_WINDCount++;
		}
}
void ControlAMSIDE()
{
	if(s_AMSIDECount == 1)
	{		
	#ifdef EXACT_CONTROL
		uint16_t 	ThisAngle;	
		double 		ThisSpeed;	
			
		ThisAngle = AMSIDERx.angle;
		if(AMSIDEFirstEnter==1) {
			AMSIDELastAngle = ThisAngle;
			AMSIDEFirstEnter = 0;
			return;
		}
		StandardlizeAMRealAngle(&AMSIDERealAngle,ThisAngle,AMSIDELastAngle);
		ThisSpeed = AMSIDERx.RotateSpeed * 6;		//单位：度每秒
		
		AMSIDEIntensity = -PID_PROCESS_Double(AMSIDEPositionPID,AMSIDESpeedPID,AMSIDEAngleTarget,AMSIDERealAngle,ThisSpeed);
		
		s_AMSIDECount = 0;
		AMSIDELastAngle = ThisAngle;
	#endif
	#ifdef EASY_CONTROL
		AMSIDEIntensity = 750 + AMSIDEAngleTarget * 6;
	#endif
	}
	else
	{
		s_AMSIDECount++;
	}
}




//副控制循环
void vice_controlLoop()
{

		ControlAMUD1();
		ControlAMUD2();
		ControlAMSIDE();
		ControlWIND();
	
		setUsalAMMotor();
	
		if(WorkState == BYPASS_STATE||WorkState == GETBULLET_STATE)
		{
			ControlAMFB();
			setSeldomAMMotor();
		}
	
		if(WorkState == BYPASS_STATE)
		{
			__HAL_TIM_SET_COMPARE(&BYPASS_TIM, TIM_CHANNEL_1,1300);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&BYPASS_TIM, TIM_CHANNEL_1,1000);
		}
}

//机械臂控制初始化
void AMControlInit()
{
	AMUD1FirstEnter = 1;
	AMUD2FirstEnter = 1;
	AMFBFirstEnter = 1;
	AMSIDEFirstEnter = 1;
	WINDFirstEnter = 1;
	AMUD1RealAngle = 0.0;
	AMUD2RealAngle = 0.0;
	AMFBRealAngle = 0.0;
	AMSIDERealAngle = 0.0;
	WINDRealAngle = 0.0;
	AMUD1LastAngle = 0.0;
	AMUD2LastAngle = 0.0;
	AMFBLastAngle = 0.0;
	AMSIDELastAngle = 0.0;
	WINDLastAngle = 0.0;
	
	ControlAMUD1();
	ControlAMUD2();
	ControlAMFB();
	ControlWIND();
	ControlAMSIDE();
	HAL_TIM_PWM_Start(&BYPASS_TIM, TIM_CHANNEL_1);
}

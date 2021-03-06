/**
  ******************************************************************************
  * File Name          : ControlTask.c
  * Description        : 主控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"


#define LED_GREEN_TOGGLE() HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
#define LED_RED_TOGGLE()   HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin)
#define LED_GREEN_OFF()    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_SET)
#define LED_RED_OFF()      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_SET)
#define LED_GREEN_ON()     HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_RESET)
#define LED_RED_ON()       HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_RESET)

WorkState_e WorkState = PREPARE_STATE;
uint16_t prepare_time = 0;

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

int16_t CMFLIntensity = 0, CMFRIntensity = 0, CMBLIntensity = 0, CMBRIntensity = 0;

//底盘PID初始化
void CMControlInit(void)
{
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	CM3SpeedPID.Reset(&CM3SpeedPID);
	CM4SpeedPID.Reset(&CM4SpeedPID);
}

//单个底盘电机的控制，下同
void ControlCMFL(void)
{			
	CM1SpeedPID.ref =  	  ChassisSpeedRef.forward_back_ref*0.075 
						+ ChassisSpeedRef.left_right_ref*0.075 
						+ ChassisSpeedRef.rotate_ref*0.075;	
	CM1SpeedPID.ref = 160 * CM1SpeedPID.ref;
			
			
	CM1SpeedPID.fdb = CMFLRx.RotateSpeed;

	CM1SpeedPID.Calc(&CM1SpeedPID);
	CMFLIntensity = CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output;
}

void ControlCMFR(void)
{			
	CM2SpeedPID.ref = 	- ChassisSpeedRef.forward_back_ref*0.075 
						+ ChassisSpeedRef.left_right_ref*0.075 
						+ ChassisSpeedRef.rotate_ref*0.075;
	CM2SpeedPID.ref = 160 * CM2SpeedPID.ref;
			
			
	CM2SpeedPID.fdb = CMFRRx.RotateSpeed;

	CM2SpeedPID.Calc(&CM2SpeedPID);
	CMFRIntensity = CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output;
}

void ControlCMBL(void)
{		
	CM3SpeedPID.ref =  	  ChassisSpeedRef.forward_back_ref*0.075 
						- ChassisSpeedRef.left_right_ref*0.075 
						+ ChassisSpeedRef.rotate_ref*0.075;
	CM3SpeedPID.ref = 160 * CM3SpeedPID.ref;
			
			
	CM3SpeedPID.fdb = CMBLRx.RotateSpeed;

	CM3SpeedPID.Calc(&CM3SpeedPID);
	CMBLIntensity = CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.output;
}

void ControlCMBR(void)
{		
	CM4SpeedPID.ref = 	- ChassisSpeedRef.forward_back_ref*0.075 
						- ChassisSpeedRef.left_right_ref*0.075 
						+ ChassisSpeedRef.rotate_ref*0.075;
	CM4SpeedPID.ref = 160 * CM4SpeedPID.ref;
			
			
	CM4SpeedPID.fdb = CMBRRx.RotateSpeed;

	CM4SpeedPID.Calc(&CM4SpeedPID);
	CMBRIntensity = CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.output;
}

float gap_angle = 0.0;
//底盘旋转控制
void ControlRotate(void)
{	
	CMRotatePID.ref = 0;
	CMRotatePID.fdb = rotate_speed;
	CMRotatePID.Calc(&CMRotatePID);   
	ChassisSpeedRef.rotate_ref = CMRotatePID.output * 13 + ChassisSpeedRef.forward_back_ref * 0.01 + ChassisSpeedRef.left_right_ref * 0.01;
}


//底盘电机CAN信号控制
void setCMMotor()
{
	CanTxMsgTypeDef pData;
	hcan1.pTxMsg = &pData;
	
	hcan1.pTxMsg->StdId = 0x200;
	hcan1.pTxMsg->ExtId = 0;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;
	
	hcan1.pTxMsg->Data[0] = (uint8_t)(CMFLIntensity >> 8);
	hcan1.pTxMsg->Data[1] = (uint8_t)CMFLIntensity;
	hcan1.pTxMsg->Data[2] = (uint8_t)(CMFRIntensity >> 8);
	hcan1.pTxMsg->Data[3] = (uint8_t)CMFRIntensity;
	hcan1.pTxMsg->Data[4] = (uint8_t)(CMBLIntensity >> 8);
	hcan1.pTxMsg->Data[5] = (uint8_t)CMBLIntensity;
	hcan1.pTxMsg->Data[6] = (uint8_t)(CMBRIntensity >> 8);
	hcan1.pTxMsg->Data[7] = (uint8_t)CMBRIntensity;

	if(can1_update == 1 & can1_type == 1)
	{
		//CAN通信前关中断
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK)
		{
			Error_Handler();
		}
		can1_update = 0;
		#ifdef CAN12
		can1_type = 2;
		#endif
		//CAN通信后开中断，防止中断影响CAN信号发送
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
	}
}



//状态机切换
void WorkStateFSM(void)
{
	switch (WorkState)
	{
		case PREPARE_STATE:			//准备模式
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			if(prepare_time < 1000) prepare_time++;
			if(prepare_time == 1000)//开机一秒进入正常模式
			{
				WorkState = TEST_STATE;
				prepare_time = 0;
			}
		}break;
		case TEST_STATE:
		{
			static uint16_t UDcounter = 0;
			static uint16_t AMcounter = 0;
			static double UDRA=0;
			static double AMRA=0;
			if(AMcounter<300)
			{
				if((AMR.RealAngle-AMRA)<0.1&&(AMR.RealAngle-AMRA)>-0.1) AMcounter++;
				else
				{
					AMcounter=0;
					AMRA=AMR.RealAngle;
				}
				if(AMcounter<60) {AMR.TargetAngle+=0.5;AML.TargetAngle=-AMR.TargetAngle;}
			}
			if(UDcounter<300)
			{
				if((UD1.RealAngle-UDRA)<0.1&&(UD1.RealAngle-UDRA)>-0.1) UDcounter++;
				else
				{
					UDcounter=0;
					UDRA=UD1.RealAngle;
				}
				if(UDcounter<60) {UD1.TargetAngle+=1;UD2.TargetAngle=-UD1.TargetAngle;}
			}
			if(UDcounter==300&&AMcounter==300)
			{				
				Open_Gripper();
				
				Delay(300,{
					AMR.RealAngle=230;
					AML.RealAngle=-AMR.RealAngle;
					AMR.TargetAngle=30;
					AML.TargetAngle=-30;
				
					UD1.RealAngle=300;
					UD2.RealAngle=-UD1.RealAngle;
					UD1.TargetAngle=300;
					UD2.TargetAngle=-300;
					
					Close_Gripper();
					
					UDcounter = 0;
					AMcounter = 0;
					UDRA=0;
					AMRA=0;
					WorkState = NORMAL_STATE;
				});
			}
		}break;
		case NORMAL_STATE:			//正常模式
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			if (inputmode == REMOTE_INPUT)
			{
				if (functionmode == GET) WorkState = GET_STATE;
				if (functionmode == HELP) WorkState = HELP_STATE;
			}
		}break;
		case HELP_STATE:				//遥控救援模式
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			if (inputmode == KEY_MOUSE_INPUT) WorkState = PREPARE_STATE;
			if (inputmode == REMOTE_INPUT)
			{
				if (functionmode == NORMAL) WorkState = NORMAL_STATE;
				if (functionmode == GET) WorkState = GET_STATE;
			}
		}break;
		case GET_STATE:					//遥控取弹模式
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			if (inputmode == KEY_MOUSE_INPUT) WorkState = PREPARE_STATE;
			if (inputmode == REMOTE_INPUT)
			{
				if (functionmode == HELP) 	WorkState = HELP_STATE;
				if (functionmode == NORMAL)	WorkState = NORMAL_STATE;
			}
		}break;
		case STOP_STATE:				//紧急停止
		{
			if (inputmode == REMOTE_INPUT || inputmode == KEY_MOUSE_INPUT)
			{
				WorkState = PREPARE_STATE;
				RemoteTaskInit();
			}
		}break;
	}
}

int8_t Test_UD_Direction = 0;
void Slowly_Test_UD()
{
	static uint16_t UDcounter = 0;
	static double UDRA=0;
	if(Test_UD_Direction != 0)
	{
		if(UDcounter<300)
		{
			if((UD1.RealAngle-UDRA)<0.1&&(UD1.RealAngle-UDRA)>-0.1) 
				UDcounter++;
			else
			{
				UDcounter=0;
				UDRA=UD1.RealAngle;
			}
			if(UDcounter<60) {
				UD1.TargetAngle += 1.5 * Test_UD_Direction;
				UD2.TargetAngle =  -UD1.TargetAngle;
			}
		}
		if(UDcounter==300)
		{	
			if(Test_UD_Direction==1)
			{
				UD1.RealAngle   = 300;
				UD2.RealAngle   = -UD1.RealAngle;
				UD1.TargetAngle = UD1.RealAngle - 30;
				UD2.TargetAngle = -UD1.TargetAngle;
			}
			else
			{
				UD1.RealAngle   = -900;
				UD2.RealAngle   = -UD1.RealAngle;
				UD1.TargetAngle = UD1.RealAngle + 50;
				UD2.TargetAngle = -UD1.TargetAngle;
			}				
			UDcounter = 0;
			UDRA=0;
			Test_UD_Direction = 0;
		}
	}
}
//主控制循环
void controlLoop()
{
	WorkStateFSM();
	
	#ifdef SLOW_UPDOWN
		Slowly_Test_UD();
	#endif
	
	if(WorkState != STOP_STATE && WorkState != PREPARE_STATE)
	{
		ControlRotate();
		ControlCMFL();
		ControlCMFR();
		ControlCMBL();
		ControlCMBR();
		
		setCMMotor();
		
		#ifdef CAN12
		for(int i=4;i<8;i++) ControlMotor(can1[i]);
		setCAN12();
		#endif
		#ifdef CAN21
		for(int i=0;i<4;i++) ControlMotor(can2[i]);
		setCAN21();
		#endif
		#ifdef CAN22
		for(int i=4;i<8;i++) ControlMotor(can2[i]);
		setCAN22();
		#endif
	}
}

extern int32_t auto_counter;
//时间中断入口函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)//2ms时钟`
	{
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		
		//主循环在时间中断中启动
		controlLoop();
		
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}
	else if (htim->Instance == htim7.Instance)//ims时钟
	{
		rc_cnt++;
		if(auto_counter > 0) auto_counter--;
		if (rc_update)
		{
			if( (rc_cnt <= 17) && (rc_first_frame == 1))
			{
				RemoteDataProcess(rc_data);				//遥控器数据解算
				HAL_UART_AbortReceive(&RC_UART);
				HAL_UART_Receive_DMA(&RC_UART, rc_data, 18);
				rc_cnt = 0;
			}
			else
			{
				if(rc_first_frame) WorkState = PREPARE_STATE;
				HAL_UART_AbortReceive(&RC_UART);
				HAL_UART_Receive_DMA(&RC_UART, rc_data, 18);
				rc_cnt = 0;
				rc_first_frame = 1;
			}
			rc_update = 0;
		}
	}
	else if (htim->Instance == htim10.Instance)  //10ms，处理上位机数据，优先级不高
	{
		//data.data1 = AMUD1RealAngle;
		//data.data3=0;
		//data.mask=0;
		
		#ifdef DEBUG_MODE
		//zykProcessData();
		//dataCallBack();
		#endif
	}
}

/**
  ******************************************************************************
  * File Name          : RemoteTask.c
  * Description        : 遥控器处理任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

uint8_t rc_data[18];
RC_Ctl_t RC_CtrlData;
InputMode_e inputmode = REMOTE_INPUT; 
FunctionMode_e functionmode = NORMAL;
ChassisSpeed_Ref_t ChassisSpeedRef; 
RemoteSwitch_t g_switch1;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//键盘速度斜坡
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
extern Engineer_State_e EngineerState;
extern int8_t Test_UD_Direction;
//extern tUserData data;
KeyboardMode_e KeyboardMode=NO_CHANGE;
int16_t  GM_ZERO=0;
int16_t  GM_SKEW=0;
int8_t 	move_direction = 1;
int8_t 	init_count=0;

float rotate_speed;

//遥控器控制量初始化
void RemoteTaskInit()
{
	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
	FBSpeedRamp.ResetCounter(&FBSpeedRamp);
	
	//机械臂电机目标物理角度值
	UD1.TargetAngle = 0;
	UD2.TargetAngle = 0;
	GMY.TargetAngle = 0;
	GMP.TargetAngle = 0;
	AMR.TargetAngle = 0;
	AML.TargetAngle = 0;
	/*底盘速度初始化*/
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
	//键鼠初始化
	KeyboardMode=NO_CHANGE;
	//自动取弹初始化
	EngineerState = NOAUTO_STATE;
}

//摇杆控制量解算
int16_t channel0 = 0;
int16_t channel1 = 0;
int16_t channel2 = 0;
int16_t channel3 = 0;
extern int8_t TOTAL_HIGHT;
void Limit_Position()
{
	#ifndef SLOW_UPDOWN 
		VAL_LIMIT(UD1.TargetAngle, UD_BOTTOM, UD_TOP);
	#endif 
	UD2.TargetAngle=-UD1.TargetAngle;
	VAL_LIMIT(AMR.TargetAngle, 0,AM_FRONT);
	AML.TargetAngle=-AMR.TargetAngle;
	VAL_LIMIT(GMP.TargetAngle,-45,45);
	VAL_LIMIT(GM_SKEW,-60,60);
}

void RemoteControlProcess(Remote *rc)
{
	//max=297
	static int8_t flag=0;
	channel0 = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); //右横
	channel1 = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); //右纵
	channel2 = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); //左横
	channel3 = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); //左纵
	if(WorkState == NORMAL_STATE)
	{	
		ChassisSpeedRef.forward_back_ref = move_direction * channel1 * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = move_direction * channel0 * RC_CHASSIS_SPEED_REF/2;
		rotate_speed = -channel2 * RC_ROTATE_SPEED_REF;
		
		if(channel3 > IGNORE_RANGE)	Open_BDOOR();
		else Close_BDOOR();
		
		OnePush((channel3<-IGNORE_RANGE),{
			flag=!flag;
		})
		if(flag) Open_ElectroMagnet();
		else Close_ElectroMagnet();
	}
	if(WorkState == HELP_STATE)
	{
		GMP.TargetAngle += move_direction * channel1 * 0.01;
		GM_SKEW += move_direction * channel0 * 0.01;
		Limit_Position();
		GMY.TargetAngle = GM_ZERO + GM_SKEW;
		if(channel3 > IGNORE_RANGE) {
			AMR.TargetAngle = AM_FRONT;
		}
		else if(channel3 < -IGNORE_RANGE) {
			if(AMR.TargetAngle>AM_BACK) AMR.TargetAngle-=2;
		}
		
		if(channel2 > IGNORE_RANGE) {
			Open_Gripper();
		}
		else if(channel2 < -IGNORE_RANGE) {
			Close_Gripper();
		}
	}
	if(WorkState == GET_STATE)
	{
		rotate_speed = 0;
		
		ChassisSpeedRef.forward_back_ref = channel1 * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channel0 * RC_CHASSIS_SPEED_REF/2;
		CML.TargetAngle=ChassisSpeedRef.forward_back_ref*5;
		CMR.TargetAngle=-ChassisSpeedRef.forward_back_ref*5;
		
		if(channel2 > IGNORE_RANGE) {
			AMR.TargetAngle+=AMFB_ANGLE_STEP;
		}
		else if(channel2 < -IGNORE_RANGE) {
			AMR.TargetAngle-=AMFB_ANGLE_STEP;
		}
		
		if(channel3 > IGNORE_RANGE) {
			UD1.TargetAngle+=AMUD_ANGLE_STEP;
		}
		else if(channel3 < -IGNORE_RANGE) {
			UD1.TargetAngle-=AMUD_ANGLE_STEP;
		}
		RefreshAnologRead();
		//Chassis_Choose(1,1);
	}
	Limit_Position();
}


uint16_t MK_FORWORD_BACK_SPEED 	= NORMAL_FORWARD_BACK_SPEED;
uint16_t MK_LEFT_RIGHT_SPEED  	= NORMAL_LEFT_RIGHT_SPEED;

//键鼠控制量解算
void KeyboardModeFSM(Key *key)
{
			//键盘状态切换 
			if((key->v & 0x30) == 0x30)//Shift_Ctrl
			{
				MK_FORWORD_BACK_SPEED=  LOW_FORWARD_BACK_SPEED;
				MK_LEFT_RIGHT_SPEED = LOW_LEFT_RIGHT_SPEED;
				KeyboardMode=SHIFT_CTRL;
			}
			else if(key->v & KEY_SHIFT)//Shift
			{
				MK_FORWORD_BACK_SPEED=  HIGH_FORWARD_BACK_SPEED;
				MK_LEFT_RIGHT_SPEED = HIGH_LEFT_RIGHT_SPEED;
				KeyboardMode=SHIFT;
			}
			else if(key->v & KEY_CTRL)//Ctrl
			{
				MK_FORWORD_BACK_SPEED=  LOW_FORWARD_BACK_SPEED;
				MK_LEFT_RIGHT_SPEED = LOW_LEFT_RIGHT_SPEED;
				KeyboardMode=CTRL;
			}
			else
			{
				MK_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
				MK_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
				KeyboardMode=NO_CHANGE;
			}
			
			if(EngineerState != NOAUTO_STATE) 
			{
				MK_FORWORD_BACK_SPEED=  LOW_FORWARD_BACK_SPEED;
				MK_LEFT_RIGHT_SPEED = LOW_LEFT_RIGHT_SPEED;
			}
}

void MouseKeyControlProcess(Mouse *mouse, Key *key)
{
	static uint8_t 	GiveBigBullet_State = 0;
	//static uint8_t 	GiveSmallBullet_State = 0;
	static uint8_t 	AUTO_UPDOWN_STATE = 0;
	static uint8_t 	HELP_OTHERS_STATE = 0;
	
	if(WorkState != STOP_STATE && WorkState != PREPARE_STATE)
	{
		VAL_LIMIT(mouse->x, -150, 150); 
		VAL_LIMIT(mouse->y, -150, 150); 
		
		if((mouse->press_l ^ HELP_OTHERS_STATE) == 1) {
			GMP.TargetAngle += move_direction * mouse->y * 0.24;
			GM_SKEW -= mouse->x * 0.24;
			rotate_speed = 0;
		}
		else
		{
			rotate_speed = - mouse->x * MK_ROTATE_SPEED_REF;
		}
		OnePush(mouse->press_r,{
			move_direction=-move_direction;
			if(GM_ZERO==0) GM_ZERO=180;
			else GM_ZERO=0;
			GM_SKEW=0;
			GMP.TargetAngle=0;
		})
		GMY.TargetAngle = GM_ZERO + GM_SKEW;

		KeyboardModeFSM(key);
		RefreshAnologRead();
		
		switch (KeyboardMode)
		{
			case SHIFT_CTRL:	//细节微调与自动控制按钮
			{//GM Yaw Adjust
				if(key->v & KEY_R)//r
				{
					GMP.TargetAngle = 0;
					GMP.RealAngle = 0;
					GMY.TargetAngle = 0;
					GMY.RealAngle = 0;
				}
				break;
			}
			case CTRL:				//手动机械臂调整，慢速底盘
			{//AM Movement Process
				if(key->v & KEY_Z)	//z 下降
				{
					#ifdef SLOW_UPDOWN
						Test_UD_Direction = -1;
					#else
						UD1.TargetAngle = UD_BOTTOM;
					#endif
				};
				if(key->v & KEY_X)	//x 上升
				{
					#ifdef SLOW_UPDOWN
						Test_UD_Direction = 1;
					#else
						UD1.TargetAngle = UD_TOP;
					#endif
				};
				if(key->v & KEY_C)  //c 供弹
				{
					GiveBigBullet_State=1;
				}
				
				if(key->v & KEY_Q)
				{
					Open_ElectroMagnet();
					HELP_OTHERS_STATE = 1;
				}
				if(key->v & KEY_E)
				{
					Close_ElectroMagnet();
					HELP_OTHERS_STATE = 0;
				}
				
				if(key->v & KEY_G)
				{
					AUTO_UPDOWN_STATE = 1;
				}
				
				if(key->v & KEY_B)
				{
					AUTO_UPDOWN_STATE = 0;
				}
				
				//注意这里不用break！
			}
			case SHIFT:						//高速底盘
			case NO_CHANGE:					//正常底盘
			{//CM Movement Process
				if(key->v & KEY_W)  		//key: w
					ChassisSpeedRef.forward_back_ref =  move_direction * MK_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
				else if(key->v & KEY_S) 	//key: s
					ChassisSpeedRef.forward_back_ref = -move_direction * MK_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
				else
				{
					ChassisSpeedRef.forward_back_ref = 0;
					FBSpeedRamp.ResetCounter(&FBSpeedRamp);
				}
				if(key->v & KEY_D)  		//key: d
					ChassisSpeedRef.left_right_ref =  move_direction * MK_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
				else if(key->v & KEY_A) 	//key: a
					ChassisSpeedRef.left_right_ref = -move_direction * MK_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
				else
				{
					ChassisSpeedRef.left_right_ref = 0;
					LRSpeedRamp.ResetCounter(&LRSpeedRamp);
				}

				if(key->v & KEY_R)		//r搜索置位
				{
					AMR.TargetAngle = AM_FRONT;
					//if(EngineerState == NOAUTO_STATE) EngineerState = LEVEL_SHIFT;
				}
				else if(key->v & KEY_F)//f自动获取
				{
					if(EngineerState == NOAUTO_STATE) EngineerState = ARM_STRETCH;
				}
				else if(key->v & KEY_G)//g取消
					EngineerState = ERROR_HANDLE;
				break;
			}
		}
		
		if(GiveBigBullet_State)//2s自动关
		{
			UD1.TargetAngle = 0;
			Delay(20,{
				Open_BDOOR();
				Delay(10,{
					Close_BDOOR();
					GiveBigBullet_State = 0;
					#ifdef SLOW_UPDOWN
						Test_UD_Direction = 1;
					#else
						UD1.TargetAngle = UD_TOP;
					#endif
				})
			})
		}
		
		if(AUTO_UPDOWN_STATE) HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_RESET);
		if(HELP_OTHERS_STATE) HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_RESET);
		
		AutoGet((key->v & KEY_CTRL)); 
		Chassis_Choose(AUTO_UPDOWN_STATE,(key->v&KEY_Q));
		Limit_Position();

		/*裁判系统离线时的功率限制方式*/
//		if(JUDGE_State == OFFLINE)
//		{
//			if(abs(ChassisSpeedRef.forward_back_ref) + abs(ChassisSpeedRef.left_right_ref) > 500)
//			{
//				if(ChassisSpeedRef.forward_back_ref > 325)
//				{
//				ChassisSpeedRef.forward_back_ref =  325 +  (ChassisSpeedRef.forward_back_ref - 325) * 0.15f;
//				}
//				else if(ChassisSpeedRef.forward_back_ref < -325)
//				{
//				ChassisSpeedRef.forward_back_ref =  -325 +  (ChassisSpeedRef.forward_back_ref + 325) * 0.15f;
//				}
//				if(ChassisSpeedRef.left_right_ref > 300)
//				{
//				ChassisSpeedRef.left_right_ref =  300 +  (ChassisSpeedRef.left_right_ref - 300) * 0.15f;
//				}
//				else if(ChassisSpeedRef.left_right_ref < -300)
//				{
//				ChassisSpeedRef.left_right_ref =  -300 +  (ChassisSpeedRef.left_right_ref + 300) * 0.15f;
//				}
//			}

//			if ((mouse->x < -2.6) || (mouse->x > 2.6))
//			{
//				if(abs(ChassisSpeedRef.forward_back_ref) + abs(ChassisSpeedRef.left_right_ref) > 400)
//				{
//					if(ChassisSpeedRef.forward_back_ref > 250){
//					 ChassisSpeedRef.forward_back_ref =  250 +  (ChassisSpeedRef.forward_back_ref - 250) * 0.15f;
//					}
//					else if(ChassisSpeedRef.forward_back_ref < -250)
//					{
//						ChassisSpeedRef.forward_back_ref =  -250 +  (ChassisSpeedRef.forward_back_ref + 250) * 0.15f;
//					}
//					if(ChassisSpeedRef.left_right_ref > 250)
//					{
//					 ChassisSpeedRef.left_right_ref =  250 +  (ChassisSpeedRef.left_right_ref - 250) * 0.15f;
//					}
//					else if(ChassisSpeedRef.left_right_ref < -250)
//					{
//						ChassisSpeedRef.left_right_ref =  -250 +  (ChassisSpeedRef.left_right_ref + 250) * 0.15f;
//					}
//				}
//			}
//		}
	}
}


/*拨杆数据处理*/   
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
	static uint32_t switch_cnt = 0;

	sw->switch_value_raw = val;
	sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;

	//value1 value2的值其实是一样的
	//value1高4位始终为0
	sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
	(sw->switch_value_buf[sw->buf_index]);

	sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

	sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;	

	//如果两次数据一样，即没有更新数据，拨杆不动
	if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
	{
		switch_cnt++;	
	}
	else
	{
		switch_cnt = 0;
	}
	//如果拨杆维持了一定时间，即连续来了40帧一样的数据，则把拨杆数据写入switch_long_value
	if(switch_cnt >= 40)
	{
		sw->switch_long_value = sw->switch_value_buf[sw->buf_index]; 	
	}
	//指向下一个缓冲区
	sw->buf_last_index = sw->buf_index;
	sw->buf_index++;		
	if(sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
	{
		sw->buf_index = 0;	
	}			
}


//遥控器数据解算
void RemoteDataProcess(uint8_t *pData)
{
	HAL_IWDG_Refresh(&hiwdg);
	if(pData == NULL)
	{
			return;
	}
	//遥控器 11*4 + 2*2 = 48，需要 6 Bytes
	//16位，只看低11位
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
											 ((int16_t)pData[4] << 10)) & 0x07FF;
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	
	//16位，只看最低两位
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

	//鼠标需要 8 Bytes
	RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

	RC_CtrlData.mouse.press_l = pData[12];
	RC_CtrlData.mouse.press_r = pData[13];
	
	//键盘需要 2 Bytes = 16 bits ，每一位对应一个键
	RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);

	//输入状态设置
	if(RC_CtrlData.rc.s2 == 1) inputmode = REMOTE_INPUT;
	else if(RC_CtrlData.rc.s2 == 3) inputmode = KEY_MOUSE_INPUT; 
	else inputmode = STOP; 
	
	//功能状态设置
	if(RC_CtrlData.rc.s1 == 1) functionmode = NORMAL; 
	else if(RC_CtrlData.rc.s1 == 3) functionmode = HELP; 
	else functionmode = GET; 
	
	/*左上角拨杆状态（RC_CtrlData.rc.s1）获取*/	//用于遥控器发射控制
	GetRemoteSwitchAction(&g_switch1, RC_CtrlData.rc.s1);
	
	switch(inputmode)
	{
		case REMOTE_INPUT:               
		{
			if(WorkState != STOP_STATE && WorkState != PREPARE_STATE && WorkState != TEST_STATE)
			{ 
				RemoteControlProcess(&(RC_CtrlData.rc));
			}
		}break;
		case KEY_MOUSE_INPUT:              
		{
			if(WorkState != STOP_STATE && WorkState != PREPARE_STATE && WorkState != TEST_STATE)
			{ 
				MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key);
			}
		}break;
		case STOP:               
		{
			 
		}break;
	}
}

//初始化遥控器串口DMA接收
void InitRemoteControl(){
	if(HAL_UART_Receive_DMA(&RC_UART, rc_data, 18) != HAL_OK){
			Error_Handler();
	} 
	RemoteTaskInit();
}

//遥控器串口中断入口函数，从此处开始执行
uint8_t rc_first_frame = 0;
uint8_t rc_update = 0;
uint8_t rc_cnt = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &RC_UART){
		rc_update = 1;
	}
	else if(UartHandle == &MANIFOLD_UART)
	{
		//manifoldUartRxCpltCallback();  //妙算信号数据解算
		#ifdef DEBUG_MODE
		ctrlUartRxCpltCallback();
		#endif
	}
	else if(UartHandle == &JUDGE_UART)
	{
		judgeUartRxCpltCallback();  //裁判系统数据解算
	}
}   

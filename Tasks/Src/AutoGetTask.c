/**
  ******************************************************************************
  * File Name          : AutoGetTask.c
  * Description        : 自动取弹控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
Distance_Couple_t distance_couple;
Engineer_State_e EngineerState = NOAUTO_STATE;
int32_t ad0=0,ad1=0,ad2=0,ad3=0,ad4=0;
extern uint32_t ADC_Value[];

void RefreshAnologRead()
{
		for(uint16_t i=0;i<100;i++)
		{
			if(i%5==0)ad0+=ADC_Value[i];
			if(i%5==1)ad1+=ADC_Value[i];
			if(i%5==2)ad2+=ADC_Value[i];
			if(i%5==3)ad3+=ADC_Value[i];
			if(i%5==4)ad4+=ADC_Value[i];
		}
		//     average  bias
		ad0 = ad0 / 20 - 0;	//front
		ad1 = ad1 / 20 - 0;	//rightout
		ad2 = ad2 / 20 - 0;	//rightin
		ad3 = ad3 / 20 - 0;	//leftout
		ad4 = ad4 / 20 - 0;	//leftin
		
		distance_couple.front.vol_ref    = ad0;
		distance_couple.leftin.vol_ref   = ad1;
		distance_couple.leftout.vol_ref  = ad4;
		distance_couple.rightin.vol_ref  = ad3;
		distance_couple.rightout.vol_ref = ad2;
		
		FLAG_SET(distance_couple.front.vol_ref,		distance_couple.front.flag	 );
		FLAG_SET(distance_couple.leftin.vol_ref,	distance_couple.leftin.flag	 );
		FLAG_SET(distance_couple.leftout.vol_ref,	distance_couple.leftout.flag );
		FLAG_SET(distance_couple.rightin.vol_ref,	distance_couple.rightin.flag );
		FLAG_SET(distance_couple.rightout.vol_ref,distance_couple.rightout.flag);
		
		distance_couple.move_flags = distance_couple.leftout.flag  * 8 +
																 distance_couple.leftin.flag   * 4 +
																 distance_couple.rightin.flag  * 2 +
																 distance_couple.rightout.flag * 1;
}


uint8_t Auto_StartTest(char signal)
{
	if(distance_couple.move_flags == 0x0) return 0;
	switch(signal)
	{
		case 'l':if(distance_couple.move_flags == 0x1) return 0;break;
		case 'r':if(distance_couple.move_flags == 0x8) return 0;break;
		default:break;
	}
	return 1;
}

int8_t Auto_ShiftTest(char signal)
{
	if(Auto_StartTest(signal))
	{
		if(distance_couple.move_flags == 0x6 ||
				distance_couple.move_flags == 0x4)
		return 1;
		else return 0;
	}
	else return -1;
}

uint32_t auto_counter=0;

#define AUTODELAY(TIM,execution)\
{\
	static uint8_t flag = 1;\
	if(flag)\
	{\
		flag = 0;\
		auto_counter = TIM;\
	}\
	if(!auto_counter)\
	{\
		flag = 1;\
		execution\
	}\
}

void AutoGet(char signal,uint8_t flag)
{
//	static int8_t auto_flag;
	static uint8_t step = 0;
	static uint8_t inTimes=0;
	RefreshAnologRead();
	switch(EngineerState)
	{
		case START_TEST:
		{
			AUTODELAY(1000,{
				EngineerState = LEVEL_SHIFT;
			});
		}
		case LEVEL_SHIFT:
		{
			/*ChassisSpeedRef.forward_back_ref = -5;
			auto_flag = Auto_ShiftTest(signal);
			global_catch = auto_flag;
			switch(auto_flag)
			{
				case 1: 
					auto_counter = 500;
					EngineerState = NOAUTO_STATE;
					//EngineerState = ARM_STRETCH;
					//EngineerState = LEVEL_SHIFT;
					break;
				case 0:
					if(signal == 'l')
						ChassisSpeedRef.left_right_ref = 35;
					else ChassisSpeedRef.left_right_ref = -35;
					break;
				case -1:
					EngineerState = ERROR_HANDLE;
					break;
			}*/
			if(inTimes==0) {
				AMUDAngleTarget+=300;
				inTimes++;
			}
			if(inTimes>0)
			{	
				if((distance_couple.move_flags & 6)==0)
				{
					inTimes++;
					if(inTimes==10)
					{	
						HAL_GPIO_WritePin(M_VAVLE_FB_IO, GPIO_PIN_SET);	//前伸
						AMUDAngleTarget-=300;
						EngineerState = NOAUTO_STATE;
						inTimes=0;
					}
				}
				else{
				inTimes=1;
				}
			}
			break;
		}
		case ARM_STRETCH:
		{
			if(!auto_counter)
			{
				auto_counter=1000;
				if(step == 7) {
					step=0;
					EngineerState = NOAUTO_STATE;
				}
				switch(step)
				{
					/*case 0: AMUDAngleTarget -= 300;	HAL_GPIO_WritePin(M_VAVLE_OC_IO, GPIO_PIN_RESET);	break;	//释放
					case 1: auto_counter=3000;			ChassisSpeedRef.forward_back_ref = 40;						break;	//自行后退
					case 2: auto_counter=3000;			ChassisSpeedRef.forward_back_ref = -40;										//自行前进
																					HAL_GPIO_WritePin(M_VAVLE_FB_IO, GPIO_PIN_SET);		break;	//前伸*/
					case 1:													HAL_GPIO_WritePin(M_VAVLE_OC_IO, GPIO_PIN_SET);		break;	//抓紧
					case 2:	if(AMUDAngleTarget > 900)																													//抬高
										{auto_counter=1500;		AMUDAngleTarget = 1400;}
									else
										{auto_counter=4000;		AMUDAngleTarget = 1400;}
									break;	
					case 3: auto_counter=500;				HAL_GPIO_WritePin(M_VAVLE_FB_IO, GPIO_PIN_RESET);	break;	//回缩
					case 4: 												HAL_GPIO_WritePin(M_VAVLE_FB_IO, GPIO_PIN_SET);		break;	//前伸
					case 5: 												HAL_GPIO_WritePin(M_VAVLE_OC_IO, GPIO_PIN_RESET);	break;	//释放
					case 6: 												HAL_GPIO_WritePin(M_VAVLE_FB_IO, GPIO_PIN_RESET);	break;	//回缩
				}
				if(step != 3 || !flag) step++;
			}
			break;
		}	
		case ERROR_HANDLE:
		{
			AMUDAngleTarget = 10;																//降低
			HAL_GPIO_WritePin(M_VAVLE_FB_IO, GPIO_PIN_RESET);		//回缩
			HAL_GPIO_WritePin(M_VAVLE_OC_IO, GPIO_PIN_RESET);		//释放
			step = 0;
			inTimes = 0;
			EngineerState = NOAUTO_STATE;
		}
		case NOAUTO_STATE:
			break;
	}
	Limit_Position();
}

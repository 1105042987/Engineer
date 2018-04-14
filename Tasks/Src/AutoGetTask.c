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
	global_catch++;
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
		distance_couple.leftin.vol_ref   = ad4;
		distance_couple.leftout.vol_ref  = ad3;
		distance_couple.rightin.vol_ref  = ad2;
		distance_couple.rightout.vol_ref = ad1;
		
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
		if(distance_couple.move_flags == 0x3 ||
				distance_couple.move_flags == 0x4 ||
				distance_couple.move_flags == 0x6)
		return 1;
		else return 0;
	}
	else return -1;
}


void AutoGet(char signal)
{
	static int8_t auto_flag;
	RefreshAnologRead();
	switch(EngineerState)
	{
		case START_TEST:
		{
			if(Auto_StartTest(signal)) EngineerState = LEVEL_SHIFT;
			else EngineerState = ERROR_HANDLE;
			break;
		}
		case LEVEL_SHIFT:
		{
			ChassisSpeedRef.forward_back_ref += 30;
			auto_flag = Auto_ShiftTest(signal);
			switch(auto_flag)
			{
				case 1: 
					EngineerState = HEIGHT_ADJUST;
					break;
				case 0:
					if(signal == 'l')
						ChassisSpeedRef.left_right_ref -= 30;
					else ChassisSpeedRef.left_right_ref += 30;
					break;
				case -1:
					EngineerState = NOAUTO_STATE;
					break;
			}
			break;
		}
		case HEIGHT_ADJUST:
		{
			ChassisSpeedRef.forward_back_ref += 30;
			
			if(distance_couple.move_flags == 0x0)
			{
				if(auto_flag > 50)
				{
					EngineerState = ARM_STRETCH;
					auto_flag = 0;
				}
				else {
					auto_flag++;
					AMUD1AngleTarget += AMUD_ANGLE_STEP;
					AMUD2AngleTarget += AMUD_ANGLE_STEP;
				}
			}
			else {
				AMUD1AngleTarget += AMUD_ANGLE_STEP;
				AMUD2AngleTarget += AMUD_ANGLE_STEP;
				auto_flag = 0;
			}
			
			if(AMUD1AngleTarget < -400) //越界 AMANGLE_STEP是负值
				EngineerState = ERROR_HANDLE;
			
			break;
		}
		case ARM_STRETCH:
		{
			ChassisSpeedRef.forward_back_ref += 30;
			
			if(distance_couple.front.flag) EngineerState = BULLET_GET;
			else AMFBAngleTarget += AMFB_ANGLE_STEP;
			
			if(AMFBAngleTarget < -400) //越界
				EngineerState = ERROR_HANDLE;
			
			break;
		}	
		case BULLET_GET:
		{
			
			
			
			
			break;
		}
		case ERROR_HANDLE:
		{
			if(AMFBAngleTarget < -10 || AMUD1AngleTarget < -10) 
			{
				AMFBAngleTarget -= AMFB_ANGLE_STEP;
				AMUD1AngleTarget -= AMUD_ANGLE_STEP;
				AMUD2AngleTarget -= AMUD_ANGLE_STEP;
			}
			else{
				
			}
			break;
		}
		case NOAUTO_STATE:
			break;
	}
	Limit_Position();
}

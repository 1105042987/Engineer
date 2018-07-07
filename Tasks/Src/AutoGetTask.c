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
int32_t ad1=0,ad2=0,ad3=0,ad4=0,ad5=0,ad6=0,ad7=0,ad8=0;
int8_t TOTAL_HIGHT=0;
extern uint32_t ADC_Value[];

void RefreshAnologRead()
{
		for(uint16_t i=0;i<160;i++)
		{
			if(i%8==0)ad1+=ADC_Value[i];
			if(i%8==1)ad2+=ADC_Value[i];
			if(i%8==2)ad3+=ADC_Value[i];
			if(i%8==3)ad4+=ADC_Value[i];
			if(i%8==4)ad5+=ADC_Value[i];
			if(i%8==5)ad6+=ADC_Value[i];
			if(i%8==6)ad7+=ADC_Value[i];
			if(i%8==7)ad8+=ADC_Value[i];
		}
		//     average  bias
		ad1 = ad1 / 21 - 0;	//rightout
		ad2 = ad2 / 21 - 0;	//rightin
		ad3 = ad3 / 21 - 0;	//leftout
		ad4 = ad4 / 21 - 0;	//leftin
		ad5 = ad5 / 21 - 0;	//rightout
		ad6 = ad6 / 21 - 0;	//rightin
		ad7 = ad7 / 21 - 0;	//leftout
		ad8 = ad8 / 21 - 0;	//leftin
		
		distance_couple.frontf.val_ref	= ad8;
		distance_couple.frontr.val_ref	= ad7;
		distance_couple.frontl.val_ref	= ad6;
		distance_couple.backb.val_ref	= ad5;
		distance_couple.backr.val_ref	= ad4;
		distance_couple.backl.val_ref	= ad3;
		distance_couple.left.val_ref	= ad2;
		distance_couple.right.val_ref	= ad1;
		
		
		FLAG_SET(distance_couple.frontf);
		FLAG_SET(distance_couple.frontr);
		FLAG_SET(distance_couple.frontl);
		FLAG_SET(distance_couple.backb);
		FLAG_SET(distance_couple.backr);
		FLAG_SET(distance_couple.backl);
		FLAG_SET(distance_couple.left);
		FLAG_SET(distance_couple.right);
		
		distance_couple.move_flags = ((distance_couple.left.flag)								*512) +
									 ((distance_couple.right.flag) 								*256) +
									 ((distance_couple.frontl.flag) 							*128) +
									 ((distance_couple.frontr.flag) 							* 64) +
									 ((distance_couple.backl.flag) 								* 32) +
									 ((distance_couple.backr.flag) 								* 16) +
									 ((distance_couple.frontr.flag&distance_couple.frontl.flag) *  8) +
									 ((distance_couple.frontf.flag) 							*  4) +
									 ((distance_couple.backb.flag)								*  2) +
									 ((distance_couple.backr.flag&distance_couple.backl.flag)	*  1);
		//低八位：低四位 基础判定，高四位 精细判定（转向）
		//高八位：低四位 抓取判定
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

void AutoGet(uint8_t flag)
{
//	static int8_t auto_flag;
	static uint8_t step = 0;
	static uint8_t inTimes=0;
	switch(EngineerState)
	{
		case START_TEST:
		{
			AUTODELAY(100,{
				EngineerState = LEVEL_SHIFT;
			});
		}
		case LEVEL_SHIFT:
		{
			if(inTimes==0) {
				UD1.TargetAngle+=300;
				UD2.TargetAngle+=300;
				inTimes++;
			}
			if(inTimes>0){	
				if((distance_couple.move_flags>>8)==0){
					inTimes++;
					if(inTimes==10){	
						HAL_GPIO_WritePin(M_VAVLE_FB_IO, GPIO_PIN_SET);	//前伸
						UD1.TargetAngle-=300;
						UD2.TargetAngle-=300;
						EngineerState = NOAUTO_STATE;
						inTimes=0;
					}
				}
				else{ inTimes=1; }
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
					case 1:	HAL_GPIO_WritePin(M_VAVLE_OC_IO, GPIO_PIN_SET);								break;	//抓紧
					case 2:	if(UD1.TargetAngle > 900)																													//抬高
							{auto_counter=1500;		UD1.TargetAngle = 1400;UD2.TargetAngle = 1400;}
							else
							{auto_counter=4000;		UD1.TargetAngle = 1400;UD2.TargetAngle = 1400;}
							break;	
					case 3: auto_counter=500;		HAL_GPIO_WritePin(M_VAVLE_FB_IO, GPIO_PIN_RESET);	break;	//回缩
					case 4: 						HAL_GPIO_WritePin(M_VAVLE_FB_IO, GPIO_PIN_SET);		break;	//前伸
					case 5: 						HAL_GPIO_WritePin(M_VAVLE_OC_IO, GPIO_PIN_RESET);	break;	//释放
					case 6: 						HAL_GPIO_WritePin(M_VAVLE_FB_IO, GPIO_PIN_RESET);	break;	//回缩
				}
				if(step != 3 || !flag) step++;
			}
			break;
		}	
		case ERROR_HANDLE:
		{
			UD1.TargetAngle = 10;									//降低
			HAL_GPIO_WritePin(M_VAVLE_FB_IO, GPIO_PIN_RESET);		//回缩
			HAL_GPIO_WritePin(M_VAVLE_OC_IO, GPIO_PIN_RESET);		//释放
			step = 0;
			inTimes = 0;
			EngineerState = NOAUTO_STATE;
		}
		case NOAUTO_STATE:	break;
	}
	Limit_Position();
}

void Chassis_Choose()
{
	static uint8_t signal1=0;
	static uint8_t signal2=0;
	if(UD1.RealAngle<-600)
	{//small chassis
		signal1=0;
		switch(distance_couple.move_flags&0x000f)
		{
			case 9:
				if (ChassisSpeedRef.forward_back_ref < 0) {
					ChassisSpeedRef.forward_back_ref=0;
					CML.TargetAngle=0;
					CMR.TargetAngle=0;
					if(signal2==0){
						UD1.TargetAngle+=STEPHIGHT;
						UD2.TargetAngle-=STEPHIGHT;
						signal2=1;
					}
						
				}
				break;
			case 15:
				if(ChassisSpeedRef.forward_back_ref > 0){
					ChassisSpeedRef.forward_back_ref=0;
					CML.TargetAngle=0;
					CMR.TargetAngle=0;
					if(signal2==0){
						UD1.TargetAngle+=STEPHIGHT;
						UD2.TargetAngle-=STEPHIGHT;
						signal2=1;
					}
				}
				break;
			default:break;
		}
	}
	else if(UD1.RealAngle>-50)
	{// chassis
		signal2=0;
		switch(distance_couple.move_flags&0xf)
		{
			case 6:
				if (ChassisSpeedRef.forward_back_ref < 0) {
					//ChassisSpeedRef.forward_back_ref=0;
					CML.TargetAngle=0;
					CMR.TargetAngle=0;
					switch((distance_couple.move_flags>>4)&0x3)
					{
						case 1://右转
							rotate_speed=ChassisSpeedRef.forward_back_ref/4;
							ChassisSpeedRef.forward_back_ref=5;
							break;
						case 2://左转
							rotate_speed=ChassisSpeedRef.forward_back_ref/-4;
							ChassisSpeedRef.forward_back_ref=5;
							break;
						case 0:
							ChassisSpeedRef.forward_back_ref=0;
							if(signal1==0){
								UD1.TargetAngle-=STEPHIGHT;
								UD2.TargetAngle+=STEPHIGHT;
								signal1=1;
								TOTAL_HIGHT--;
							}break;
					}	
				}
				break;
			case 15:
				if(ChassisSpeedRef.forward_back_ref>0){
					ChassisSpeedRef.forward_back_ref=0;
					CML.TargetAngle=0;
					CMR.TargetAngle=0;
					if(signal1==0){
						UD1.TargetAngle-=STEPHIGHT;
						UD2.TargetAngle+=STEPHIGHT;
						signal1=1;
						TOTAL_HIGHT++;
					}
				}
				break;
			default:break;
		}
	}
	else{
		ChassisSpeedRef.forward_back_ref=0;
		CML.TargetAngle=0;
		CMR.TargetAngle=0;
	}
}

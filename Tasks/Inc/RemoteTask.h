/**
  ******************************************************************************
  * File Name          : RemoteTask.h
  * Description        : 遥控器处理任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __REMOTETASK_H
#define __REMOTETASK_H

#include "includes.h"

#define RC_UART huart1
#define MANIFOLD_UART huart3
#define CTRL_UART huart3
#define JUDGE_UART huart6



//解算数据区
#define REMOTE_CONTROLLER_STICK_OFFSET  1024u

#define REMOTE_SWITCH_VALUE_UP         	0x01u  
#define REMOTE_SWITCH_VALUE_DOWN				0x02u
#define REMOTE_SWITCH_VALUE_CENTRAL			0x03u

#define REMOTE_SWITCH_VALUE_BUF_DEEP   16u

//遥控常量数据区
#define RC_CHASSIS_SPEED_REF    0.60f
#define RC_ROTATE_SPEED_REF 		0.05f

#define IGNORE_RANGE 200

//键鼠常量数据区
//Bit0-----W			0x1
//Bit1-----S			0x2
//Bit2-----A			0x4
//Bit3-----D			0x8
//Bit4-----Shift	0x10
//Bit5-----Ctrl		0x20
//Bit6-----Q			0x40
//Bit7-----E			0x80
//Bit8-----R			0x100
//Bit9-----F			0x200
//Bit10-----G			0x400
//Bit11-----Z			0x800
//Bit12-----X			0x1000
//Bit13-----C			0x2000
//Bit14-----V			0x4000
//Bit15-----B			0x8000
#define NORMAL_FORWARD_BACK_SPEED 	400
#define NORMAL_LEFT_RIGHT_SPEED  		400
#define HIGH_FORWARD_BACK_SPEED 		600
#define HIGH_LEFT_RIGHT_SPEED   		600
#define LOW_FORWARD_BACK_SPEED 			200
#define LOW_LEFT_RIGHT_SPEED   			200

#define MOUSE_LR_RAMP_TICK_COUNT		50
#define MOUSR_FB_RAMP_TICK_COUNT		60

#define MK_ROTATE_SPEED_REF 				0.80f




#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote;

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	

typedef	__packed struct
{
	uint16_t v;
}Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;

typedef enum
{
	REMOTE_INPUT = 1,
	KEY_MOUSE_INPUT = 3,
	STOP = 2,
}InputMode_e;

typedef enum
{
	NORMAL = 1,
	HELP = 3,
	GET = 2,
}FunctionMode_e;

typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

typedef __packed struct
{
	 uint8_t switch_value_raw;      // the current switch value
	 uint8_t switch_value1;				  //  last value << 2 | value
	 uint8_t switch_value2;				  //
	 uint8_t switch_long_value; 		//keep still if no switching
	 uint8_t switch_value_buf[REMOTE_SWITCH_VALUE_BUF_DEEP]; 
	 uint8_t buf_index;
	 uint8_t buf_last_index;
	 uint8_t buf_end_index;
}RemoteSwitch_t;

typedef enum{
	SHIFT,
	CTRL,
	SHIFT_CTRL,
	NO_CHANGE,
} KeyboardMode_e;


extern ChassisSpeed_Ref_t ChassisSpeedRef; 
extern InputMode_e inputmode;
extern FunctionMode_e functionmode;
extern float rotate_speed;
extern double AMUD1AngleTarget;
extern double AMUD2AngleTarget;
extern double AMFBAngleTarget;
extern double GMYAWAngleTarget;
extern double GMPITCHAngleTarget;

extern uint8_t rc_data[18];
extern uint8_t rc_first_frame;
extern uint8_t rc_update;
extern uint8_t rc_cnt;

void RemoteDataProcess(uint8_t *pData);
void InitRemoteControl(void);
void RemoteTaskInit(void);
void Limit_Position(void);

#endif /*__ REMOTETASK_H */

/**
  ******************************************************************************
  * File Name          : CANTask.c
  * Description        : CAN通信任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

#define CanRxGetU16(canRxMsg, num) (((uint16_t)canRxMsg.Data[num * 2] << 8) | (uint16_t)canRxMsg.Data[num * 2 + 1])
uint8_t isRcan1Started = 0, isRcan2Started = 0;
CanRxMsgTypeDef Can1RxMsg,Can2RxMsg;
ESCC6x0RxMsg_t CMFLRx,CMBLRx,CMFRRx,CMBRRx;
uint8_t can1_update = 1;
uint8_t can1_type = 1;
uint8_t can2_update = 1;
uint8_t can2_type = 1;

/********************CAN发送*****************************/
//CAN数据标记发送，保证发送资源正常
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if(hcan == &hcan1){
		can1_update = 1;
	}
	else if(hcan == &hcan2)
	{
		can2_update = 1;
	}
}

/********************CAN******************************/
void InitCanReception()
{
	//http://www.eeworld.com.cn/mcu/article_2016122732674_3.html
	hcan1.pRxMsg = &Can1RxMsg;
	/*##-- Configure the CAN1 Filter ###########################################*/
	CAN_FilterConfTypeDef  sFilterConfig;
	sFilterConfig.FilterNumber = 0;//14 - 27//14
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 14;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	if(HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK){
		Error_Handler(); 
	}
	isRcan1Started = 1;
	
	hcan2.pRxMsg = &Can2RxMsg;
	/*##-- Configure the CAN2 Filter ###########################################*/
	CAN_FilterConfTypeDef sFilterConfig2;
	sFilterConfig2.FilterNumber = 14;//14 - 27//14
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig2.FilterIdHigh = 0x0000;
	sFilterConfig2.FilterIdLow = 0x0000;
	sFilterConfig2.FilterMaskIdHigh = 0x0000;
	sFilterConfig2.FilterMaskIdLow = 0x0000;
	sFilterConfig2.FilterFIFOAssignment = 0;
	sFilterConfig2.FilterActivation = ENABLE;
	sFilterConfig2.BankNumber = 14;
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig2);
	if(HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK){
		Error_Handler(); 
	}
	isRcan2Started = 1;
}
 
//CAN接收中断入口函数
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	if(hcan == &hcan1)	//CAN1数据
	{
		int num=Can1RxMsg.StdId-0x201;
		if(num>3&&num<8)//4,5,6,7
		{
			if(can1[num]!=0)
			{
				can1[num]->RxMsg.angle		= CanRxGetU16(Can1RxMsg, 0);
				can1[num]->RxMsg.RotateSpeed= CanRxGetU16(Can1RxMsg, 1);
				can1[num]->RxMsg.moment		= CanRxGetU16(Can1RxMsg, 2);
			}
			else Error_Handler();
		}
		else 
		{
			switch(Can1RxMsg.StdId){
				case CMFL_RXID:
					CMFLRx.angle = CanRxGetU16(Can1RxMsg, 0);
					CMFLRx.RotateSpeed = CanRxGetU16(Can1RxMsg, 1);
					break;
				case CMFR_RXID:
					CMFRRx.angle = CanRxGetU16(Can1RxMsg, 0);
					CMFRRx.RotateSpeed = CanRxGetU16(Can1RxMsg, 1);
					break;
				case CMBL_RXID:
					CMBLRx.angle = CanRxGetU16(Can1RxMsg, 0);
					CMBLRx.RotateSpeed = CanRxGetU16(Can1RxMsg, 1);
					break;
				case CMBR_RXID:
					CMBRRx.angle = CanRxGetU16(Can1RxMsg, 0);
					CMBRRx.RotateSpeed = CanRxGetU16(Can1RxMsg, 1);
					break;
				default:
				Error_Handler();
			}
		}
		if(HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK){
			isRcan1Started = 0;
		}else{
			isRcan1Started = 1;
		}
	}
	else if(hcan == &hcan2)//CAN2数据
	{
		int num=Can2RxMsg.StdId-0x201;
		if(num>=0&&num<8)//0,1,2,3,4,5,6,7
		{
			if(can2[num]!=0)
			{
				can2[num]->RxMsg.angle		= CanRxGetU16(Can1RxMsg, 0);
				can2[num]->RxMsg.RotateSpeed= CanRxGetU16(Can1RxMsg, 1);
				can2[num]->RxMsg.moment		= CanRxGetU16(Can1RxMsg, 2);
			}
			else Error_Handler();
		}
		else Error_Handler();
		if(HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK)
		{
			isRcan2Started = 0;
		}else{
			isRcan2Started = 1;
		}
	}
}



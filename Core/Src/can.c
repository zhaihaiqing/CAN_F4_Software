/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


CAN_TxHeaderTypeDef	TxHeader;	//发送消息
CAN_RxHeaderTypeDef RxHeader;	//接收消息

extern CAN_HandleTypeDef hcan1;

/*
0为显性电平、1位隐性电平，线与实现，ID低的报文，经过线与获得优先发送权，优先级高，ID高的仲裁失利后立即进入接收模式

CAN共5中帧类型：
	数据帧：用于发送单元向接收单元传送数据的帧（包含帧起始、仲裁段、控制段、数据段（0-8个字节）、CRC段、ACK段、帧结束）
	远程帧：用于接收单元向具有相同ID的发送单元请求数据的帧
	错误帧：用于当检测出错误时向其他单元通知错误的帧
	过载帧：用于接收单元通知其尚未做好接收准备的帧
	间隔帧：用于将数据帧及遥控帧与前面的帧分离开的帧
	
	
	数据帧结构：帧起始+仲裁段+控制段+数据段+CRC段+ACK段+帧结束
	
	帧起始：单个显性位组成
	仲裁段：即滤波器ID，同时也表示优先级，CAN没有节点概念，ID是指报文ID，有标准格式（11位）和扩展格式（29位）两种，RTR用于标记是否为远程帧
	控制段：6位，标准格式：IDE+R0+DLC，IDE为识别符扩展位，标准帧中位于控制段（显性），扩展帧中位于仲裁段（隐性），R0为保留位，DLC数据数据长度为（占4位），表示范围0-8
	数据段：长度0-8（字节）
	CRC段：CRC校验码
	ACK段：ACK应答，
	帧结束：7个连续的隐性位组成
	
	
	远程帧结构：帧起始+仲裁段+控制段+CRC段+ACK段+帧结束
	
	与数据帧相比，远程帧无数据段，且RTR位为1（隐性电平）
	
	
	过滤器有两种工作模式，列表模式和屏蔽码模式。
		列表模式：两个32位寄存器作为标识码和屏蔽码使用，CAN接收到的报文标识符必须和标识码、屏蔽码中的一个相同时才会通过过滤器，14组滤波器，所以总共可以过滤28种标识符。
		屏蔽码模式：标识码和屏蔽码需要一起使用，标识码的32位与屏蔽码的32为一一对应，屏蔽码某一位设置为1，则需要检查该位的标识码，如果相同，则通过过滤
		
		设置后从硬件层只过滤出需要关心的报文，也可以不设置，接收全部报文
*/




//CAN过滤器设置，硬件过滤
HAL_StatusTypeDef CAN_Filter_Config(CAN_HandleTypeDef *h_can)
{
	CAN_FilterTypeDef  sFilterConfig;
	
	sFilterConfig.FilterBank	=	0;																				//设置过滤器，单个can共有0-13（14）个，开启0号过滤器，每个滤波器有两个32bit的寄存器用来配置过滤规则，
	sFilterConfig.FilterMode	=	CAN_FILTERMODE_IDMASK;										//选择过滤器工作模式，IDMASK，过滤器共有两种模式：列表模式（数量可能不够）和掩码模式（过滤不精确）
	sFilterConfig.FilterScale	=	CAN_FILTERSCALE_32BIT;										//32bit模式
	
	sFilterConfig.FilterMaskIdHigh	=	0x0000;																//32位ID MASK，设置检查项，按位检查，0代表不检查、1代表检查
	sFilterConfig.FilterMaskIdLow		=	0x0000;																//
	
	sFilterConfig.FilterIdHigh	=	0x0000;			//32位ID
	sFilterConfig.FilterIdLow		=	0x0000;			//32位ID
	
//	sFilterConfig.FilterIdHigh	=	((((uint32_t)0x205<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff0000)>>16;			//32位ID
//	sFilterConfig.FilterIdLow		=	(((uint32_t)0x205<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff;								//32位ID
	
	
	
	sFilterConfig.FilterFIFOAssignment	=	CAN_FilterFIFO0;									//过滤器0关联到FIFO0，选择使用哪个FIFO
	sFilterConfig.FilterActivation	=	CAN_FILTER_ENABLE;										//Can过滤器使能
	sFilterConfig.SlaveStartFilterBank	= 0;
	
	if(HAL_CAN_ConfigFilter(h_can,&sFilterConfig)	!= HAL_OK)								//初始化过滤器
	{
		log_info("can filter error!\r\n");
		return HAL_ERROR;
	}
	
	if(HAL_CAN_ActivateNotification(h_can,CAN_IT_RX_FIFO0_MSG_PENDING)	!=	HAL_OK)
	{
		log_info("can an error\r\n");
		return HAL_ERROR;
	}
	 
	return HAL_OK;
}





/*
//发送Can信息

TxHeader:发送的数据帧的帧头部分
std_id：帧ID，标准帧
aData：数据帧的数据域
len：发送的数据长度

*/
uint8_t HAL_CAN_SendTxMessage(CAN_TxHeaderTypeDef *TxHeader,uint32_t std_id,uint8_t aData[],uint16_t len)
{
	uint32_t TxMailBox;
	uint8_t FreeTxMailBoxNum;
	
	
	if(len>8)
	{
		log_info("CAN Tx Data len ERROR!,len=%d,len limit is 8\r\n",len);
		len=8;
	}
	
	TxHeader->StdId	=	std_id;				//帧ID
	TxHeader->DLC	= len;						//数据域长度
	TxHeader->IDE	=	CAN_ID_STD;			//帧类型
	TxHeader->RTR	=	CAN_RTR_DATA;		//数据帧与远程帧
	TxHeader->TransmitGlobalTime	= DISABLE;
	
	while(0	==	FreeTxMailBoxNum)//等待发送邮箱不为空
	{
		FreeTxMailBoxNum	=	HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	}
	
	if(HAL_CAN_AddTxMessage(&hcan1,TxHeader,aData,&TxMailBox)	!= HAL_OK)
	{
		log_info("HAL_CAN_AddTxMessage error\r\n");
		return 0;
	}
	
	return 1;
}


uint8_t CAN_txdata[9]={0};
uint8_t CAN_rxdata[9]={0};
uint8_t CAN_rxflag=0;

//CAN设置，设置过滤器、激活CAN
void can_init(void)
{
	CAN_txdata[0]=5000>>8;
	CAN_txdata[1]=5000&0x00ff;
	
	CAN_Filter_Config(&hcan1);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	
}

//CAN接收数据回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_FilterFIFO0,&RxHeader,CAN_rxdata);
		CAN_rxflag=1;
	}	
}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

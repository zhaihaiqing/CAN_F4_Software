/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */



/* USER CODE BEGIN Prototypes */

HAL_StatusTypeDef CAN_Filter_Config(CAN_HandleTypeDef *h_can);

void can_init(void);
uint8_t HAL_CAN_SendTxMessage(CAN_TxHeaderTypeDef *TxHeader,uint32_t std_id,uint8_t aData[],uint16_t len);



extern CAN_TxHeaderTypeDef	TxHeader;
extern CAN_RxHeaderTypeDef 	RxHeader;
extern uint8_t CAN_txdata[9];
extern uint8_t CAN_rxdata[9];
extern uint8_t CAN_rxflag;

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

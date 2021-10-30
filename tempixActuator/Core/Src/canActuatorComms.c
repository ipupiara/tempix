/*
 * canComms.c
 *
 *  Created on: Jun 11, 2021
 *      Author: peetz
 */

#include <canActuatorComms.h>
#include <canRelated.h>
#include <main.h>

CAN_HandleTypeDef hcan;


void initCanFilters()
{
	HAL_StatusTypeDef  initState;
	CAN_FilterTypeDef  filterTypeDef;
	filterTypeDef.FilterScale = CAN_FILTERSCALE_16BIT;
	filterTypeDef.FilterMode = CAN_FILTERMODE_IDMASK;
	filterTypeDef.FilterIdHigh = 0x0100;   // 0x0100 on receiver side
	filterTypeDef.FilterMaskIdHigh =  0x0700;
	filterTypeDef.FilterIdLow  = 0x07FF;
	filterTypeDef.FilterMaskIdLow =	0x07FF;
	filterTypeDef.FilterBank = 0;
	filterTypeDef.SlaveStartFilterBank = 1;
	filterTypeDef.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	initState = HAL_CAN_ConfigFilter(&hcan, &filterTypeDef);
	if (initState != HAL_OK) {
		Install_Error_Handler();
	}
}


void MX_CAN_Init(void)
{


	GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();

	// CAN1 GPIO Configuration
	// PA11     ------> CAN1_RX
	// PA12     ------> CAN1_TX

	GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler =  48; //   125 kb/s;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);

  __HAL_CAN_ENABLE_IT(&hcan,CAN_IER_TMEIE);

  __HAL_CAN_ENABLE_IT(&hcan,CAN_IER_FMPIE0);
  __HAL_CAN_ENABLE_IT(&hcan,CAN_IER_FMPIE1);

  __HAL_CAN_ENABLE_IT(&hcan,CAN_IER_ERRIE);

  __HAL_CAN_ENABLE_IT(&hcan,CAN_IER_EWGIE);
  __HAL_CAN_ENABLE_IT(&hcan,CAN_IER_EPVIE);
  __HAL_CAN_ENABLE_IT(&hcan,CAN_IER_BOFIE);
  __HAL_CAN_ENABLE_IT(&hcan,CAN_IER_LECIE);



}


void respondPingRequest(uint8_t aData[])
{

}

void sendPingToTempixController()
{
	TempixSimpleCommand scmd;
	scmd.commandId = thottleActorPingRquest;
	scmd.commandData1 = 0xaaaaaaaa;
	scmd.commandData2 = 0xaaaaaaaa;
	sendCanTempixSimpleCommand(&hcan,&scmd);

}

void dispatchCanMessage( CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
	if (isCanMessageType(pHeader,controllerPingRequest)) {
		respondPingRequest(aData);
	}
}


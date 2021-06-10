/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB high priority or CAN TX interrupts.
  */
void USB_HP_CAN1_TX_IRQHandler(void)
{
	uint32_t tsrflags = (uint32_t) hcan.Instance->TSR;
	if ((tsrflags & CAN_TSR_RQCP0) != 0)		{
		// Clear the Transmission Complete flag (and TXOK0,ALST0,TERR0 bits)
		__HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_RQCP0);

//		err = OSFlagPost(TransmitMailboxStatus,mailbox0Free,OS_FLAG_SET, &err);
		if ((tsrflags & CAN_TSR_TXOK0) != 0) {
			// Transmission Mailbox 0 complete callback
		}
		else  {
			if ((tsrflags & CAN_TSR_ALST0) != 0) {
			  // Update error code
			} else if ((tsrflags & CAN_TSR_TERR0) != 0) {
			  /* Update error code */
			} else  {// Transmission Mailbox 0 abort callback  HAL_CAN_TxMailbox0AbortCallback(hcan);
			}
		}
	}

	if ((tsrflags & CAN_TSR_RQCP1) != 0)		{
		// Clear the Transmission Complete flag (and TXOK0,ALST0,TERR0 bits)
		__HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_RQCP0);

		if ((tsrflags & CAN_TSR_TXOK1) != 0) {
			// Transmission Mailbox 0 complete callback
		}
		else  {
			if ((tsrflags & CAN_TSR_ALST1) != 0) {
			  // Update error code
			} else if ((tsrflags & CAN_TSR_TERR1) != 0) {
			  /* Update error code */
			} else  {// Transmission Mailbox 0 abort callback  HAL_CAN_TxMailbox0AbortCallback(hcan);
			}
		}
	}

	if ((tsrflags & CAN_TSR_RQCP2) != 0)		{
		// Clear the Transmission Complete flag (and TXOK0,ALST0,TERR0 bits)
		__HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_RQCP2);

		if ((tsrflags & CAN_TSR_TXOK2) != 0) {
			// Transmission Mailbox 0 complete callback
		}
		else  {
			if ((tsrflags & CAN_TSR_ALST2) != 0) {
			  // Update error code
			} else if ((tsrflags & CAN_TSR_TERR2) != 0) {
			  /* Update error code */
			} else  {// Transmission Mailbox 0 abort callback  HAL_CAN_TxMailbox0AbortCallback(hcan);
			}
		}
	}
}


__weak void dispatchCanMessage( CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(&hcan);

	 /* NOTE : This function Should not be modified, when the callback is needed,
	            the HAL_CAN_TxMailbox0CompleteCallback could be implemented in the
	            user file
	   */
}

void dispatchMsgOfFifo(uint32_t RxFifo)
{
	CAN_RxHeaderTypeDef  mHeader;
	uint8_t mData[8];

	if ( HAL_CAN_GetRxMessage(&hcan, RxFifo, &mHeader, mData) == HAL_OK) {
		dispatchCanMessage(&mHeader, mData);
	}  else {
		// handle some Error
	}
}


/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	uint32_t interrupts = (uint32_t) hcan.Instance->IER;

	// Receive FIFO 0 message pending interrupt management
	if ((interrupts & CAN_IT_RX_FIFO0_MSG_PENDING) != 0) {
		// Check if message is still pending
		if ((hcan.Instance->RF0R & CAN_RF0R_FMP0) != 0) {
			dispatchMsgOfFifo(CAN_RX_FIFO0);
		}
	}

}

/**
  * @brief This function handles CAN RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
	uint32_t interrupts = (uint32_t) hcan.Instance->IER;

	// Receive FIFO 0 message pending interrupt management
	if ((interrupts & CAN_IT_RX_FIFO1_MSG_PENDING) != 0) {
		// Check if message is still pending
		if ((hcan.Instance->RF1R & CAN_RF1R_FMP1) != 0) {
			dispatchMsgOfFifo(CAN_RX_FIFO1);
		}
	}
}

/**
  * @brief This function handles CAN SCE interrupt.
  */
void CAN1_SCE_IRQHandler(void)
{
	// empty handler for method installation
	// uint32_t errorcode = HAL_CAN_ERROR_NONE;
	uint32_t interrupts = hcan.Instance->IER;
	uint32_t msrflags = hcan.Instance->MSR;
//	uint32_t tsrflags = hcan1.Instance->TSR;
//	uint32_t rf0rflags = hcan1.Instance->RF0R;
//	uint32_t rf1rflags = hcan1.Instance->RF1R;
	uint32_t esrflags = hcan.Instance->ESR;



	  //
	  if ((interrupts & CAN_IT_ERROR) != 0)
	  {
	    if ((msrflags & CAN_MSR_ERRI) != 0)
	    {
	      //
	      if (((interrupts & CAN_IT_ERROR_WARNING) != 0) &&
	          ((esrflags & CAN_ESR_EWGF) != 0))
	      {

	      }

	      //
	      if (((interrupts & CAN_IT_ERROR_PASSIVE) != 0) &&
	          ((esrflags & CAN_ESR_EPVF) != 0))
	      {

	      }

	      //
	      if (((interrupts & CAN_IT_BUSOFF) != 0) &&
	          ((esrflags & CAN_ESR_BOFF) != 0))
	      {

	      }


	      if (((interrupts & CAN_IT_LAST_ERROR_CODE) != 0) &&
	          ((esrflags & CAN_ESR_LEC) != 0))
	      {
	        switch (esrflags & CAN_ESR_LEC)
	        {
	          case (CAN_ESR_LEC_0):
	            //
	            //errorcode |= HAL_CAN_ERROR_STF;
	            break;
	          case (CAN_ESR_LEC_1):
	            //
	            //errorcode |= HAL_CAN_ERROR_FOR;
	            break;
	          case (CAN_ESR_LEC_1 | CAN_ESR_LEC_0):
	            //
	            //errorcode |= HAL_CAN_ERROR_ACK;
	            break;
	          case (CAN_ESR_LEC_2):
	            //
	            //errorcode |= HAL_CAN_ERROR_BR;
	            break;
	          case (CAN_ESR_LEC_2 | CAN_ESR_LEC_0):
	            //
	            //errorcode |= HAL_CAN_ERROR_BD;
	            break;
	          case (CAN_ESR_LEC_2 | CAN_ESR_LEC_1):
	            //
	            //errorcode |= HAL_CAN_ERROR_CRC;
	            break;
	          default:
	            break;
	        }

	        // Clear Last error code Flag
	        CLEAR_BIT(hcan.Instance->ESR, CAN_ESR_LEC);
	      }
	    }

	    // Clear ERRI Flag
	    __HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_ERRI);
	  }

	  //  /* Sleep interrupt management *********************************************/
	  //  if ((interrupts & CAN_IT_SLEEP_ACK) != RESET)
	  //  {
	  //    if ((msrflags & CAN_MSR_SLAKI) != RESET)
	  //    {
	  //      /* Clear Sleep interrupt Flag */
	  //      __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_SLAKI);
	  //
	  //      /* Sleep Callback */
	  //      /* Call weak (surcharged) callback */
	  //      HAL_CAN_SleepCallback(hcan);
	  //    }
	  //  }
	  //
	  //  /* WakeUp interrupt management *********************************************/
	  //  if ((interrupts & CAN_IT_WAKEUP) != RESET)
	  //  {
	  //    if ((msrflags & CAN_MSR_WKUI) != RESET)
	  //    {
	  //      /* Clear WakeUp Flag */
	  //      __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_WKU);
	  //
	  //      /* WakeUp Callback */
	  //      /* Call weak (surcharged) callback */
	  //      HAL_CAN_WakeUpFromRxMsgCallback(hcan);
	  //    }
	  //  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

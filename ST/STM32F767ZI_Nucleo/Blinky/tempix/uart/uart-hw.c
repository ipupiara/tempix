
#include <uart-hw.h>
#include <string.h>
#include <uart-comms.h>
#include <uosii-includes.h>

UART_HandleTypeDef huart1;      // TODO check against micrium if uart1 is ok
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

uint32_t  currentStrLen;

// structure copied from stm32f7xx_hal_dma.c
typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

// structure copied from stm32f7xx_hal_dma.c
static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  /* Clear DBM bit */
  hdma->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

  /* Configure DMA Stream data length */
  hdma->Instance->NDTR = DataLength;

  /* Memory to Peripheral */
  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
  {
    /* Configure DMA Stream destination address */
    hdma->Instance->PAR = DstAddress;

    /* Configure DMA Stream source address */
    hdma->Instance->M0AR = SrcAddress;
  }
  /* Peripheral to Memory */
  else
  {
    /* Configure DMA Stream source address */
    hdma->Instance->PAR = SrcAddress;

    /* Configure DMA Stream destination address */
    hdma->Instance->M0AR = DstAddress;
  }
}


void clearDmaInterruptFlags(DMA_HandleTypeDef *hdma)
{
	DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
	regs->IFCR = 0x3FU << hdma_usart1_tx.StreamIndex;
}

void clearUartInterruptFlags(UART_HandleTypeDef * huart)
{
	__HAL_UART_CLEAR_IT(&huart1,USART_ICR_TCCF_Msk);
	__HAL_UART_CLEAR_IT(&huart1,USART_ICR_TCCF_Msk);
}



void  Error_Handler()
{
	commsError = 1;      // ignore errror, just reset and try again, brings currently always fe, but all works always ok, no prob at all ?????
}

void incErrorCounter(DMA_HandleTypeDef *hdma)
{
	if (__HAL_DMA_GET_FLAG(hdma,__HAL_DMA_GET_TE_FLAG_INDEX(hdma))) {
		++ teCounter;
	}
	if (__HAL_DMA_GET_FLAG(hdma,__HAL_DMA_GET_FE_FLAG_INDEX(hdma))) {
		++ feCounter;
	}
	if (__HAL_DMA_GET_FLAG(hdma,__HAL_DMA_GET_DME_FLAG_INDEX(hdma))) {
		++ dmeCounter;
	}
}

void USART1_IRQHandler(void)
{
	INT8U idleDetected = 0;

    if (__HAL_UART_GET_FLAG(&huart1,USART_ISR_IDLE_Msk)  )  {
     	//  copy rest of data to receive buffer and signal receit event
    	__HAL_UART_CLEAR_IT(&huart1,USART_ISR_IDLE_Msk);
    	idleDetected = 1;
     }
    if (__HAL_UART_GET_FLAG(&huart1,USART_ICR_TCCF_Msk)  )  {
        	//  copy rest of data to receive buffer and signal receit event
    	__HAL_UART_CLEAR_IT(&huart1,USART_ICR_TCCF_Msk);
       	idleDetected = 1;
    }

     //  TODO debug the one below and check that it performs only what needed, evtl. extract and move needed instruction to here
 //    HAL_UART_IRQHandler(&huart1);  // default method of mx help nothing here.

//  todo reset isr (TC, TXE and) IDLE flag. eventually disable idle interrupt until next start sending

     /* --------------- HANDLER YOUR ISR HERE --------------- */
     if (idleDetected == 1)   {
		CPU_SR_ALLOC();

		CPU_CRITICAL_ENTER();
		 OSIntEnter();           /* Tell OS that we are starting an ISR           */
		 CPU_CRITICAL_EXIT();

		 // TODO   transfer needed bytes to receive buffer and signal received event


		 OSIntExit();
     }
}

void enableAllDmaInterrupts(DMA_HandleTypeDef* hdma)
{
	__HAL_DMA_ENABLE_IT(hdma,DMA_IT_TC);
	__HAL_DMA_ENABLE_IT(hdma,DMA_IT_HT);
	__HAL_DMA_ENABLE_IT(hdma,DMA_IT_TE);
	__HAL_DMA_ENABLE_IT(hdma,DMA_IT_FE);
	__HAL_DMA_ENABLE_IT(hdma,DMA_IT_DME);
}

void DMA2_Stream2_IRQHandler(void)
{
	CPU_SR_ALLOC();
	INT8U err = OS_ERR_NONE;

	CPU_CRITICAL_ENTER();
	OSIntEnter();           /* Tell OS that we are starting an ISR           */
	CPU_CRITICAL_EXIT();

	if (__HAL_DMA_GET_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF2_6) != 0)  {
		err = OSSemPost( dmaQSem);
		if (err != OS_ERR_NONE) {
			Error_Handler();
		}
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF2_6);
	}


    if (__HAL_DMA_GET_FLAG(&hdma_usart1_rx,DMA_FLAG_HTIF2_6) != 0)  {

    	Error_Handler();
    	__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_HTIF2_6);
    }




	if ((__HAL_DMA_GET_FLAG(&hdma_usart1_rx,DMA_FLAG_TEIF2_6))
								| (__HAL_DMA_GET_FLAG(&hdma_usart1_rx,DMA_FLAG_FEIF2_6))
								| (__HAL_DMA_GET_FLAG(&hdma_usart1_rx,DMA_FLAG_DMEIF2_6))) {
		Error_Handler();
		incErrorCounter(&hdma_usart1_rx);
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TEIF2_6);
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_FEIF2_6);
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_DMEIF2_6);
	}



	 OSIntExit();
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */



void DMA2_Stream7_IRQHandler(void)
{
	CPU_SR_ALLOC();
	CPU_CRITICAL_ENTER();
	OSIntEnter();
	CPU_CRITICAL_EXIT();

	INT8U err = OS_ERR_NONE;

	if (__HAL_DMA_GET_FLAG(&hdma_usart1_tx,DMA_FLAG_TCIF3_7) != 0)  {

		err = OSSemPost( dmaQSem);
		if (err != OS_ERR_NONE) {
			Error_Handler();
		}
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,DMA_FLAG_TCIF3_7);
	}


    if (__HAL_DMA_GET_FLAG(&hdma_usart1_tx,DMA_FLAG_HTIF3_7) != 0)  {
    	__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,DMA_FLAG_HTIF3_7);
    }

	if ((__HAL_DMA_GET_FLAG(&hdma_usart1_tx,DMA_FLAG_TEIF3_7))
								| (__HAL_DMA_GET_FLAG(&hdma_usart1_tx,DMA_FLAG_FEIF3_7))
								| (__HAL_DMA_GET_FLAG(&hdma_usart1_tx,DMA_FLAG_DMEIF3_7))) {
		Error_Handler();
		incErrorCounter(&hdma_usart1_tx);
		msgNr=txMsgCounter;  //   todo carefully abort job and deinit if possible
		errMsgStrLen = currentStrLen;
		errMsgNdtr = hdma_usart1_tx.Instance->NDTR;
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,DMA_FLAG_TEIF3_7);
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,DMA_FLAG_FEIF3_7);
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,DMA_FLAG_DMEIF3_7);
	}

	OSIntExit();
}



void MX_Usart1_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();  // todo check if this is needed
  __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


}


void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	  __HAL_RCC_DMA2_CLK_ENABLE();


	    /* USART1 DMA Init */
	    /* USART1_RX Init */
	    hdma_usart1_rx.Instance = DMA2_Stream2;
	    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
	    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
	    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
	    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
	    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
	    {
	      Error_Handler();
	    }
	    __HAL_LINKDMA(&huart1,hdmarx,hdma_usart1_rx);
	    clearDmaInterruptFlags(&hdma_usart1_rx);
	    enableAllDmaInterrupts(&hdma_usart1_rx);


	    /* USART1_TX Init */
	    hdma_usart1_tx.Instance = DMA2_Stream7;
	    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
	    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
	    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
	    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
	    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
	    {
	      Error_Handler();
	    }
	    __HAL_LINKDMA(&huart1,hdmatx,hdma_usart1_tx);
	    clearDmaInterruptFlags(&hdma_usart1_rx);
	    enableAllDmaInterrupts(&hdma_usart1_tx);


  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);    // todo check against priority in bspIntVectSet
//  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);


}

INT8U initUartHw()
{
	INT8U res =  OS_ERR_NONE;

	feCounter = 0;
	teCounter = 0;
	dmeCounter = 0;
	msgCounter = 0;
	rxMsgCounter = 0;
	txMsgCounter = 0;
	commsError = 0;
	msgNr = 0;
	currentStrLen = 0;
	errMsgNdtr = 0;
	dmaQSem = OSSemCreate(0);
	if (res == OS_ERR_NONE)
	{

		MX_Usart1_GPIO_Init();   //  need gpio clock for uart


		//  TODO  check that all  needed clocks run, else start here in first place
		//        check settings are ok with uosii

		  huart1.Instance = USART1;
		  huart1.Init.BaudRate = 57600;
		  huart1.Init.WordLength = UART_WORDLENGTH_8B;
		  huart1.Init.StopBits = UART_STOPBITS_1;
		  huart1.Init.Parity = UART_PARITY_NONE;
		  huart1.Init.Mode = UART_MODE_TX_RX;
		  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
		  if (HAL_UART_Init(&huart1) != HAL_OK)
		  {
		    //  Error_Handler();
			  res = 0xFE;
		  }
		  huart1.Instance->CR3 |= USART_CR3_DMAT_Msk;
		  clearUartInterruptFlags(&huart1);
		  huart1.Instance->CR1 |= USART_CR1_IDLEIE_Msk;
//		  huart1.Instance->CR1 |= USART_CR1_TCIE_Msk;
		  MX_DMA_Init();


		  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
		  BSP_IntVectSet (USART1_IRQn,5,0,USART1_IRQHandler);
		  BSP_IntVectSet (DMA2_Stream2_IRQn,6,0,DMA2_Stream2_IRQHandler);
		  BSP_IntVectSet (DMA2_Stream7_IRQn,7,0,DMA2_Stream7_IRQHandler);
		  disableUartInterrupts();
		  if ((hdma_usart1_tx.Instance->CR & DMA_SxCR_EN_Msk    ) == 0 ) {
			  OSSemPost( dmaQSem);
		  }  else {
			  Error_Handler();
		  }
	}



	return res;
}

INT8U startUartHw()
{
	INT8U res = 0;

	enableUartInterrupts();
	return res;

}

INT8U enableUartInterrupts()
{
	INT8U res = 0;
	commsError = 0;
	clearDmaInterruptFlags(&hdma_usart1_tx);
	clearDmaInterruptFlags(&hdma_usart1_rx);
	clearUartInterruptFlags(&huart1);
	BSP_IntEnable(USART1_IRQn);
	BSP_IntEnable(DMA2_Stream2_IRQn);
	BSP_IntEnable(DMA2_Stream7_IRQn);

	return res;
}

INT8U disableUartInterrupts()
{
	INT8U res = 0;
	BSP_IntDisable (USART1_IRQn);
	BSP_IntDisable(DMA2_Stream2_IRQn);
	BSP_IntDisable(DMA2_Stream7_IRQn);
	return res;
}

INT8U sendUartString(char* sndStr)
{
	INT8U res = 0;
	++ msgCounter;
	++ txMsgCounter;
	commsError = 0;
	currentStrLen =  strlen(sndStr);
	DMA_SetConfig(&hdma_usart1_tx, (uint32_t)sndStr, (uint32_t)&huart1.Instance->TDR, strlen(sndStr));   // incl. zero term. +1 ?

	clearDmaInterruptFlags(&hdma_usart1_tx);

	__HAL_DMA_ENABLE(&hdma_usart1_tx);

	return res;
}





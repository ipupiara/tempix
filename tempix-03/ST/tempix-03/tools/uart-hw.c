
#include <uart-hw.h>
#include <string.h>
#include <uart-comms.h>
#include <uosii-includes.h>
#include <dma-tools.h>

/*
 * ATTENTION:  implementation of UART receiver as done here is applicable only for
 *  manually entered data (single strings of restricted size, randomly sent).
 *  otherwise a faster and more sophisticated reception capability of buffers has to
 *  be implemented.
 *
 *  max data received at once must be less than fullBufferSize
 *
 *  message dispatch is done inside isr for simplicity reason as this method is only used for
 *  for manual debugging, testing and parameter finetuning (saved to eeprom chip). So for
 *  productive software run this "isr sin" is without any impact on interrupt latency, as even
 *  it is planned to enable uart comms initialization and usage only via jumper on cpu board.
 *
*/

UART_HandleTypeDef huart1;      // TODO check against micrium if uart1 is ok
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

enum {
	withoutHT = 0,
	withHT
};

enum {
	fromHalfTransferCompleteIsr = 1,
	fromTransferCompleteIsr,
	fromUartIsr
};


void clearUartInterruptFlags(UART_HandleTypeDef * huart)
{
	__HAL_UART_CLEAR_IT(&huart1,USART_ICR_TCCF_Msk);
	__HAL_UART_CLEAR_IT(&huart1,USART_ICR_TCCF_Msk);
}

void  uart_dmaError_handler(DMA_HandleTypeDef *hdma)
{
//  TODO for later, debug why (with 8mhz crystal and 216 mhz cpu freq?) fe error often happen
//	incDMAErrorCounter(hdma);
//	++ commsError;
//	commsError = 1;     // ignore error, just reset and try again, brings currently always fe, but all works always ok, no prob at all ?????
}


#define bufferCounterType INT8U
#define halfDmaRxBufferSize   0x20
#define fullDmaRxBufferSize 2 * halfDmaRxBufferSize
#define endBufferPosition ( fullDmaRxBufferSize - 1 )
#define maxDmaStringSize  0x80

#define debugArraySize 0xFF
INT8U  debugArray [debugArraySize];
INT8U  debugArrayCnt;

void resetDebugArray()
{
	memset(debugArray,0x00,sizeof(debugArray));
	debugArrayCnt = 0;
}

void addToDebugArray(INT8U from,INT8U val)
{
/*	if (debugArrayCnt < debugArraySize+3) {
		debugArray[debugArrayCnt] = from;
		debugArray[debugArrayCnt+1] = val;
		debugArrayCnt += 3;
	}*/
}


INT8U  receiveStringBuffer  [maxDmaStringSize + 1];

typedef struct
{
  INT8U firsthalf[halfDmaRxBufferSize];
  INT8U secondhalf[halfDmaRxBufferSize];
} halfBufferTwice_Buffer_Type;

typedef union
{
	halfBufferTwice_Buffer_Type halfTwicebuffer;
	INT8U byteBuffer [fullDmaRxBufferSize];
} DMA_Buffer_Type;

DMA_Buffer_Type dmaBuffer;

bufferCounterType lastNdtr;
bufferCounterType amtWrittenStringChars;



void resetStringBuffer()
{
	amtWrittenStringChars = 0;
	memset(receiveStringBuffer,0x00,sizeof(receiveStringBuffer));
}

void resetDmaBuffer()
{
	lastNdtr = fullDmaRxBufferSize;
	memset(&dmaBuffer,0x00,sizeof(dmaBuffer));
}

void transferBuffer(INT8U  tobeForwardedFrom)
{
	bufferCounterType newNdtr= (INT32U)  (hdma_usart1_rx.Instance->NDTR);
	INT8U dummyInt;
	bufferCounterType amtRcvd;
	if (tobeForwardedFrom == fromUartIsr)  {
			// TODO  look how come in the values
		dummyInt = 0;
	}
	if (tobeForwardedFrom == fromTransferCompleteIsr)  {
		amtRcvd = lastNdtr;
	}  else {
		amtRcvd =   lastNdtr  - newNdtr;
	}
	// is lastNdtr on TC already 0 ???
	INT8U receivedAt = fullDmaRxBufferSize - lastNdtr;

	bufferCounterType amtCpy = amtRcvd;
	// evtl if inTc ndtr might already be reset ??

	if (amtWrittenStringChars < maxDmaStringSize )  {
		if (amtWrittenStringChars + amtRcvd > maxDmaStringSize) {
			amtCpy = maxDmaStringSize  - amtWrittenStringChars;
		}
	} else {
		amtCpy = 0;
	}

	if (amtCpy > 0) {
		for (INT8U cnt =0; cnt < amtCpy;++cnt) {
			receiveStringBuffer[amtWrittenStringChars + cnt] = dmaBuffer.byteBuffer[receivedAt + cnt];
		}
	//	strncpy(stringBuffer[amtWrittenStringChars],dmaBuffer[amtReadBufferChars],amtCpy);
	}
	//	memcpy(&stringBuffer[amtWrittenStringChars],&dmaBuffer.byteBuffer[amtReadBufferChars],amtCpy);

	lastNdtr = newNdtr;
	amtWrittenStringChars += amtCpy;
	addToDebugArray(tobeForwardedFrom,amtWrittenStringChars);

	if (tobeForwardedFrom == fromTransferCompleteIsr)  {
		resetDmaBuffer();
	} else if (tobeForwardedFrom == fromUartIsr) {
		if (amtWrittenStringChars > 0) {
			forwardReceivedStringBuffer((char*)receiveStringBuffer);
			resetStringBuffer();
		}
	}
	if (dummyInt > 0xf0) {
		info_printf("should  never come here\n");
	}
}


INT32U debugIdleCounter;


static void USART1_IRQHandler(void)
{
	INT8U idleDetected = 0;

    if (__HAL_UART_GET_FLAG(&huart1,USART_ISR_IDLE_Msk)  )  {
    	__HAL_UART_CLEAR_IT(&huart1,USART_ISR_IDLE_Msk);
    	idleDetected = 1;
    	++ debugIdleCounter;
     }
    if (__HAL_UART_GET_FLAG(&huart1,USART_ICR_TCCF_Msk)  )  {
        	//  copy rest of data to receive buffer and signal received event
    	__HAL_UART_CLEAR_IT(&huart1,USART_ICR_TCCF_Msk);
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

	     	//  copy rest of data to receive buffer and signal receit event
	    	transferBuffer(fromUartIsr);   //  TODO check concurrency behaviour with dma interrupt !!!
	    						//  is idle delay ok?
	    	//  handle string over to uartReceiveHandler  (thread)
	    	//  evtl. better copy to new variable, so a next one can be read in, if handling is done async
	    	//  handle async data with accesscounter odd/even for simplicity reason
	    	//amtWrittenStringChars = 0; // reset string for next read from uart



		 OSIntExit();
		 //  todo check eie (error interrupt enabled) and enable in cr
     }
}

void enableAllDmaInterrupts(DMA_HandleTypeDef* hdma, INT8U exceptHT)
{
	__HAL_DMA_ENABLE_IT(hdma,DMA_IT_TC);
	if (exceptHT != withoutHT) {
		__HAL_DMA_ENABLE_IT(hdma,DMA_IT_HT);
	}
	__HAL_DMA_ENABLE_IT(hdma,DMA_IT_TE);
	__HAL_DMA_ENABLE_IT(hdma,DMA_IT_FE);
	__HAL_DMA_ENABLE_IT(hdma,DMA_IT_DME);
}

void DMA2_Stream2_IRQHandler(void)   // RX
{
	CPU_SR_ALLOC();
//	INT8U err = OS_ERR_NONE;

	CPU_CRITICAL_ENTER();
	OSIntEnter();           /* Tell OS that we are starting an ISR           */
	CPU_CRITICAL_EXIT();

	if (__HAL_DMA_GET_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF2_6) != 0)  {
		transferBuffer(fromTransferCompleteIsr);
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF2_6);
	}


    if (__HAL_DMA_GET_FLAG(&hdma_usart1_rx,DMA_FLAG_HTIF2_6) != 0)  {
    	transferBuffer(fromHalfTransferCompleteIsr);
    	__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_HTIF2_6);
    }

	if ((__HAL_DMA_GET_FLAG(&hdma_usart1_rx,DMA_FLAG_TEIF2_6))
								| (__HAL_DMA_GET_FLAG(&hdma_usart1_rx,DMA_FLAG_FEIF2_6))
								| (__HAL_DMA_GET_FLAG(&hdma_usart1_rx,DMA_FLAG_DMEIF2_6))) {
		uart_dmaError_handler(&hdma_usart1_rx);
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TEIF2_6);
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_FEIF2_6);
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_DMEIF2_6);
	}
	 OSIntExit();
}


void DMA2_Stream7_IRQHandler(void)   // TX
{
	CPU_SR_ALLOC();
	CPU_CRITICAL_ENTER();
	OSIntEnter();
	CPU_CRITICAL_EXIT();

	INT8U err = OS_ERR_NONE;
	if (__HAL_DMA_GET_FLAG(&hdma_usart1_tx,DMA_FLAG_TCIF3_7) != 0)  {

		err = OSSemPost( dmaQSem);
		if (err != OS_ERR_NONE) {
			uart_dmaError_handler(&hdma_usart1_tx);
		}
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,DMA_FLAG_TCIF3_7);
	}

    if (__HAL_DMA_GET_FLAG(&hdma_usart1_tx,DMA_FLAG_HTIF3_7) != 0)  {
    	__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,DMA_FLAG_HTIF3_7);
    }

	if ((__HAL_DMA_GET_FLAG(&hdma_usart1_tx,DMA_FLAG_TEIF3_7))
								| (__HAL_DMA_GET_FLAG(&hdma_usart1_tx,DMA_FLAG_FEIF3_7))
								| (__HAL_DMA_GET_FLAG(&hdma_usart1_tx,DMA_FLAG_DMEIF3_7))) {
		uart_dmaError_handler(&hdma_usart1_tx);
	  //   todo carefully abort job and deinit if possible

		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,DMA_FLAG_TEIF3_7);
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,DMA_FLAG_FEIF3_7);
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,DMA_FLAG_DMEIF3_7);
	}

	OSIntExit();
}



void usart1_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();  // todo check if this is needed
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


void uart_DMA_Init(void)
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
	    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
	    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
	    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
	    {
	      uart_dmaError_handler(&hdma_usart1_rx);
	    }
//	    __HAL_LINKDMA(&huart1,hdmarx,hdma_usart1_rx);
	    clearDmaInterruptFlags(&hdma_usart1_rx);
	    enableAllDmaInterrupts(&hdma_usart1_rx,withHT);
	    resetDmaBuffer();

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
	      uart_dmaError_handler(&hdma_usart1_tx);
	    }
//	    __HAL_LINKDMA(&huart1,hdmatx,hdma_usart1_tx);
	    clearDmaInterruptFlags(&hdma_usart1_tx);
	    enableAllDmaInterrupts(&hdma_usart1_tx,withoutHT);

	    BSP_IntVectSet (DMA2_Stream2_IRQn,7,CPU_INT_KA,DMA2_Stream2_IRQHandler);
	    BSP_IntVectSet (DMA2_Stream7_IRQn,7,CPU_INT_KA,DMA2_Stream7_IRQHandler);
}

void startCircReceiver()
{
	DMA_SetTransferConfig(&hdma_usart1_rx, (uint32_t)&huart1.Instance->RDR , (uint32_t)&dmaBuffer, sizeof(dmaBuffer));
	clearDmaInterruptFlags(&hdma_usart1_rx);
	__HAL_DMA_ENABLE(&hdma_usart1_rx);
}


INT8U initUartHw()
{
	INT8U res =  OS_ERR_NONE;

	debugIdleCounter = 0;
	resetDebugArray();

	feCounter = 0;
	teCounter = 0;
	dmeCounter = 0;
	rxMsgCounter = 0;
	txMsgCounter = 0;
	commsError = 0;

	dmaQSem = OSSemCreate(0);

	resetStringBuffer();

	 __HAL_RCC_USART1_CLK_ENABLE();

	if (res == OS_ERR_NONE)
	{

		usart1_GPIO_Init();   //  need gpio clock for uart


		//  TODO  check that all  needed clocks run, else start here in first place
		//        check settings are ok with uosii

		  huart1.Instance = USART1;
		  huart1.Init.BaudRate = 9600;
		  huart1.Init.WordLength = UART_WORDLENGTH_8B;
		  huart1.Init.StopBits = UART_STOPBITS_1;
		  huart1.Init.Parity = UART_PARITY_NONE;
		  huart1.Init.Mode = UART_MODE_TX_RX;
//		  huart1.Init.Mode = UART_MODE_TX;
		  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
		  if (HAL_UART_Init(&huart1) != HAL_OK)
		  {
		    //  Error_Handler();
			  res = 0xFE;
		  }
		  huart1.Instance->CR3 |= (USART_CR3_DMAR_Msk | USART_CR3_DMAT_Msk);
		  clearUartInterruptFlags(&huart1);
		  huart1.Instance->CR1 |= USART_CR1_IDLEIE_Msk;
//		  huart1.Instance->CR1 |= USART_CR1_TCIE_Msk;


		  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
		  BSP_IntVectSet (USART1_IRQn,9,CPU_INT_KA,USART1_IRQHandler);

		  uart_DMA_Init();

		  disableUartInterrupts();
		  if ((hdma_usart1_tx.Instance->CR & DMA_SxCR_EN_Msk    ) == 0 ) {
			  OSSemPost( dmaQSem);
		  }  else {
			  uart_dmaError_handler(&hdma_usart1_tx);
		  }
	}
	return res;
}

INT8U startUartHw()
{
	INT8U res = 0;
	startCircReceiver();
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

void deInitUart()
{
	disableUartInterrupts();
	//  TODO add disable, clock shutdown ....
}

INT8U sendUartString(char* sndStr)
{
	INT8U res = 0;
	++ txMsgCounter;
//	commsError = 0;

	DMA_SetTransferConfig(&hdma_usart1_tx, (uint32_t)sndStr, (uint32_t)&huart1.Instance->TDR, strlen(sndStr));

	clearDmaInterruptFlags(&hdma_usart1_tx);

	__HAL_DMA_ENABLE(&hdma_usart1_tx);

	// TODO wait until buffer is sent since the caller expects a sysnchronous method
	return res;
}


#include <string.h>
#include <i2c.h>
#include <dma-tools.h>
#include <uosii-includes.h>
#include <StateClass.h>
#include <main.h>
#include <uart-comms.h>

#define I2C_FLAG_NACKF   I2C_FLAG_AF

OS_EVENT *i2cResourceSem;



OS_EVENT *i2cJobSem;

I2C_HandleTypeDef hi2c1;

uint8_t  transmitErrorCollectorInt8u;
uint8_t  jobSemSet;

//OS_STK  i2cMethodStk[APP_CFG_DEFAULT_TASK_STK_SIZE];

void reInitOnError();

typedef enum  {
	sendI2c = 0,
	receiveI2c,
} jobTypes;


typedef struct
{
	jobTypes  jobType;
	INT8U*  buffer;
	INT8U	amtChars;
	INT8U   bufferCnt;
	INT8U   address;
} i2cJobDataType;

i2cJobDataType i2cJobData;

void setI2cJobSema()
{
	INT8U err;
//	OSSemSet(i2cJobSem,1,&err);
	if (jobSemSet == 0)  {    // prevent multiple events by irqs
		err = OSSemPost(i2cJobSem);
		jobSemSet = 1;
		if (err != 0)  {
			jobSemSet = 2;  //  just for debugging
		}
	}
}

void i2cFinishedOk()
{
	setI2cJobSema();
}


void i2cError(INT8U err)
{
	 //log error
	 transmitErrorCollectorInt8u = err;
	 setI2cJobSema();

}


void enableI2c()
{
	 __HAL_I2C_ENABLE(&hi2c1);
}

void disableI2c()
{
	__HAL_I2C_DISABLE(&hi2c1);
}


#ifdef i2cUseDma

DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;


void DMA1_Stream0_IRQHandler(void)
{
	CPU_SR_ALLOC();
//	INT8U err = OS_ERR_NONE;

	CPU_CRITICAL_ENTER();
	OSIntEnter();           /* Tell OS that we are starting an ISR           */
	CPU_CRITICAL_EXIT();

	if (__HAL_DMA_GET_FLAG(&hdma_i2c1_rx,DMA_FLAG_TCIF0_4) != 0)  {
//		transferBuffer();
//		i2cFinishedOk();
		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_rx,DMA_FLAG_TCIF0_4);
	}


    if (__HAL_DMA_GET_FLAG(&hdma_i2c1_rx,DMA_FLAG_HTIF0_4) != 0)  {
//    	transferBuffer();
    	__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_rx,DMA_FLAG_HTIF0_4);
    }

	if ((__HAL_DMA_GET_FLAG(&hdma_i2c1_rx,DMA_FLAG_TEIF0_4))
								| (__HAL_DMA_GET_FLAG(&hdma_i2c1_rx,DMA_FLAG_FEIF0_4))
								| (__HAL_DMA_GET_FLAG(&hdma_i2c1_rx,DMA_FLAG_DMEIF0_4))) {
		incDMAErrorCounter(&hdma_i2c1_rx);
		i2cError(dmaIsr(&hdma_i2c1_rx ));
		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_rx,DMA_FLAG_TEIF0_4);
		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_rx,DMA_FLAG_FEIF0_4);
		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_rx,DMA_FLAG_DMEIF0_4);
	}
	 OSIntExit();
}

void DMA1_Stream6_IRQHandler(void)
{
	CPU_SR_ALLOC();
//	INT8U err = OS_ERR_NONE;

	CPU_CRITICAL_ENTER();
	OSIntEnter();           /* Tell OS that we are starting an ISR           */
	CPU_CRITICAL_EXIT();

	if (__HAL_DMA_GET_FLAG(&hdma_i2c1_tx,DMA_FLAG_TCIF2_6) != 0)  {
		i2cFinishedOk();
		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_tx,DMA_FLAG_TCIF2_6);
	}


    if (__HAL_DMA_GET_FLAG(&hdma_i2c1_tx,DMA_FLAG_HTIF2_6) != 0)  {
//    	transferBuffer();
    	__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_tx,DMA_FLAG_HTIF2_6);
    }

	if ((__HAL_DMA_GET_FLAG(&hdma_i2c1_tx,DMA_FLAG_TEIF2_6))
								| (__HAL_DMA_GET_FLAG(&hdma_i2c1_tx,DMA_FLAG_FEIF2_6))
								| (__HAL_DMA_GET_FLAG(&hdma_i2c1_tx,DMA_FLAG_DMEIF2_6))) {
		incDMAErrorCounter(&hdma_i2c1_tx);
		i2cError(dmaIsr(&hdma_i2c1_tx));
		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_tx,DMA_FLAG_TEIF2_6);
		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_tx,DMA_FLAG_FEIF2_6);
		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_tx,DMA_FLAG_DMEIF2_6);
	}
	 OSIntExit();
}

void i2cDmaInit()
{

    /* I2C1 DMA Init */
    /* I2C1_RX Init */
    hdma_i2c1_rx.Instance = DMA1_Stream0;
    hdma_i2c1_rx.Init.Channel = DMA_CHANNEL_1;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
    	i2cError(0x85);
    }

    __HAL_LINKDMA(&hi2c1,hdmarx,hdma_i2c1_rx);

    /* I2C1_TX Init */
    hdma_i2c1_tx.Instance = DMA1_Stream6;
    hdma_i2c1_tx.Init.Channel = DMA_CHANNEL_1;
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
    {
    	i2cError(0x86);
    }
    __HAL_LINKDMA(&hi2c1,hdmatx,hdma_i2c1_tx);

	  __HAL_I2C_TxDmaENABLE(&hi2c1);
	  __HAL_I2C_RxDmaENABLE(&hi2c1);

	  BSP_IntVectSet (DMA1_Stream0_IRQn,tempixIsrPrioLevel,CPU_INT_KA,DMA1_Stream0_IRQHandler);
	  BSP_IntVectSet (DMA1_Stream6_IRQn,tempixIsrPrioLevel,CPU_INT_KA,DMA1_Stream6_IRQHandler);
	  BSP_IntEnable(DMA1_Stream0_IRQn);
	  BSP_IntEnable(DMA1_Stream6_IRQn);

	clearDmaInterruptFlags(&hdma_i2c1_tx);
	clearDmaInterruptFlags(&hdma_i2c1_rx);
	enableAllDmaInterrupts(&hdma_i2c1_tx,withoutHT);
	enableAllDmaInterrupts(&hdma_i2c1_rx,withoutHT);

}

#else

void sendNextI2CByte()
{
	if (i2cJobData.bufferCnt < i2cJobData.amtChars) {
		hi2c1.Instance->TXDR = i2cJobData.buffer[i2cJobData.bufferCnt];
		++i2cJobData.bufferCnt;
	}
}

void receiveNextI2CByte()
{
	if (i2cJobData.bufferCnt < i2cJobData.amtChars) {
			i2cJobData.buffer[i2cJobData.bufferCnt] = hi2c1.Instance->RXDR ;
			++i2cJobData.bufferCnt;
	}
}

#endif



void establishContactAndRun()
{
#ifdef i2cUseDma
	if (i2cJobData.jobType == sendI2c) {
		DMA_SetTransferConfig(&hdma_i2c1_tx,(uint32_t)i2cJobData.buffer,(uint32_t)&hi2c1.Instance->TXDR,i2cJobData.amtChars);
		clearDmaInterruptFlags(&hdma_i2c1_tx);
		__HAL_DMA_ENABLE(&hdma_i2c1_tx);
	} else {
		DMA_SetTransferConfig(&hdma_i2c1_rx,(uint32_t)&hi2c1.Instance->RXDR,(uint32_t)i2cJobData.buffer,i2cJobData.amtChars);
		clearDmaInterruptFlags(&hdma_i2c1_rx);
		__HAL_DMA_ENABLE(&hdma_i2c1_rx);
	}
#endif
	i2cTransferConfig(&hi2c1,i2cJobData.address,i2cJobData.amtChars,(i2cJobData.jobType == receiveI2c ? 1:0));
	i2cSendStart(&hi2c1);
}

void enableI2cInterrupts()
{
#ifdef i2cUseDma
	clearDmaInterruptFlags(&hdma_i2c1_rx);
	clearDmaInterruptFlags(&hdma_i2c1_tx);
	BSP_IntEnable(DMA1_Stream6_IRQn);
	BSP_IntEnable(DMA1_Stream0_IRQn);
#endif
//	clearUartInterruptFlags(&huart1);
	BSP_IntEnable(I2C1_ER_IRQn);
	BSP_IntEnable(I2C1_EV_IRQn);



}

void disableI2cInterrupts()
{
#ifdef i2cUseDma
	BSP_IntDisable(DMA1_Stream6_IRQn);
	BSP_IntDisable(DMA1_Stream0_IRQn);
#endif
	BSP_IntDisable(I2C1_ER_IRQn);
	BSP_IntDisable(I2C1_EV_IRQn);
}



void I2C1_EV_IRQHandler(void)
{
	uint32_t itflags   = READ_REG(hi2c1.Instance->ISR);
#ifndef i2cUseDma
	if ((itflags & I2C_FLAG_TXIS) != 0)   {
		sendNextI2CByte();
	}
	if ((itflags & I2C_FLAG_RXNE) != 0)   {
		receiveNextI2CByte();
	}
#endif
	if ((itflags & I2C_FLAG_TC) != 0)  {
		i2cFinishedOk();
	}
	if (((itflags & I2C_FLAG_STOPF) != 0)| ((itflags & I2C_FLAG_NACKF) != 0) )  {
		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);
		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_NACKF);
		  i2cError(0x77);
	}
}

void I2C1_ER_IRQHandler(void)
{
	CPU_SR_ALLOC();
//	INT8U err = OS_ERR_NONE;

	CPU_CRITICAL_ENTER();
	OSIntEnter();           /* Tell OS that we are starting an ISR           */
	CPU_CRITICAL_EXIT();
	// copied from stm32f7xx_hal_i2d.c
	uint32_t itflags   = READ_REG(hi2c1.Instance->ISR);
	uint32_t itsources = READ_REG(hi2c1.Instance->CR1);
	  /* I2C Bus error interrupt occurred ------------------------------------*/
	  if (((itflags & I2C_FLAG_BERR) != RESET) && ((itsources & I2C_IT_ERRI) != RESET))
	  {
	    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_BERR);
	  }

	  /* I2C Over-Run/Under-Run interrupt occurred ----------------------------------------*/
	  if (((itflags & I2C_FLAG_OVR) != RESET) && ((itsources & I2C_IT_ERRI) != RESET))
	  {
	    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_OVR);
	  }

	  /* I2C Arbitration Loss error interrupt occurred -------------------------------------*/
	  if (((itflags & I2C_FLAG_ARLO) != RESET) && ((itsources & I2C_IT_ERRI) != RESET))
	  {
	    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_ARLO);
	  }
	  i2cError(0x82);  //  todo implement refined error message with above details....

	  // next two ifs are just for debugging reasons
	  if ((itflags & I2C_FLAG_AF) != 0) {   //  should actually be named I2C_FLAG_NACKF. how this name ?
		  i2cError(0x69);
	  }
	  if ((itflags & I2C_FLAG_STOPF) != 0) {
		  i2cError(0x96);
	  }


	CPU_CRITICAL_ENTER();
	reInitOnError();
	CPU_CRITICAL_EXIT();

	OSIntExit();
}

INT8U transmitI2cByteArray(INT8U adr,INT8U* pResultString,INT8U amtChars, INT8U doSend, uint8_t delayMs)
{
	uint8_t res = 0xFF;

	if ((i2cInitialized == 1) ) {          //&& (OSIntNesting > 0u))
		INT8U semErr;
		OSSemPend(i2cResourceSem, 2803, &semErr);
		if (semErr == OS_ERR_NONE) {
			transmitErrorCollectorInt8u = OS_ERR_NONE;
			OSSemSet(i2cJobSem,0,&semErr);  // debug: be sure it was not set multiple times at last end of transfer..
			jobSemSet = 0;
			i2cJobData.buffer = pResultString;
			i2cJobData.amtChars = amtChars;
			i2cJobData.bufferCnt = 0;
			i2cJobData.address = adr;
			if (doSend == 1) {
				i2cJobData.jobType = sendI2c;
			} else {
				i2cJobData.jobType = receiveI2c;
				if (pResultString != 0) {
				memset(pResultString,0,amtChars);  // todo check if this work correct (not content of pointer variable is changed)
				}
			}

			establishContactAndRun();

			OSSemPend(i2cJobSem, 0, &semErr);
			if (semErr != OS_ERR_NONE) {
				transmitErrorCollectorInt8u = semErr;
			}
			if (delayMs > 0) {
				OSTimeDlyHMSM(0, 0, 0, delayMs);
			}
			//  todo wait until data written into eeprom memory
			OSSemSet(i2cResourceSem, 1, &semErr);
			res = transmitErrorCollectorInt8u;
		}  else {
			res = semErr;
		}
	}
	return res;
}

INT8U sendI2cByteArray(INT8U adr,INT8U* pString,INT8U amtChars, uint8_t delayMs)
{
	return transmitI2cByteArray(adr, pString, amtChars, 1, delayMs);
}

INT8U receiveI2cByteArray(INT8U adr,INT8U* pString,INT8U amtChars, uint8_t delayMs)
{
	return transmitI2cByteArray(adr, pString, amtChars, 0, delayMs);
}



uint8_t pollForReady(INT8U adr, uint8_t delay)
{
	int8_t res = 0xFF;
//	int8_t resOnErrStack = resetOnError;
//	resetOnError = 0;
	uint8_t dummyBuffer [1];

	while (res != 0) {
		sendI2cByteArray(adr,&dummyBuffer[0],0, delay);  // do just a very short delay if desired
//		resetOnError = resOnErrStack;
	}
    return res;
}


INT8U initI2c()
{

	i2cInitialized = 0;
//	resetOnError = 1;
	resetOnError = 0;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	INT8U err = OS_ERR_NONE;
	if (err == OS_ERR_NONE) {
		 i2cJobSem = OSSemCreate(0);
	}
	if (err == OS_ERR_NONE) {
		 i2cResourceSem = OSSemCreate(1);
	}

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    __HAL_RCC_I2C1_CLK_ENABLE();

       disableI2c();
     //   	OSTimeDlyHMSM(0u, 0u, 1u, 0u);  // wait for uart/dma ready,  else fe happens when immediately sending a msg
     //     use this block if reset of i2c should be needed

     // //  enableI2c();

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
	  i2cError(0x87);
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
	  i2cError(0x88);
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
	  i2cError(0x89);
  }
  BSP_IntVectSet (I2C1_EV_IRQn,tempixIsrPrioLevel,CPU_INT_KA,I2C1_EV_IRQHandler);
  BSP_IntVectSet (I2C1_ER_IRQn,tempixIsrPrioLevel,CPU_INT_KA,I2C1_ER_IRQHandler);

  BSP_IntEnable(I2C1_EV_IRQn);
  BSP_IntEnable(I2C1_ER_IRQn);

  __HAL_I2C_ENABLE_IT(&hi2c1,(I2C_IT_ERRI | I2C_IT_TCI));
  __HAL_I2C_ENABLE_IT(&hi2c1,(I2C_IT_STOPI | I2C_IT_NACKI));
//  __HAL_I2C_ENABLE_IT(&hi2c1,(I2C_IT_STOPI | I2C_IT_NACKI));
  __HAL_I2C_AutoEndENABLE(&hi2c1);

#ifdef i2cUseDma
  i2cDmaInit();
#else
  __HAL_I2C_ENABLE_IT(&hi2c1,(I2C_IT_RXI | I2C_IT_TXI));
#endif

  enableI2c();
  i2cInitialized = 1;
  return err; // error return does not really make sense.....
}



void reInitI2cAfterError()   // called from backgroundEventQ
{
	INT8U err = OS_ERR_NONE;
	OSSemDel(i2cJobSem, OS_DEL_ALWAYS, &err);
	OSSemDel(i2cResourceSem, OS_DEL_ALWAYS, &err);
	initI2c();
}


void reInitOnError()
{
	i2cSendStop(&hi2c1);
	if (resetOnError != 0)  {
		uint8_t err = OS_ERR_NONE;
		backGroundEvent *  bgEvPtr;
		i2cInitialized = 0;
		disableI2c();
		bgEvPtr = (backGroundEvent *) OSMemGet(backGroundEventMem, &err);
		if( bgEvPtr != 0 ) {
			bgEvPtr->evType = i2cReinitNeeded;
			err = OSQPost(backGroundEventTaskQ, (void *)bgEvPtr);
		}
		if ( err != OS_ERR_NONE) {
			info_printf("critical: could not reset i2c\n");
		}
	}
}

/*
 * uart-comms.c
 *
 *  Created on: Oct 12, 2020
 *      Author: ixchel
 */


#include <string.h>
#include <math.h>
#include <stdarg.h>



#include "uart-comms.h"
#include "uart-hw.h"

INT32U  amtErr;			// amt calls to err_printf
INT32U  amtPrintErr;   // errors during print out, where err_printf should not be called
INT8U  err_init_print;

OS_MEM *serialMsgMem;
OS_STK  SerialQMethodStk[APP_CFG_DEFAULT_TASK_STK_SIZE];
//static  void  SerialQMethod (void *p_arg);






OS_EVENT*  serialMsgTaskQ;
serialMem serialStrBuf[serialStrBufSz];
void* serialTaskQMsg[serialTaskQMsgSz];


void OnPrintError()
{
  ++amtPrintErr;  // at least keep the number for debugging, maybe later an led could be switched on or so...
}


static  void  SerialQMethod (void *p_arg)
{
	INT8U err;
	serialMem* sm;

     while (1) {
    	 sm = (serialMem *)OSQPend(serialMsgTaskQ, 1097, &err);
    	 if (err == OS_ERR_NONE) {
    		 do {
				 OSSemPend(dmaQSem, 107, &err);
				 if (err == OS_ERR_NONE){
					 OSSemSet(dmaQSem, 0, &err);  // by initalization to value of 1, if ever the count
												 // should get higher than one, what does not make sense
												 // next string might get lost
					 if (err  != OS_ERR_NONE) {
						 OnPrintError();
					 }
					 strncpy(currentSerialMem.serialStr,sm->serialStr,serialStrSz-1);
					 currentSerialMem.serialStr[serialStrSz-1] = 0;    // for safety reason
					 sendUartString(currentSerialMem.serialStr);
					 err = OSMemPut(serialMsgMem, (void *)sm);
				 } else {
					 if ((err == OS_ERR_TIMEOUT))
					 {   // TODO check ready to send string, but sema accidentally not set, recover situation

					 }
				 }
				 if (err != OS_ERR_NONE) {
					OnPrintError();   // err_printf("mem put problem sec100 method\n");
				 }
    		 } while (err  != OS_ERR_NONE);
    	 } else {
    	  // Message not received, must have timed out
    	// better not err_printf("serial Q timeout\n");
    		 // but maybe set a pin, or other message on different device
    	 }
    }
}



void init_printf()
{

	serialOn = 0;
	amtErr = 0;
	amtPrintErr = 0;
	err_init_print = OS_ERR_NONE;

	if (err_init_print == OS_ERR_NONE) {
	//	BSP_USART_Init (APP_USART_COM, 57600);  // start with something slow, later increase

		if (!(dmaQSem = OSSemCreate(0)))
		// after initialisation, probable a character can be sent
		// worst case we just loose it
		{
			//err_printf("could not create serialQSem\n");
			err_init_print = 0xFF;
		}
	}

	memset(serialStrBuf,0x00,sizeof(serialStrBuf));
	if (err_init_print == OS_ERR_NONE) {
		serialMsgMem = OSMemCreate(&serialStrBuf[0], serialStrBufSz, sizeof(serialMem), &err_init_print);
	}
	if (err_init_print == OS_ERR_NONE) {
		OSMemNameSet(serialMsgMem, (INT8U*)"serialMsgMem", &err_init_print);
	}
	if (err_init_print == OS_ERR_NONE) {
		serialMsgTaskQ = OSQCreate(&serialTaskQMsg[0], serialTaskQMsgSz);
		if (! serialMsgTaskQ) err_init_print = 0xFF;
	}


//	if (err_init_print == OS_ERR_NONE) {
//		if (BSP_INTC_IntReg(&APP_USARTRxTxISR, App_IRQ_USART, 1) == BSP_INTC_ERR_NONE ) {
//			err_init_print = OS_ERR_NONE;
//		} else {
//			err_init_print = 0xFF;
//		}
//
//	}
//	BSP_USART_IntEn (APP_USART_COM, (1<< AVR32_USART_IER_TXRDY));



	if (err_init_print ==  OS_ERR_NONE  )  {


		err_init_print = OSTaskCreateExt(SerialQMethod,                                       /* Create the start task                                    */
			                    (void *)0,
			                    (OS_STK *)&SerialQMethodStk[APP_CFG_DEFAULT_TASK_STK_SIZE - 1],
			                    SerialQ_TASK_PRIO,
			                    SerialQ_TASK_PRIO,
			                    (OS_STK *)&SerialQMethodStk[0],
								APP_CFG_DEFAULT_TASK_STK_SIZE,
			                    (void *)0,
			                    OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

			                   // at least count the errros for debugging reasons


			    OSTaskNameSet(SerialQ_TASK_PRIO, (INT8U *)"SerQ", &err_init_print);
	}
	if (err_init_print ==  OS_ERR_NONE  )  {
		initUartHw();
	}

	if (err_init_print !=  OS_ERR_NONE  ) {OnPrintError(); }
	serialOn = (err_init_print == OS_ERR_NONE);

}

void printStartMessage()
{
	info_printf("serial print ready for use\n");
	info_printf("second test ok\n");
	info_printf("third test ok\n");
	info_printf("12345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890");
}

void start_printf()
{
	startUartHw();
}

void info_printf( char *emsg, ...)
{
	va_list ap;

	INT8U	err;
	serialMem* sm;

	va_start(ap, emsg);

	if (serialOn == 1) {
		sm = (serialMem *) OSMemGet(serialMsgMem, &err);
		if( sm != 0 ) {
			vsnprintf(sm->serialStr, serialStrSz-1,  emsg, ap);
			//if (res < 0) res = serialStrSz;   // for safety reason

			sm->serialStr[serialStrSz-1] = 0;    // for safety reason

			err = OSQPost(serialMsgTaskQ, (void *)sm);
			if ( err != OS_ERR_NONE) {
			//	do something but dont loop--->> err_printf("Q post err tickHook\n");
				OnPrintError();
			}

		} else	{  //OSMemGet
			// do somethingg ???
			OnPrintError();
		}
	} else {   // serialOn
		// might turn on an led or so
		OnPrintError();
	}
	va_end(ap);
	//	printf(emsg, ap);
}

// just for usage with short strings, otherwise sizes of buffers need to be increased
void  err_printf ( char *emsg, ...)
{
	va_list ap;

	va_start(ap, emsg);
	++ amtErr;
	info_printf(emsg, ap);
	va_end(ap);
}


/*
 * uart-comms.h
 *
 *  Created on: Oct 12, 2020
 *      Author: ixchel
 */

#ifndef ST_TEMPIX_TOOLS_UART_HW_H_
#define ST_TEMPIX_TOOLS_UART_HW_H_


#include <uosii-includes.h>

OS_EVENT *dmaQSem;

INT8U initUartHw();

INT8U startUartHw();

INT8U sendUartString(char* sndStr);

void USART1_IRQHandler(void);

INT8U enableUartInterrupts();

INT8U disableUartInterrupts();

INT32U  errMsgNdtr;
INT32U  errMsgStrLen;
INT32U  feCounter;
INT32U  teCounter;
INT32U  dmeCounter;
INT32U  msgCounter;
INT32U  rxMsgCounter;
INT32U  txMsgCounter;
INT32U  msgNr;

#endif /* ST_TEMPIX_TOOLS_UART_HW_H_ */

/*
 * uart-comms.h
 *
 *  Created on: Oct 12, 2020
 *      Author: ixchel
 */

#ifndef ST_TEMPIX_TOOLS_UART_HW_H_
#define ST_TEMPIX_TOOLS_UART_HW_H_


#include <uosii-includes.h>

#define dmaAvailable  0x01

OS_EVENT *dmaQSem;

INT8U initUartHw();

void deInitUart();

INT8U startUartHw();

INT8U sendUartString(char* sndStr);


INT8U enableUartInterrupts();

INT8U disableUartInterrupts();


INT32U  rxMsgCounter;
INT32U  txMsgCounter;

#endif /* ST_TEMPIX_TOOLS_UART_HW_H_ */

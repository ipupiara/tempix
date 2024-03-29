/*
 * uart-comms.h
 *
 *  Created on: Oct 12, 2020
 *      Author: ixchel
 */

#ifndef ST_TEMPIX_TOOLS_UART_HW_H_
#define ST_TEMPIX_TOOLS_UART_HW_H_

#define maxUartReceiveDmaStringSize  0x80

#include <uosii-includes.h>

#define dmaAvailable  0x01

INT8U initUartHw();

INT8U startUartHw();

INT8U sendUartString(char* sndStr);


INT8U enableUartInterrupts();

INT8U disableUartInterrupts();


#endif /* ST_TEMPIX_TOOLS_UART_HW_H_ */

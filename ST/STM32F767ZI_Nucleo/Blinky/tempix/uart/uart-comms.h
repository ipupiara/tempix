/*
 * uart-comms.h
 *
 *  Created on: Oct 12, 2020
 *      Author: ixchel
 */

#ifndef ST_TEMPIX_TOOLS_UART_COMMS_H_
#define ST_TEMPIX_TOOLS_UART_COMMS_H_

#include <uosii-includes.h>

INT8U  serialOn;

INT8U  commsError;

#define serialStrBufSz    7
#define serialStrSz    32*4     //  32 bit aligned
typedef struct serialMem{
  char serialStr [serialStrSz];  // ATTENTION serialStr must be at first place, so that &(serialMem) == &(serialStr)
}serialMem;
#define serialTaskQMsgSz  serialStrBufSz


serialMem currentSerialMem;

void init_printf();
void err_printf ( char *emsg, ...);
void info_printf( char *emsg, ...);
void start_printf();
void printStartMessage();



#endif /* ST_TEMPIX_TOOLS_UART_COMMS_H_ */

#ifndef  main_h_includefile
#define  main_h_includefile


#include  <uosii-includes.h>
#include  <uart-hw.h>
#include <canRelated.h>

typedef enum backGroundEvents
{
	evUartStringReceived,
	i2cReinitNeeded,
	canMessageReadyForSend,
	evThottleActorPingResponse,
	evNumberOfBackgroundEvents
} BackGroundEvents;

typedef struct  {
	uint8_t evType;
	union {
		char uartString [maxUartReceiveDmaStringSize+1];  // need a copy of these data for async handling (else disappear)
		struct {			// currently not in use
			u_int8_t  anyInt;
		} zeroEvent;
		TempixSimpleCommand canMessage;
		INT8U dummyFiller [10]; // needed for messageQueue
//		uint8_t  enumerator [];
	}  evData;

} backGroundEvent ;

uint16_t clockRelValueVsMaxClk(uint16_t valAtMax);

#define backGroundEventBufSz    7

uint8_t proceedBackGroundEvent(backGroundEvent* bgEv);

#endif

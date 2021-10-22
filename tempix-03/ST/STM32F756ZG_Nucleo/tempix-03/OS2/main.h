#ifndef  main_h_includefile
#define  main_h_includefile

#include  <uosii-includes.h>
#include  <uart-hw.h>

enum backGroundEvents
{
	evUartStringReceived,
	i2cReinitNeeded,
	evThottleActorPingResponse,
	evNumberOfBackgroundEvents
};

typedef struct  {
	uint8_t evType;
	union {
		char uartString [maxUartReceiveDmaStringSize+1];  // need a copy of these data for async handling (else disappear)
		struct {			// currently not in use
			u_int8_t  anyInt;
		} zeroEvent;
		uint8_t canData[8];
		INT8U dummyFiller [10]; // needed for messageQueue
	}  evData;

} backGroundEvent ;

#define backGroundEventBufSz    7

OS_MEM * backGroundEventMem;

OS_EVENT*  backGroundEventTaskQ;

backGroundEvent backGroundEventBuffer[backGroundEventBufSz];
void* backGroundEventPtrBuffer[backGroundEventBufSz];

void sendCanPingMessage();

#endif

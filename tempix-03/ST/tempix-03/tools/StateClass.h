
#if !defined(CCtateClassHeader)
#define CCtateClassHeader

#include "Tstatechart.h"
#include <uosii-includes.h>


enum  eEventTypes
{
	evPidIdle,
	evPidTimout,
	evAccSetPressed,
	evDecResPressed,
	evSwitchedOn,
	evSwitchedOff,
	evClutchBreakPressed,
	evErrorDetected,
	evErrorResovled,
	evFatalError
};

INT8U   workingTimerActive;

typedef struct  {
	uint8_t evType;
	union {
		uint8_t keyCode;
		struct {			// currently not in use

		} zeroAdjustingState;
		INT8U dummyFiller [10]; // needed for messageQueue
	}  evData;
} CTempixEvent ;


TStatechart TempixStateChart;
TStatechart* pTempixStateChart;

void startTempixStateChart();


void stopTempixStateChart();


bool processTempixEvent(TStatechart* t,CTempixEvent* ev);

void postTempixEvent(INT8U ev);




#endif



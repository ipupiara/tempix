
#include <stdio.h>
//#include <iostream.h>
//#include <assert.h>
#include <string.h>

#include "Tstatechart.h"
#include "StateClass.h"
#include <uart-comms.h>
#include <uosii-includes.h>
#include <pidControl.h>
#include <gpioSupport.h>
#include <servoControl.h>
#include <i2c.h>

extern const uStInt uStIntHandlingDone;
extern const uStInt uStIntNoMatch;

CTempixEvent* currentTempixEvent;

OS_EVENT *TempixStatechartResourceSema;

// This defines and names the states the class has.
#warning  attention: sequence must be the same as in xaStates (below)  !!!
enum eStates
{
	eStateTempixStateChart,
	eStartState = eStateTempixStateChart,
	eStateTempixOff,
	eStateTempixOn,
	eStateTempixReady,
	eStateTempixWorking,
	eStateTempixNotReady,
	eStateFataLError,
	eNumberOfStates
};

// todo add/evaluate a system state checker for error resolvement and ready state check

OS_TMR  * workingTimer;

OS_STK  pidThreadMethodStk[APP_CFG_DEFAULT_TASK_STK_SIZE];



#define fsmEventBuffSz  8
OS_MEM *fsmEventPtrMem;
OS_EVENT*  fsmEventQ;
CTempixEvent fsmEventBuf[fsmEventBuffSz];
void* fsmEventPtrArray[fsmEventBuffSz];




uint8_t areAllErrorsResolved()
{
	uint8_t res = 0;

	return res;
}


void workingTimerCallback( void *parg)
{
	//  just set event for next trigger
	workingTimerActive = 1;
	postTempixEvent(evPidTimout);
}

void postTempixEvent(INT8U ev)
{
	CTempixEvent* fm;
	INT8U err;

	fm = (CTempixEvent*) OSMemGet(fsmEventPtrMem, &err);
	if (fm != 0) {
		fm->evType = ev;

		if ((fm->evType == evErrorDetected) || (fm->evType == evFatalError)) {
			err = OSQPostFront(fsmEventQ, (void *)fm);
		}
		else if ((fm->evType == evErrorDetected) || (fm->evType == evFatalError)) {
			err = OSQFlush(fsmEventQ);
			if (err == OS_ERR_NONE) {
				err = OSQPost(fsmEventQ, (void *)fm);
			}
		}
		else  {
			err = OSQPost(fsmEventQ, (void *)fm);
		}
	} //  else cant do much, mostly called from any isr, ev blink an alarm
}

static  void  tempixFsmMethod (void *p_arg)
{
	INT8U err;

	CTempixEvent* ev;

     while (1) {
    	 ev = (CTempixEvent *)OSQPend(fsmEventQ, 3049   , &err);
    	 if (err == OS_ERR_NONE) {
			processTempixEvent(pTempixStateChart, ev);
			err = OSMemPut(fsmEventPtrMem, (void *)ev);
		 } else  if ((err == OS_ERR_TIMEOUT)){
			info_printf("timeout in fsm working thread");  // todo uncomment this line after debug
		 } else {
			 info_printf("error in fsm workging thread %x",err);
		 }
	 }
}

void prepareFsmUosMethod()
{
	INT8U err;
	INT32U amtPerSec = OS_TMR_CFG_TICKS_PER_SEC  / 5;
	workingTimerActive = 0;
	workingTimer = OSTmrCreate( amtPerSec,amtPerSec,OS_TMR_OPT_PERIODIC,
								(OS_TMR_CALLBACK)workingTimerCallback,NULL,
										(INT8U *)"workingTimer",&err);
	if (err != OS_ERR_NONE)  {
		info_printf("error creating workingTimer");
	}
/*	if (!(fsmTiggerUosEvent = OSSemCreate(0))){
		info_printf("could not create fsmTiggerUosEvent\n");
		err = 0x77;
	}*/

	memset(fsmEventBuf,0x00,sizeof(fsmEventBuf));
	if (err == OS_ERR_NONE) {
		fsmEventPtrMem = OSMemCreate(&fsmEventBuf[0], fsmEventBuffSz, sizeof(CTempixEvent), &err);
	}
	if (err == OS_ERR_NONE) {
		OSMemNameSet(fsmEventPtrMem, (INT8U*)"fsmMsgMem", &err);
	}
	if (err == OS_ERR_NONE) {
		fsmEventQ = OSQCreate(&fsmEventPtrArray[0], fsmEventBuffSz);
		if (! fsmEventQ) err = 0xFF;
	}


	if (err ==  OS_ERR_NONE  )  {
		err = OSTaskCreateExt(tempixFsmMethod,                                       /* Create the start task                                    */
			                    (void *)0,
			                    (OS_STK *)&pidThreadMethodStk[APP_CFG_DEFAULT_TASK_STK_SIZE - 1],
								PID_THREAD_PRIO,
								PID_THREAD_PRIO,
			                    (OS_STK *)&pidThreadMethodStk[0],
								APP_CFG_DEFAULT_TASK_STK_SIZE,
			                    (void *)0,
			                    OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

	}
	if (err ==  OS_ERR_NONE  )  {
		OSTaskNameSet(PID_THREAD_PRIO, (INT8U *)"SerQ", &err);
	}
	if (err != OS_ERR_NONE)  {
		info_printf("error creating pid workingThead or Name of working thread");
	}
}

uStInt evTempixStateChartChecker(void)
{
	uStInt res = uStIntNoMatch;
	if (currentTempixEvent->evType   == evI2CResetNeeded) {
		OSTimeDlyHMSM(0, 0, 0, 2);
		// give it some time (1-2 ms) for a hw reset (no big delay for this fsm)
		initI2c();
		res =  uStIntHandlingDone;
	}

	return res;
}

void entryTempixStateChart(void)
{
	info_printf("entering tempix statechart\n");
}


uStInt evTempixOffChecker(void)
{
	uStInt res = uStIntNoMatch;
////	printf("check for event in State evStateIdle\n");

	if (currentTempixEvent->evType   == evSwitchedOn)  //  todo add errorStatecheck
	{
		BEGIN_EVENT_HANDLER(pTempixStateChart, eStateTempixOn);
			// No event action.

		END_EVENT_HANDLER(pTempixStateChart);
		res =  uStIntHandlingDone;
	}
	if (currentTempixEvent->evType   == evErrorDetected)
	{
		BEGIN_EVENT_HANDLER(pTempixStateChart, eStateTempixNotReady);
			// No event action.

		END_EVENT_HANDLER(pTempixStateChart);
		res =  uStIntHandlingDone;
	}

	return (res);
}

void entryTempixOff(void)
{
	if (! areAllErrorsResolved()) {
		postTempixEvent(evErrorDetected);
	} else if (isTempixSwitchedOn()) {
		postTempixEvent(evSwitchedOn);
	}
}

void exitTempixOff(void)
{
	info_printf("exit TempixIdle state\n");

}


uStInt evTempixOn(void)
{
	uStInt res = uStIntNoMatch;

	if (currentTempixEvent->evType   == evSwitchedOff)  {
		BEGIN_EVENT_HANDLER(pTempixStateChart, eStateTempixOff);
			// No event action.

		END_EVENT_HANDLER(pTempixStateChart);
		res =  uStIntHandlingDone;
	}
	if (currentTempixEvent->evType   == evErrorDetected)
	{
		BEGIN_EVENT_HANDLER(pTempixStateChart, eStateTempixNotReady);
			// No event action.

		END_EVENT_HANDLER(pTempixStateChart);
		res =  uStIntHandlingDone;
	}
	return (res);
}

void entryTempixOn(void)
{
	INT8U err;
	info_printf("entering TempixRunning state\n");
	OSTmrStart(workingTimer, &err);
	if (err != OS_ERR_NONE) {
		info_printf("workingTimer not started");
	}
}

void exitTempixOn(void)
{
	INT8U err;
	info_printf("exit TempixRunning state\n");
	OSTmrStop(workingTimer,OS_TMR_OPT_NONE, NULL, &err);
	if (err != OS_ERR_NONE) {
		info_printf("workingTimer not started");
	}
}


uStInt evTempixReady(void)
{
	uStInt res = uStIntNoMatch;
	//	printf("check for event in State evStateIdle\n");

	if (currentTempixEvent->evType   == evAccSetPressed)
	{
		if ((getCurrentSpeedMPerSec() > minSpeedMPerSec) && (isClutchBreakReleased())) {

			setDesiredSpeedInvCurrentSpeedInv();
			BEGIN_EVENT_HANDLER(pTempixStateChart, eStateTempixWorking); // todo and speed...   ;
				// No event action.

			END_EVENT_HANDLER(pTempixStateChart);
		}
		res =  uStIntHandlingDone;
	}
	if (currentTempixEvent->evType   == evDecResPressed)
	{
		uint32_t  desiredSpdInv=getDesiredSpeedInv();
		float spd = getSpeedMPerSec(desiredSpdInv);   //  this way for debbugging
		if ((spd > minSpeedMPerSec) && isClutchBreakReleased()) {
			BEGIN_EVENT_HANDLER(pTempixStateChart, eStateTempixOn);
				// No event action.

			END_EVENT_HANDLER(pTempixStateChart);
		}
		res =  uStIntHandlingDone;
	}
	return (res);
}

void entryTempixReady(void)
{
	info_printf("entering TempixReady state\n");
}

void exitTempixReady(void)
{
	info_printf("exit TempixReady state\n");
}



uStInt evTempixWorking(void)
{
	uStInt res = uStIntNoMatch;

	if (currentTempixEvent->evType   == evAccSetPressed)
	{
		incSpeed();
	}
	if (currentTempixEvent->evType   == evDecResPressed)
	{
		decSpeed();
	}
	if (currentTempixEvent->evType   == evClutchBreakPressed)
	{
		BEGIN_EVENT_HANDLER(pTempixStateChart, eStateTempixReady);
			// No event action.

		END_EVENT_HANDLER(pTempixStateChart);
		res =  uStIntHandlingDone;
	}
	if (currentTempixEvent->evType   == evPidTimout)
	{
		triggerNextPid();
		res =  uStIntHandlingDone;
	}

	return (res);
}

void entryTempixWorking(void)
{
	info_printf("entering TempixWorking state\n");
}

void exitTempixWorking(void)
{

}


uStInt evTempixNotReadyChecker(void)
{
	uStInt res = uStIntNoMatch;
	//	printf("check for event in State evStateIdle\n");

	if (currentTempixEvent->evType   == evErrorResovled)
	{
		// lets the off state decide where to go, since it does this anyhow
		BEGIN_EVENT_HANDLER(pTempixStateChart, eStateTempixOff);
			// No event action.

		END_EVENT_HANDLER(pTempixStateChart);
		res =  uStIntHandlingDone;
	}
	return (res);
}

void entryTempixNotReadyState(void)
{
	info_printf("entry ErrorState state\n");
}

void exitTempixNotReadyState(void)
{
	info_printf("exit ErrorState state\n");
}


uStInt evFatalErrorChecker(void)
{
	uStInt res = uStIntNoMatch;
	//	printf("check for event in State evStateIdle\n");

	return (res);
}

void entryFatalErrorState(void)
{
	info_printf("entry fatalError state\n");
}

void exitFatalErrorState(void)
{
	info_printf("exit fatalError state\n");
}


#ifndef  sdccNULL 

#define tfNull 0

#else

t_fvoid  tfNull;

#endif 

// attention: sequence must be the same as above enum eStates



xStateType xaStates[eNumberOfStates] = {
 	{eStateTempixStateChart,    // name
		-1,									//parent
		eStateTempixOff,    // default substate
		0,    // keep history
		evTempixStateChartChecker,    // event checking fu
		tfNull,       // def state entry function
		entryTempixStateChart,     //    entering state
		tfNull},     // exiting state


 	{eStateTempixOff,
 		eStateTempixStateChart,
		-1,
		0,
		evTempixOffChecker,
		tfNull,
		entryTempixOff,
		exitTempixOff},


 	{eStateTempixOn,
 		eStateTempixStateChart,
		eStateTempixReady,
		0,
		evTempixOn,
		tfNull,
		entryTempixOn,
		exitTempixOn},

	{eStateTempixReady,
		eStateTempixOn,
	 	-1,
	 	0,
	 	evTempixReady,
	 	tfNull,
		entryTempixReady,
		exitTempixReady},


	 {eStateTempixWorking,
		eStateTempixOn,
	 	-1,
	 	0,
		evTempixWorking,
	 	tfNull,
		entryTempixWorking,
		exitTempixWorking},


 	{eStateTempixNotReady,
 		eStateTempixStateChart,
		-1,
		0,
		evTempixNotReadyChecker,
		tfNull,
		entryTempixNotReadyState,
		exitTempixNotReadyState},
	
	{eStateFataLError,
		eStateTempixStateChart,
		-1,
		0,
		evFatalErrorChecker,
		tfNull,
		entryFatalErrorState,
		exitFatalErrorState}
};




void startTempixStateChart()
{
	prepareFsmUosMethod();
#ifdef  sdccNULL 

	tfNull = (t_fvoid ) NULL;

#endif 

 	pTempixStateChart = & TempixStateChart;
	createTStatechart (& TempixStateChart, xaStates, eNumberOfStates, eStartState);
}


void stopTempixStateChart()
{
	destructTStatechart(&TempixStateChart);
}


bool processTempixEvent(TStatechart* ts,CTempixEvent* ev)
{
	INT8U err;
	currentTempixEvent = ev;
	return ProcessEvent(ts);
	return err;
}


/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*               This file is provided as an example on how to use Micrium products.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only. This file can be modified as
*               required to meet the end-product requirements.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can find our product's user manual, API reference, release notes and
*               more information at https://doc.micrium.com.
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                              uC/OS-II
*                                            EXAMPLE CODE
*
* Filename : main.c
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/
#include <main.h>


#include <uart-comms.h>
#include <uart-hw.h>
#include <i2c.h>
#include <dma-tools.h>
#include <StateClass.h>
#include <gpioSupport.h>
#include <adcControl.h>
#include <pidControl.h>
#include <servoControl.h>
#include <canComms.h>


#include  "../app_cfg.h"


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

static  OS_STK  StartupTaskStk[APP_CFG_STARTUP_TASK_STK_SIZE];





/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  StartupTask (void  *p_arg);


void initEventQ()
{
	uint8_t err= OS_ERR_NONE;
	if (err == OS_ERR_NONE) {
		backGroundEventMem = OSMemCreate(&backGroundEventBuffer[0], backGroundEventBufSz, sizeof(backGroundEvent), &err);
	}
	if (err == OS_ERR_NONE) {
		OSMemNameSet(backGroundEventMem, (INT8U*)"backGroundEventMem", &err);
	}
	if (err == OS_ERR_NONE) {
		backGroundEventTaskQ = OSQCreate(&backGroundEventPtrBuffer[0], backGroundEventBufSz);
		if (! backGroundEventTaskQ) err = 0xFF;
	}
}

uint8_t dispatchBackgroundEvent(backGroundEvent* ev)
{
	uint8_t res = OS_ERR_NONE;


	return res;
}

int  main (void)
{
#if OS_TASK_NAME_EN > 0u
    CPU_INT08U  os_err;
#endif

    HAL_Init();                                                 /* Initialize STM32Cube HAL Library                     */
    BSP_ClkInit();                                              /* Initialize the main clock                            */
    BSP_IntInit();                                              /* Initialize RAM interrupt vector table.               */
    BSP_OS_TickInit();                                          /* Initialize kernel tick timer                         */

    Mem_Init();                                                 /* Initialize Memory Managment Module                   */
    CPU_IntDis();                                               /* Disable all Interrupts                               */
    CPU_Init();                                                 /* Initialize the uC/CPU services                       */

    OSInit();                                                   /* Initialize uC/OS-II    */
    initEventQ();


    OSTaskCreateExt( StartupTask,                               /* Create the startup task                              */
                     0,
                    &StartupTaskStk[APP_CFG_STARTUP_TASK_STK_SIZE - 1u],
                     APP_CFG_STARTUP_TASK_PRIO,
                     APP_CFG_STARTUP_TASK_PRIO,
                    &StartupTaskStk[0u],
                     APP_CFG_STARTUP_TASK_STK_SIZE,
                     0u,
                    (OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));

#if OS_TASK_NAME_EN > 0u
    OSTaskNameSet(         APP_CFG_STARTUP_TASK_PRIO,
                  (INT8U *)"Startup Task",
                           &os_err);
#endif
    OSStart();                                                  /* Start multitasking (i.e. give control to uC/OS-II)   */

    while (DEF_ON) {                                            /* Should Never Get Here.                               */
        ;
    }
}


/*
*********************************************************************************************************
*                                            STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'StartupTask()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  StartupTask (void *p_arg)
{
   (void)p_arg;
   uint8_t err = OS_ERR_NONE;
   backGroundEvent *  bgEvPtr;

   BSP_OS_TickEnable();                                        /* Enable the tick timer and interrupt                  */
   BSP_LED_Init();                                             /* Initialize LEDs                                      */
//  OS_TRACE_INIT();

    init_printf();
	OSTimeDlyHMSM(0u, 0u, 1u, 0u);  // wait for uart/dma ready,  else fe happens when immediately sending a msg
	printStartMessage();

    initI2c();
    initEeprom();

//	startTempixStateChart();
//	initGpioSupport();
//	initAdc();
//	initCanComms();
//	initServoControl();
	initPid();


#if (OS_TASK_STAT_EN > 0u)
    OSStatInit();                                               /* Determine CPU capacity                               */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif

    while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */
        BSP_LED_Toggle(USER_LD1);
        if (commsError > 0) {
        	BSP_LED_Toggle(USER_LD3);
        } else {
        	BSP_LED_Off(USER_LD3);
        }
      	bgEvPtr = (backGroundEvent *)OSQPend(backGroundEventTaskQ, 1009, &err);
       	if (err == OS_ERR_NONE) {
       		switch (bgEvPtr->evType) {                /* See if we timed-out or aborted                */
				case evUartStringReceived:                         /* Extract message from TCB (Put there by QPost) */
					forwardReceivedStringBuffer(bgEvPtr->evData.uartString);
					break;
				case i2cReinitNeeded:
					OSTimeDlyHMSM(0, 0, 0, 2);
					reInitI2cAfterError();
					 break;
				default:
					info_printf("backGroundEvent %i not handled.\n",bgEvPtr->evType);
					break;
       		}
       	}
       	if (i2cInitialized == 0)  {  // poll for this here, because else i2c will no longer work
       		reInitI2cAfterError();
       	}
    }
}

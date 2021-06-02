#ifndef gpioSupportIncludeFile
#define gpioSupportIncludeFile

#include<stdint.h>
#include <math.h>
#include <uosii-includes.h>
#include "stm32f7xx_hal.h"


void initGpioSupport ();


INT8U   setAccPressed;
INT8U   resDecPressed;
INT8U   switchedOn;
INT8U	switchedOff;
INT8U   clutchBreakPressed;


INT8U isTempixSwitchedOn();
INT8U isClutchBreakReleased();


#endif

#ifndef servoConrol_H
#define servoConrol_H

#include<stdint.h>
#include <math.h>
#include <uosii-includes.h>

#define turnsToMeasure   10
void initServoControl();

INT32U  getDesiredSpeedInv();
INT32U  getCurrentSpeedInv();
void setDesiredSpeedInv(INT32U speed);
void  setCurrentSpeedInv(INT32U speed);
void setDesiredSpeedInvCurrentSpeedInv();
void setSpeedTimeout(INT8U tOut);
INT8U getSpeedTimeout();


void changeThrottlePosition(uint16_t newPos);
INT32U getThrottlePosition();

#define minSpeedMPerSec    13.9    //  approx  50 kmh
float getCurrentSpeedMPerSec();
float getSpeedMPerSec(uint32_t  speedI);

#endif

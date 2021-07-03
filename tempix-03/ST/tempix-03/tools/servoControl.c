
#include <servoControl.h>
#include <canComms.h>
#include <StateClass.h>



INT32U  desiredSpeedInv;
INT32U  currentSpeedInv;
INT8U   speedTimeout;
INT32U  throttlePosition;


/*
 * speed calculation
 *------------------
 *------------------
 * diameter of wheel  50 cm, circumpherence   157 cm approx 1.6 meter
 * ring pinion gear ratio    2.5
 *
 * timer ck_int frequency    216 mhz
 *
 * minimum speed 50 km/h = 13.8 m/s =>  Duration for 10 Turns  approx 0.45 sec
 *
 * needed values:
 * distance per pinion turn     =  diameter of wheel /  ring pinion gear ratio  =   62 cm
 * arr value
 * psc value
 *
 *  max speed  180 km/h  =  50 m/s   =   approx  80  pinion-turns/ sec
 *  pinion turn at min speed
 *  max measure time       0.5 sec		approx 22 turns
 *
 *
 *
 */

#define distancePerPinionTurn   0.62

float getSpeedMPerSec(uint32_t  speedI)
{
	float res = 0.0;
	float speedInvF = speedI;
	if (speedI >= 1 )  {
		res =  ( 1.0E+6 / speedInvF) * turnsToMeasure * distancePerPinionTurn;
	}
	return res;
}


float getCurrentSpeedMPerSec()
{
	float res = 0.0;
	INT32U speedInv = getCurrentSpeedInv();
	res = getSpeedMPerSec(speedInv);
	return res;
}

void changeThrottlePosition(uint16_t newPos)
{
	throttlePosition = newPos;
}

INT32U getThrottlePosition()
{
	return throttlePosition;
}

INT32U  getDesiredSpeedInv()
{
	CPU_SR_ALLOC();
	INT32U speed;
	CPU_CRITICAL_ENTER();
	speed = desiredSpeedInv;
	CPU_CRITICAL_EXIT();
	return speed;
}

INT32U  getCurrentSpeedInv()
{
	CPU_SR_ALLOC();
	INT32U speed;
//	CPU_CRITICAL_ENTER();
	OS_ENTER_CRITICAL();
	speed = currentSpeedInv;
//	CPU_CRITICAL_EXIT();
	OS_EXIT_CRITICAL();
	return speed;
}

void setDesiredSpeedInv(INT32U speed)
{
	CPU_SR_ALLOC();
	CPU_CRITICAL_ENTER();
	desiredSpeedInv = speed;
	CPU_CRITICAL_EXIT();
}

void setDesiredSpeedInvCurrentSpeedInv()
{
	CPU_SR_ALLOC();
	CPU_CRITICAL_ENTER();
	desiredSpeedInv = currentSpeedInv;
	CPU_CRITICAL_EXIT();
}


void  setCurrentSpeedInv(INT32U speed)
{
	CPU_SR_ALLOC();
	CPU_CRITICAL_ENTER();
	currentSpeedInv = speed;
	CPU_CRITICAL_EXIT();
}


void setSpeedTimeout(INT8U tOut)
{
	speedTimeout = tOut;
}

INT8U getSpeedTimeout()
{
	return speedTimeout;
}

void initServoControl()
{
	speedTimeout = 0;
	desiredSpeedInv = 0;
	currentSpeedInv = 0;
	throttlePosition = 0;
}

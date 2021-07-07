#if !defined(pidControlHeader)
#define pidControlHeader

#include<stdint.h>
#include <math.h>


void initEeprom();

typedef double real;

enum directions {
	up,
	down
};

uint32_t  correctionDirection;

void triggerNextPid();

void incSpeed();

void decSpeed();

void setCurrentSpeedAsGoal();

void initPid();

void resetPid();

void printPidState();

void receiveServoStateUpdate();


#endif

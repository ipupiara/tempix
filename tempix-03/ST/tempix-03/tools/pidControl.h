#if !defined(pidControlHeader)
#define pidControlHeader

#include<stdint.h>
#include <math.h>


typedef double real;

enum directions {
	up,
	down
};


void initEeprom();
void triggerNextPid();

void incSpeed();

void decSpeed();

void setCurrentSpeedAsGoal();

void initPid();

void resetPid();

void printPidState();

void receiveServoStateUpdate();


#endif


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <pidControl.h>
#include <gpioSupport.h>
#include <servoControl.h>
#include <canComms.h>
#include <i2c.h>
#include <uosii-includes.h>
#include <throttleActuator.h>

int8_t m_started;
real m_kPTot, m_kP, m_kI, m_kD, m_stepTime, m_error_thresh;   // persistent values to save on eeprom
real  m_integral, m_prev_error, m_inv_stepTime, corrCarryOver;    //  non persistent values

// EEpromPart  todo move this later into a new file... bad luck with this
//  eclipse linkage diddle doodle almost impossible without fanatic knowlegde of this
//  eclipse thing

OS_EVENT *i2cTransactionSem;

enum valuesToSave
{
	kPTot,
	kP,
	kI,
	kD,
	stepTime,
	error_thresh,
	amtEepromAccessors
};

typedef struct  {
	uint8_t index;
	uint8_t startPos;
	uint8_t len;
} eepromAccessor ;


#define lenOfReal  14

const eepromAccessor eepromAx[amtEepromAccessors] = {
    {kPTot, 0,lenOfReal},
    {kP, 1 * lenOfReal,lenOfReal},
	{kI, 2* lenOfReal, lenOfReal },
	{kD ,3 * lenOfReal ,lenOfReal },
	{stepTime, 4 * lenOfReal, lenOfReal },
	{error_thresh, 5 * lenOfReal,lenOfReal }
};

uint8_t  setEepromAddress(INT8U adr,INT8U memAdr)
{
	uint8_t res = 0;
	uint8_t byteArr [1];
	byteArray[0] = memAdr;
	res = sendI2cByteArray(adr, &byteArr[0], 1, 0);
	if (res == 0) {
		pollForReady(adr, 0);  // todo test if this is even needed here, without something to write
	}
	return res;
}

INT8U transmitEepromByteArray(INT8U adr,INT8U memAdr, INT8U* pString,INT8U amtChars, unit8_t doStore)
{
	uint8_t res = 0;
	uint8_t semErr;

	OSSemPend(i2cTransactionSem, 2803, &semErr);
	if (semErr == OS_ERR_NONE) {
		res = setAddress(adr,memAdr);
		if (res == 0) {
			if (doStore) {
				res = sendI2cByteArray(adr, &byteArr[0], 1, 0);
			}  else {
				res= receiveI2cByteArray(adr, pString, amtChars);
			}
		}

	}  else {
		res = semErr;
		res = OSSemPost(i2cTransactionSem);
	}

	return res;
}

uint8_t loadEepromByteArray(INT8U adr,INT8U memAdr, INT8U* pString,INT8U amtChars)
{
	uint8_t res = 0;

	return res;
}

uint8_t storeReal(real val,uint8_t ind)
{
	uint8_t res = 0;
	uint8_t  realStr[lenOfReal + 1];
	memset(realStr,0,sizeof(realStr));
	snprintf((char *)realStr, lenOfReal , "%e", val);
	---- res = sendI2cByteArray(eepromAx[ind].startPos,realStr,eepromAx[ind].len, 5);  //  todo this is wrong, correct
	if (res != 0) {}
	return res;
}

uint8_t restoreReal(uint8_t ind, real* result)
{
	*result = 0.0;
	uint8_t  realStr[lenOfReal + 1];
	uint8_t err = OS_ERR_NONE;
	uint8_t endPtr;

	memset(realStr,0,sizeof(realStr));
	err = receiveI2cByteArray(eepromAx[ind].startPos,&realStr[0],eepromAx[ind].len);
	if (err != 0) {
		------*result =  strtod((const char*) &realStr[0],(char **) &endPtr);

	    if (res == 0) {
	        if (errno == ERANGE) {}  //  tobe tested, in our case, error should be contained in endPtr, which
	        						//  may not have a valid value (eg. 0, not a valid address)
	    }else {
			storeReal(1.0, ind);

	    }
	}
	return res;
}



void initEeprom()
{
	uint8_t err = OS_ERR_NONE;
	if (err == OS_ERR_NONE) {
		 i2cTransactionSem = OSSemCreate(1);
	}
}

//  end eeprom part


void restorePersistentValues()
{
	m_kPTot = restoreReal(kPTot);
	m_kP = restoreReal(kP);
	m_kI = restoreReal(kI);
	m_kD = restoreReal(kD);
	m_stepTime = restoreReal(stepTime);
	m_error_thresh = restoreReal(error_thresh);
}



uint32_t   desiredSpeedInv, actualSpeedInv;

uint8_t    resetOnError;

#define correctionThreshold  30

// todo implement eeprom save restore of parameters and setting parameters via comms (uart...)

void sendMessageToServoControl(int32_t corrInt)
{
	changeThrottlePos(corrInt);
}

void InitializePID()
{
    // Initialize controller parameters
	// PN 3.Oct 2011, added m_kP for better setting of proportional factor only
	// though these 4 factors will be linearly dependent
	restorePersistentValues();

    m_inv_stepTime = 1 / m_stepTime;
    m_integral = 0;
    m_started = 0;
	corrCarryOver = 0;
}



real nextCorrection(real error)
{
    // Set q_fact to 1 if the error magnitude is below
    // the threshold and 0 otherwise
    real q_fact;
	real res;
    if (fabs(error) < m_error_thresh)
        q_fact = 1.0;
    else  {
        q_fact = 0.0;
		m_integral = 0.0;
	}

    // Update the error integral
    m_integral += m_stepTime*q_fact*error;

    // Compute the error derivative
    real deriv;
    if (!m_started)  {
        m_started = 1;
        deriv = 0;
    }
    else
        deriv = (error - m_prev_error) * m_inv_stepTime;

    m_prev_error = error;

    // Return the PID controller actuator command
	res = m_kPTot*(m_kP*error + m_kI*m_integral + m_kD*deriv);
	if (res > correctionThreshold) {
		res = correctionThreshold;
	} else if (res < -1*correctionThreshold) {
		res = -1* correctionThreshold;
	}

#ifdef printfPID
	double errD = error;
	double intD = m_integral;
	double derivD = deriv;
	printf("err %f int %f deriv %f \n",errD, intD, derivD);
#endif
    return res;
}

void calcNextTriacDelay()
{
	float err;
	float corr;
	int32_t newPos;
	int32_t corrInt;
	err = getCurrentSpeedInv()  - getDesiredSpeedInv() ;
	corr = nextCorrection(err) + corrCarryOver;
	corrInt = corr;
	corrCarryOver = corr - corrInt;
	newPos = getThrottlePosition() + corrInt;
	changeThrottlePosition(newPos);
	sendMessageToServoControl(corrInt); // changeThrottlePosition(newPos);
#ifdef printfPID
	double corrD = corr;
	double carryCorrD = corrCarryOver;
	double ampsD  = currentAmps();
	printf(" corr %f corrI %i cry %f delay %x  amps %f\n",corrD,corrInt, carryCorrD, newPos, ampsD);
#endif
}

void initPid()
{
	InitializePID();
}

void triggerNextPid()
{

}

void incSpeed()
{

}
void decSpeed()
{

}

void setCurrentSpeedAsGoal()
{

}

void resetPid()
{
	corrCarryOver = 0;
 	m_integral =0;
	m_prev_error = 0;
}

void printPidState()
{
//	int16_t adcAmps;
//	float res;
//	double resD;
//	double gradD = gradAmps;
//
//	adcAmps = 0;
//
//	res = calibLowAmps +  (gradAmps * ((int16_t) adcAmps - (int16_t) calibLowADC  ));
//	resD = res;
//
//	printf("\nPID State\n");
//	printf("calLowA %i calHighA %i caLowDelay %i caHiDelay %i\n",calibLowAmps,calibHighAmps, calibLowTriacFireDuration, calibHighTriacFireDuration);
//	printf("calLowAdc %i caHiAdc %i \n",calibLowADC, calibHighADC);
//	printf("shows at 0 ADC : %f A  grad %f zeroPotiPos %i\n",resD, gradD,zeroPotiPos);
//	checkEEPOROM();
}

void receiveServoStateUpdate()
{

}


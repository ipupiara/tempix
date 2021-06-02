
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <pidControl.h>
#include <gpioSupport.h>
#include <servoControl.h>
#include <canComms.h>
#include <uosii-includes.h>
#include <throttleActuator.h>

int8_t m_started;
real m_kPTot, m_kP, m_kI, m_kD, m_stepTime, m_inv_stepTime, m_prev_error, m_error_thresh, m_integral;

real corrCarryOver;

uint32_t   desiredSpeedInv, actualSpeedInv;

#define correctionThreshold  30

// todo implement eeprom save restore of parameters and setting parameters via comms (uart...)

void sendMessageToServoControl(int32_t corrInt)
{
	changeThrottlePos(corrInt);
}

void InitializePID(real kpTot,real kpP, real ki, real kd, real error_thresh, real step_time)
{
    // Initialize controller parameters
	// PN 3.Oct 2011, added m_kP for better setting of proportional factor only
	// though these 4 factors will be linearly dependent
	m_kP   = kpP;
    m_kPTot = kpTot;
    m_kI = ki;
    m_kD = kd;
    m_error_thresh = error_thresh;

    // Controller step time and its inverse
    m_stepTime = step_time;
    m_inv_stepTime = 1 / step_time;

    // Initialize integral and derivative calculations
    m_integral = 0;
    m_started = 0;

//	 updateGradAmps();

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
//	InitializePID(real kpTot, real kpP, real ki, real kd, real error_thresh, real step_time);
//	InitializePID( -0.45, 1.1, 0.2, 0.2, 5, (pidStepDelays/42.18));

//	stableZeroAdjReached = 0;
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


#ifndef i2c_header
#define i2c_header

#include <uosii-includes.h>

#define i2cUseDma

/*
 *
 *    ATTENTION: i2c, in the current implementation, may only be called from kernel aware code.
 * 		(as nothing else is required as per today   (PN 7 Jul. 2021)
 *
 */
uint8_t resetOnError;

uint8_t pollForReady(INT8U adr, uint8_t delay);
INT8U sendI2cByteArray(INT8U adr,INT8U* pString,INT8U amtChars, uint8_t delayMs);
INT8U receiveI2cByteArray(INT8U adr,INT8U* pResultString,INT8U amtChars);

INT8U i2cInitialized;
INT8U initI2c();
void reInitI2cAfterError();
void enableI2c();

#endif

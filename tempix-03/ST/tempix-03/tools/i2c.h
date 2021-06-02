#ifndef i2c_header
#define i2c_header

#include <uosii-includes.h>

#define i2cUseDma


INT8U sendI2cByteArray(INT8U adr,INT8U* pString,INT8U amtChars);
//
INT8U receiveI2cByteArray(INT8U adr,INT8U* pResultString,INT8U amtChars);


INT8U initI2c();
void startI2c();

#endif

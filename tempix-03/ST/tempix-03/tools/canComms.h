#ifndef canComms_h
#define canComms_h
#include <stdint.h>
#include <uosii-includes.h>
#include <canRelated.h>

void initCanComms();


uint8_t syncSendTempixSimpleCommand(uint32_t sId, TempixSimpleCommand scmd);

#endif

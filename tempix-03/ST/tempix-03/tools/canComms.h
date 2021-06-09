#ifndef canComms_h
#define canComms_h
#include <stdint.h>
#include <uosii-includes.h>
#include <canRelated.h>

void initCanComms();


void sendSyncTempixCanMessage(uint32_t sId, TempixSimpleCommand scmd);

#endif

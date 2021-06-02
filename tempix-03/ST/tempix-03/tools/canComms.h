#ifndef canComms_h
#define canComms_h
#include <stdint.h>
#include <uosii-includes.h>

void initCanComms();



// lowest 11 bist allowed, ie.  Masked against  0000 07FF
#define thottleActorCommandMessage   0x00000101

#define thottleActorStateMessage	 0x00000201


//typedef struct
//{
//	uint32_t   canId;
//	uint8_t     message [8];
//}  TempixCanMessage;

typedef struct
{
	uint32_t   commandId;
	uint32_t   commandType;
	uint32_t   commandData;
}  TempixSimpleCommand;

void sendSyncTempixCanMessage(uint32_t sId, TempixSimpleCommand scmd);

#endif

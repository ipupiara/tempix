#ifndef canRelated_h
#define canRelated_h
#include <stdint.h>


// lowest 11 bits allowed, ie.  Masked against  0000 07FF
#define controllerCommandMessage   0x00000101
#define controllerPingMessage      0x00000121

#define thottleActorStateMessage	 0x00000201
#define thottleActorAlarmMessage	 0x00000210
#define thottleActorPingMessage		 0x00000221


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

void Install_Error_Handler();

#endif

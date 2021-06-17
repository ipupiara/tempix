/*
 * canComms.c
 *
 *  Created on: Jun 11, 2021
 *      Author: peetz
 */

#include <canActuatorComms.h>
#include <canRelated.h>


void respondPingRequest(uint8_t aData[])
{

}


void dispatchCanMessage( CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
	if (isCanMessageType(pHeader,controllerPingRequest)) {
		respondPingRequest(aData);
	}
}


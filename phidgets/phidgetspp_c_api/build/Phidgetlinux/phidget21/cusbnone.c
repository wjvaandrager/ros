/*
 *  no usb support
 *
 *  Created by Patrick McNeil on 06/06/05.
 *  Copyright 2005 Phidgets Inc. All rights reserved.
 *
 */
 
#include "stdafx.h"
#include "cusb.h"

#ifdef _MACOSX
int CPhidgetManager_setupNotifications(CFRunLoopRef runloop) 
{
	return EPHIDGET_UNSUPPORTED;
}

int CPhidgetManager_teardownNotifications()
{
	return EPHIDGET_UNSUPPORTED;
}

int reenumerateDevice(CPhidgetHandle phid)
{
	return EPHIDGET_UNSUPPORTED;
}
#endif

int CUSBCloseHandle(CPhidgetHandle phid)
{
	return EPHIDGET_UNSUPPORTED;
}

int CUSBSendPacket(CPhidgetHandle phid, unsigned char *buffer)
{
	return EPHIDGET_UNSUPPORTED;
}

int CUSBSetLabel(CPhidgetHandle phid, char *buffer)
{
	return EPHIDGET_UNSUPPORTED;
}

/* Buffer should be at least 8 bytes long */
int CUSBReadPacket(CPhidgetHandle phid, unsigned char *buffer)
{
	return EPHIDGET_UNSUPPORTED;
}

int CUSBBuildList(CPhidgetList **curList)
{
	return EPHIDGET_UNSUPPORTED;
}

void CUSBCleanup() {}

int CUSBOpenHandle(CPhidgetHandle phid)
{
	return EPHIDGET_UNSUPPORTED;
}



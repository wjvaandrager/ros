/*
 *  cusbmac.cpp
 *  mac-usb
 *
 *  Created by Patrick McNeil on 06/06/05.
 *  Copyright 2005 Phidgets Inc. All rights reserved.
 *
 */
 
#include "stdafx.h"
#include "cusb.h"
#include "cphidgetmanager.h"
#include "macusb.h"

/* CUSB is meant to be platform dependent */
int CUSBCloseHandle(CPhidgetHandle phid)
{
	if (!phid)
		return EPHIDGET_INVALIDARG;
		
	CPhidget_clearStatusFlag(&phid->status, PHIDGET_ATTACHED_FLAG, &phid->lock);
	if (phid->deviceHandle == NULL)
		return EPHIDGET_NOTATTACHED;
		
	//TODO: somehow cancel reads
	
	IOUSBInterfaceInterface **intf = NULL;
	intf = (IOUSBInterfaceInterface**)phid->deviceHandle;
	
	if(kIOReturnNoDevice == ((*intf)->USBInterfaceClose(intf))) {
		(void) (*intf)->Release(intf);
		return EPHIDGET_NOTATTACHED;
	}
	(void) (*intf)->Release(intf);
	
	CThread_join(&phid->readThread);

	phid->deviceHandle = NULL;
	
	return EPHIDGET_OK;
}

int CUSBSendPacket(CPhidgetHandle phid, unsigned char *buffer)
{
	unsigned long BytesWritten = 0;
    IOReturn			kr;
    IOUSBDevRequest 		request;
	IOUSBInterfaceInterface **dev = NULL;
	
	dev = (IOUSBInterfaceInterface**)phid->deviceHandle;

	if (!CPhidget_statusFlagIsSet(phid->status, PHIDGET_ATTACHED_FLAG)
		&& !CPhidget_statusFlagIsSet(phid->status, PHIDGET_ATTACHING_FLAG))
		return EPHIDGET_NOTATTACHED;
    
    request.bmRequestType = USBmakebmRequestType(kUSBOut, kUSBClass, kUSBInterface);
    request.bRequest = kUSBRqSetConfig;
    request.wValue = 0x0200;
    request.wIndex = Phid_Device_Def[phid->Phid_Device_Def_index].pdd_iid;
    request.wLength = phid->outputReportByteLength;
    request.pData = buffer;

	kr = (*dev)->ControlRequest(dev, 0, &request);
	switch(kr)
	{
		case kIOReturnSuccess:
			break;
			
		case kIOReturnNoDevice:
			LOG(PHIDGET_LOG_ERROR, "ControlRequest returned kIOReturnNoDevice: there is no connection to an IOService.");
			return EPHIDGET_UNEXPECTED;
		case kIOReturnNotOpen:
			LOG(PHIDGET_LOG_ERROR, "ControlRequest returned kIOReturnNotOpen: the interface is not open for exclusive access.");
			return EPHIDGET_UNEXPECTED;
		case kIOUSBTransactionTimeout:
			LOG(PHIDGET_LOG_ERROR, "ControlRequest returned kIOUSBTransactionTimeout: A timeout occured.");
			return EPHIDGET_UNEXPECTED;
		case kIOUSBPipeStalled:
			LOG(PHIDGET_LOG_ERROR, "ControlRequest returned kIOUSBPipeStalled: The control pipe stalled on a write - this needs to be cleared.");
			return EPHIDGET_UNEXPECTED;
		case kIOReturnNotResponding:
			LOG(PHIDGET_LOG_INFO, "ControlRequest returned kIOReturnNotResponding. Unplugged?");
			return EPHIDGET_UNEXPECTED;
		case kIOUSBEndpointNotFound:
			LOG(PHIDGET_LOG_INFO, "ControlRequest returned kIOUSBEndpointNotFound.");
			return EPHIDGET_UNEXPECTED;
			
		default:
			/* Sometimes there is an unexpected return value - why??? */
			LOG(PHIDGET_LOG_ERROR, "ControlRequest returned an unexpected value: %x",kr);
			return EPHIDGET_UNEXPECTED;
	}
	
	BytesWritten = request.wLenDone;

	if (!kr && BytesWritten == phid->outputReportByteLength) return EPHIDGET_OK;
	else 
	{
		LOG(PHIDGET_LOG_ERROR, "Wrong number of Bytes written in CUSBSendPacket: %d", BytesWritten);
		return EPHIDGET_UNEXPECTED;
	}
}


int CUSBSetLabel(CPhidgetHandle phid, char *buffer) {
	unsigned long BytesWritten = 0;
    IOReturn			kr;
    IOUSBDevRequest 		request;
	IOUSBInterfaceInterface **dev = NULL;
	
	dev = (IOUSBInterfaceInterface**)phid->deviceHandle;

	if (!CPhidget_statusFlagIsSet(phid->status, PHIDGET_ATTACHED_FLAG)
		&& !CPhidget_statusFlagIsSet(phid->status, PHIDGET_ATTACHING_FLAG))
		return EPHIDGET_NOTATTACHED;
    
    request.bmRequestType = USBmakebmRequestType(kUSBOut, kUSBStandard, kUSBDevice);
    request.bRequest = kUSBRqSetDescriptor;
    request.wValue = 0x0304;
    request.wIndex = 0x0409;
    request.wLength = buffer[0];
    request.pData = buffer;

	kr = (*dev)->ControlRequest(dev, 0, &request);
	if(kr) {
		return EPHIDGET_UNEXPECTED;
	}
	
	BytesWritten = request.wLenDone;

	if (!kr && BytesWritten == buffer[0]) return EPHIDGET_OK;
	else return EPHIDGET_UNEXPECTED;
}

/* Buffer should be at least 8 bytes long */
int CUSBReadPacket(CPhidgetHandle phid, unsigned char *buffer)
{

	IOReturn kr;
	IOUSBInterfaceInterface **intf = NULL;
	intf = (IOUSBInterfaceInterface**)phid->deviceHandle;
	UInt32 BytesRead = phid->inputReportByteLength;

	if (!CPhidget_statusFlagIsSet(phid->status, PHIDGET_ATTACHED_FLAG)
		&& !CPhidget_statusFlagIsSet(phid->status, PHIDGET_ATTACHING_FLAG))
		return EPHIDGET_NOTATTACHED;
		
	kr = (*intf)->ReadPipe(intf, 1, buffer, &BytesRead);
	switch(kr)
	{
		case kIOReturnSuccess:
			break;
		case kIOReturnNoDevice:
			LOG(PHIDGET_LOG_ERROR, "ReadPipe returned kIOReturnNoDevice: there is no connection to an IOService.");
			return EPHIDGET_UNEXPECTED;
		case kIOReturnNotOpen:
			LOG(PHIDGET_LOG_ERROR, "ReadPipe returned kIOReturnNotOpen: the interface is not open for exclusive access.");
			return EPHIDGET_UNEXPECTED;
		case kIOReturnNotResponding:
			LOG(PHIDGET_LOG_ERROR, "ReadPipe returned kIOReturnNotResponding: This probably will only happen while hardware debugging.");
			return EPHIDGET_UNEXPECTED;
		case kIOReturnAborted:
			LOG(PHIDGET_LOG_INFO, "ReadPipe returned kIOReturnAborted: This probably means that close was called.");
			return EPHIDGET_INTERRUPTED;
		default:
			/* Sometimes there is an unexpected return value - why?? */
			LOG(PHIDGET_LOG_ERROR, "ReadPipe returned an unexpected value: %x",kr);
			return EPHIDGET_UNEXPECTED;
	}

	if (phid->inputReportByteLength != BytesRead) 
	{
		LOG(PHIDGET_LOG_ERROR, "Wrong number of Bytes read: %d",BytesRead);
		return EPHIDGET_UNEXPECTED;
	}
	
	return EPHIDGET_OK;
}

int CUSBBuildList(CPhidgetList **curList)
{
	return EPHIDGET_OK;
}

void CUSBCleanup() {}

int CUSBOpenHandle(CPhidgetHandle phid)
{
    kern_return_t			err;
    CFMutableDictionaryRef 	matchingDictionary = 0;
    SInt32					idVendor;
    SInt32					idProduct;
    CFNumberRef				numberRef;
    io_iterator_t			iterator = 0;
	mach_port_t				masterPort = 0;
    io_service_t			usbDevice;
	IOUSBInterfaceInterface **intf = NULL;
		
	int i,ret = EPHIDGET_UNEXPECTED;
	
	for (i = 1; i<PHIDGET_DEVICE_COUNT; i++)
	{
		if (Phid_Device_Def[i].pdd_did == phid->deviceID) 
		{
			idVendor = Phid_Device_Def[i].pdd_vid;
			idProduct = Phid_Device_Def[i].pdd_pid;
			phid->deviceIDSpec = Phid_Device_Def[i].pdd_sdid;
			phid->Phid_Device_Def_index=i;

			//Master port for talking to the IO kit
			err = IOMasterPort(MACH_PORT_NULL, &masterPort);				
			if (err) {
				LOG(PHIDGET_LOG_WARNING, "Got error from IOMasterPort: %d",err);
				return ret;
			}
			//create a matching dictionary for the device
			matchingDictionary = IOServiceMatching(kIOUSBDeviceClassName);
			if (!matchingDictionary) return ret;
			//add Vendor ID
			numberRef = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &idVendor);
			if (!numberRef) return ret;
			CFDictionaryAddValue(matchingDictionary, CFSTR(kUSBVendorID), numberRef);
			CFRelease(numberRef);
			numberRef = 0;
			//add Product ID
			numberRef = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &idProduct);
			if (!numberRef) return ret;
			CFDictionaryAddValue(matchingDictionary, CFSTR(kUSBProductID), numberRef);
			CFRelease(numberRef);
			numberRef = 0;
			//Get the matching devices from the IO registry
			err = IOServiceGetMatchingServices(masterPort, matchingDictionary, &iterator);
			matchingDictionary = 0;			// this was consumed by the above call
			
			//iterate through matching devices
			while ( (usbDevice = IOIteratorNext(iterator)) )
			{
				ret = (getDevice(phid, usbDevice));
				if(ret==EPHIDGET_OK) {

					//actually open the device here...
					intf = (IOUSBInterfaceInterface**)phid->deviceHandle;
					if (kIOReturnSuccess != (*intf)->USBInterfaceOpen(intf))
					{
						(void) (*intf)->Release(intf);
						ret = EPHIDGET_UNEXPECTED;
					}
					else {
						return EPHIDGET_OK;
					}
				}
			}
		}
	}
	return ret;
}



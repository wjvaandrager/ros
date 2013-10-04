#include "StdAfx.h"
#include "cusb.h"

#include <windows.h>				// For all that Windows stuff
#include <winioctl.h>				// Needed for CTLCODE macro
#include <Winbase.h>

#include "../../phidget/phidget/PhidgetSDK.h"				// Phidget IOCTLs and structures

/* CUSB is meant to be platform dependent */

int CUSBCloseHandle(CPhidgetHandle phid)
{
	TESTPTR(phid)
	CPhidget_clearStatusFlag(&phid->status, PHIDGET_ATTACHED_FLAG, &phid->lock);
	if (phid->deviceHandle == INVALID_HANDLE_VALUE) return EPHIDGET_NOTATTACHED;

	CloseHandle(phid->deviceHandle);

    phid->deviceHandle = INVALID_HANDLE_VALUE;

	/*LOG(PHIDGET_LOG_DEBUG,"Shut everything down");*/
	return EPHIDGET_OK;
}

int CCONV CUSBSendPacket(CPhidgetHandle phid, unsigned char *buffer)
{
	unsigned long BytesWritten = 0;

	//LOG(PHIDGET_LOG_DEBUG,"Writing: %s", phid->deviceType);

	if (!CPhidget_statusFlagIsSet(phid->status, PHIDGET_ATTACHED_FLAG)
		&& !CPhidget_statusFlagIsSet(phid->status, PHIDGET_ATTACHING_FLAG))
		return EPHIDGET_NOTATTACHED;

	if (phid->deviceHandle == INVALID_HANDLE_VALUE)
	{
		LOG(PHIDGET_LOG_DEBUG,"Handle for writing is not valid");
		return EPHIDGET_UNEXPECTED;
	}

	if (!WriteFile((void *)phid->deviceHandle, buffer, phid->outputReportByteLength, &BytesWritten, NULL))
	{
		switch(GetLastError())
		{
		case ERROR_IO_PENDING:
			break;
		case ERROR_IO_INCOMPLETE:
			return EPHIDGET_INTERRUPTED;
		case ERROR_DEVICE_NOT_CONNECTED:
		case 1609:
			//LOG(PHIDGET_LOG_DEBUG,"Device Not Connected");
			return EPHIDGET_NOTATTACHED;
		default:
			LOG(PHIDGET_LOG_DEBUG,"Unknown error returned from Writefile - %d", GetLastError());
			return EPHIDGET_UNEXPECTED;
		}
	}
 
	if (BytesWritten == phid->outputReportByteLength) return EPHIDGET_OK;
	LOG(PHIDGET_LOG_DEBUG,"Failure in CUSBSendPacket Report Length - %d Bytes Written - %d", phid->outputReportByteLength, BytesWritten);
	return EPHIDGET_UNEXPECTED;
}

int CUSBSetLabel(CPhidgetHandle phid, char *string){
	PHIDDEV_STRING labelData;
	BOOL f;
	DWORD dwBytes;

	wcsncpy(labelData.string+1, (const WCHAR *)string+1, string[0]);
	labelData.string[0] = string[0] | (string[1] << 8);
	labelData.size = string[0];

	f = DeviceIoControl (phid->deviceHandle, IOCTL_PHIDGET_DEVICE_SET_TAGSTRING, &labelData, sizeof (PHIDDEV_STRING), 
		                     0, 0, &dwBytes, NULL);

	if(!f) {
		LOG(PHIDGET_LOG_DEBUG,"IOCTL_PHIDGET_DEVICE_SET_TAGSTRING Failed, %d", GetLastError());  
		return EPHIDGET_UNEXPECTED;
	}
	return EPHIDGET_OK;
}

int CCONV CUSBReadPacket(CPhidgetHandle phid, unsigned char *buffer)
{
	unsigned long BytesRead = 0;
	int result;

	if (phid->deviceHandle == INVALID_HANDLE_VALUE)
		return EPHIDGET_UNEXPECTED;

	if (CPhidget_statusFlagIsSet(phid->status, PHIDGET_ATTACHED_FLAG)
		|| CPhidget_statusFlagIsSet(phid->status, PHIDGET_ATTACHING_FLAG))
	{
		if (!(result = ReadFile(phid->deviceHandle, buffer, phid->inputReportByteLength,&BytesRead,NULL)))
		{
			switch(GetLastError())
			{
			case ERROR_IO_PENDING:         
				break;
			case ERROR_DEVICE_NOT_CONNECTED:
			case ERROR_INVALID_HANDLE_STATE:
				return EPHIDGET_NOTATTACHED;
			case ERROR_IO_INCOMPLETE:
				return EPHIDGET_INTERRUPTED;
			default:
				LOG(PHIDGET_LOG_DEBUG,"Unknown Error - %d Result - %d", GetLastError(),result);
				return EPHIDGET_UNEXPECTED;
			}
		}

		if (phid->inputReportByteLength != BytesRead) 
		{
			LOG(PHIDGET_LOG_DEBUG,"Failure in CUSBReadPacket Report Length - %d Bytes Read - %d", phid->inputReportByteLength, BytesRead);
			return EPHIDGET_UNEXPECTED;
		}

		return EPHIDGET_OK;
	}
	else return EPHIDGET_NOTATTACHED;
}

/* This function needs error checking */
int CUSBGetDeviceCapabilities(CPhidgetHandle phid, HANDLE dhandle)
{
	int i;
	PHIDDEV_STRING SerialNumber;
	PHIDDEV_STRING labelData;
	int				serNum;
	PHIDDEV_ATTRIBUTES attributes;
	BOOL f;
	DWORD dwBytes;

	f = DeviceIoControl (dhandle, IOCTL_PHIDGET_DEVICE_GET_ATTRIBUTES, 0, 0, 
		                     &attributes, sizeof(attributes), &dwBytes, NULL);

	if(!f) {
		LOG(PHIDGET_LOG_DEBUG,"IOCTL_PHIDGET_DEVICE_GET_ATTRIBUTES Failed, %d", GetLastError());  
	}

	phid->inputReportByteLength = attributes.InputReportByteLength;
	phid->outputReportByteLength = attributes.OutputReportByteLength;
	phid->ProductID = attributes.ProductID;
	phid->VendorID = attributes.VendorID;
	if (attributes.VersionNumber < 0x100) 
		phid->deviceVersion = attributes.VersionNumber * 100;
	else 
		phid->deviceVersion = ((attributes.VersionNumber >> 8) * 100) + ((attributes.VersionNumber & 0xff));

	f = DeviceIoControl (dhandle, IOCTL_PHIDGET_DEVICE_GET_SERIALNUMBERSTRING, 0, 0, 
		                     &SerialNumber, sizeof(SerialNumber), &dwBytes, NULL);
	if(!f) {
		LOG(PHIDGET_LOG_DEBUG,"IOCTL_PHIDGET_DEVICE_GET_SERIALNUMBERSTRING Failed, %d", GetLastError());  
	}
	f = DeviceIoControl (dhandle, IOCTL_PHIDGET_DEVICE_GET_TAGSTRING, 0, 0, 
		                     &labelData, sizeof(labelData), &dwBytes, NULL);
	if(!f) {
		switch(GetLastError()) {
			//this device does not support labels
			case ERROR_NOT_SUPPORTED:
				phid->label[0] = '\0';
				break;
			default:
				LOG(PHIDGET_LOG_DEBUG,"IOCTL_PHIDGET_DEVICE_GET_TAGSTRING Failed, %d", GetLastError()); 
		} 
	}
	else
	{
		for(i=0;i<10;i++)
			phid->label[i] = (char)labelData.string[i];
		phid->label[10] = '\0';
	}

	swscanf((wchar_t *)(SerialNumber.string), L"%d", &serNum);

	if (phid->specificDevice != FALSE)
		if (phid->serialNumber != serNum)
			return EPHIDGET_NOTFOUND;
	phid->serialNumber = serNum;

	return EPHIDGET_OK;
}


int CUSBBuildList(CPhidgetList **curList)
{
	HANDLE								hSearch;
	HANDLE								DeviceHandle;
	PDEVMGR_DEVICE_INFORMATION			devInfo;
	int Result;
	int									MemberIndex, i, found;
	unsigned long						Length;
	char * interface_spot = 0;
	PHIDDEV_ATTRIBUTES attributes;
	BOOL f;
	DWORD dwBytes;

	CPhidgetList *traverse;
	CPhidgetHandle phid;

	Length = 0;
	MemberIndex = 0;
	
	devInfo = (PDEVMGR_DEVICE_INFORMATION)malloc(sizeof(DEVMGR_DEVICE_INFORMATION));
	memset(devInfo, 0, sizeof(DEVMGR_DEVICE_INFORMATION));
	devInfo->dwSize = sizeof(DEVMGR_DEVICE_INFORMATION);
	hSearch = FindFirstDevice(DeviceSearchByDeviceName, L"PHD*", devInfo);
	if(hSearch == INVALID_HANDLE_VALUE)
	{
		switch(GetLastError())
		{
			case ERROR_NO_MORE_FILES:
				break;
			default:
				LOG(PHIDGET_LOG_DEBUG,"FindFirstDevice Failed - INVALID_HANDLE_VALUE, %x", GetLastError());
		}
	}
	else
	{
		do {
			found = 0;
			if (AttachedDevices) {
				for (traverse = AttachedDevices; traverse;
					traverse=traverse->next) {
					if (traverse->phid->CPhidgetFHandle == devInfo->hDevice) {
						CList_addToList((CListHandle *)curList, traverse->phid, CPhidget_areExtraEqual);
						found = 1;
					}
				}
			}
			if (!found)
			{
				DeviceHandle = CreateFile (devInfo->szLegacyName, 0,
					FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);

				if(DeviceHandle == INVALID_HANDLE_VALUE)  
				{
					LOG(PHIDGET_LOG_DEBUG,"CreateFile - INVALID_HANDLE_VALUE, %x", GetLastError());
					continue;
				}

				f = DeviceIoControl (DeviceHandle, IOCTL_PHIDGET_DEVICE_GET_ATTRIBUTES, 0, 0, 
										 &attributes, sizeof(attributes), &dwBytes, NULL);
				if(!f) {
					LOG(PHIDGET_LOG_DEBUG,"IOCTL_PHIDGET_DEVICE_GET_ATTRIBUTES Failed, %d", GetLastError());  
				}


				for (i = 1; i<PHIDGET_DEVICE_COUNT; i++) 
				{
					if ((attributes.VendorID == Phid_Device_Def[i].pdd_vid)
						&& (attributes.ProductID == Phid_Device_Def[i].pdd_pid)
						&& (attributes.InterfaceNumber == Phid_Device_Def[i].pdd_iid))
					{
						if((Result = CPhidget_create(&phid))) return Result;

						LOG(PHIDGET_LOG_DEBUG,"new device found");
						phid->deviceType = Phid_DeviceName[Phid_Device_Def[i].pdd_did];

						CPhidget_setStatusFlag(&phid->status, PHIDGET_ATTACHED_FLAG, &phid->lock);
						phid->deviceIDSpec = Phid_Device_Def[i].pdd_sdid;
						phid->deviceID = Phid_Device_Def[i].pdd_did;
						phid->Phid_Device_Def_index = i;

						phid->CPhidgetFHandle = devInfo->hDevice;
						
						phid->serialNumber = -1;
						phid->deviceHandle = DeviceHandle;
						Result = -1;
						if(!CUSBGetDeviceCapabilities(phid, DeviceHandle))
						{
							phid->specificDevice = PTRUE;
							Result = CList_addToList((CListHandle *)curList, phid, CPhidget_areEqual);
						}
						if (Result)
							LOG(PHIDGET_LOG_WARNING,"couldn't get new device capabilities or add to curList");
					}
				}			
				
				if (DeviceHandle != INVALID_HANDLE_VALUE)
					CloseHandle(DeviceHandle);

				memset(devInfo, 0, sizeof(DEVMGR_DEVICE_INFORMATION));
				devInfo->dwSize = sizeof(DEVMGR_DEVICE_INFORMATION);
			}
			MemberIndex++;
		}
		while(FindNextDevice(hSearch, devInfo));
		f = FindClose(hSearch);
		if(!f) {
			LOG(PHIDGET_LOG_DEBUG,"FindClose Failed, %d", GetLastError());  
		}
	}

	free(devInfo); devInfo = NULL;

	return EPHIDGET_OK;
}

void CUSBCleanup() {}

/* 
CUSBOpenHandle takes a CPhidgetInfo structure, with 
ProductID/VendorID/SerialNumber filled in.

If SerialNumber is = -1, CUSBOpenHandle will return with the first available
HID Phidget that matches the ProductID/VendorID.

CUSBOpenHandle should reserve the file handles - in other words,
a future call to CUSBOpenHandle should fail.
*/

int CUSBOpenHandle(CPhidgetHandle phid)
{
	HANDLE								DeviceHandle;
	HANDLE								hSearch;
	PDEVMGR_DEVICE_INFORMATION			devInfo;
	int									MemberIndex, i;
	int									deviceFound;
	unsigned long						Length;
	char * interface_spot = 0;
	int result;

	PHIDDEV_ATTRIBUTES attributes;
	BOOL f;
	DWORD dwBytes;

	Length = 0;
	MemberIndex = deviceFound = 0;
	
	devInfo = (PDEVMGR_DEVICE_INFORMATION)malloc(sizeof(DEVMGR_DEVICE_INFORMATION));
	memset(devInfo, 0, sizeof(DEVMGR_DEVICE_INFORMATION));
	devInfo->dwSize = sizeof(DEVMGR_DEVICE_INFORMATION);
	hSearch = FindFirstDevice(DeviceSearchByDeviceName, L"PHD*", devInfo);
	if(hSearch == INVALID_HANDLE_VALUE)
	{
		switch(GetLastError())
		{
			case ERROR_NO_MORE_FILES:
				return EPHIDGET_NOTFOUND;
			default:
				LOG(PHIDGET_LOG_DEBUG,"FindFirstDevice Failed - INVALID_HANDLE_VALUE, %x", GetLastError());
				return EPHIDGET_UNEXPECTED;
		}
	}
	do {

		DeviceHandle = CreateFile (devInfo->szLegacyName, GENERIC_WRITE | GENERIC_READ,
			FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);

		if(DeviceHandle == INVALID_HANDLE_VALUE)  
		{
			LOG(PHIDGET_LOG_DEBUG,"CreateFile - INVALID_HANDLE_VALUE, %x", GetLastError());
			continue;
		}

		f = DeviceIoControl (DeviceHandle, IOCTL_PHIDGET_DEVICE_GET_ATTRIBUTES, 0, 0, 
								 &attributes, sizeof(attributes), &dwBytes, NULL);
		if(!f) {
			LOG(PHIDGET_LOG_DEBUG,"IOCTL_PHIDGET_DEVICE_GET_ATTRIBUTES Failed, %d", GetLastError());  
		}

		//wprintf(L"Found device: %s %s %s %s\n",devInfo->szDeviceName,devInfo->szBusName,devInfo->szDeviceKey,devInfo->szLegacyName);


		for (i = 0; i<PHIDGET_DEVICE_COUNT; i++) 
		{
			if ((Phid_Device_Def[i].pdd_did == phid->deviceID)
				&& (attributes.VendorID == Phid_Device_Def[i].pdd_vid)
				&& (attributes.ProductID == Phid_Device_Def[i].pdd_pid)
				&& (attributes.InterfaceNumber == Phid_Device_Def[i].pdd_iid))
			{
				result = CUSBGetDeviceCapabilities(phid, (HANDLE)DeviceHandle);
				phid->deviceHandle = DeviceHandle;
				if(result) continue;
				phid->deviceIDSpec = Phid_Device_Def[i].pdd_sdid;
				phid->Phid_Device_Def_index = i;
				phid->deviceType = (char *)Phid_DeviceName[Phid_Device_Def[i].pdd_did];
				phid->attr = Phid_Device_Def[i].pdd_attr;

				//printf("Device Info: Pid: %d Vid: %d Iid: %d Serial: %d Label: %s\n",
				//	attributes.ProductID, attributes.VendorID, attributes.InterfaceNumber, phid->serialNumber, phid->label);
				free(devInfo); devInfo = NULL;
				return EPHIDGET_OK;
			}
		}

		if (DeviceHandle != INVALID_HANDLE_VALUE)
			CloseHandle(DeviceHandle);
		memset(devInfo, 0, sizeof(DEVMGR_DEVICE_INFORMATION));
		devInfo->dwSize = sizeof(DEVMGR_DEVICE_INFORMATION);
	}
	while(FindNextDevice(hSearch, devInfo));

	f = FindClose(hSearch);
	if(!f) {
		LOG(PHIDGET_LOG_DEBUG,"FindClose Failed, %d", GetLastError());  
	}
	free(devInfo); devInfo = NULL;
	return EPHIDGET_NOTFOUND;
}


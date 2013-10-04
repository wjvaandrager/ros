#include "stdafx.h"
#include "cusb.h"

#include <Wtypes.h>
#include <setupapi.h>
#include <wchar.h>
#include <dbt.h>
#include <Winuser.h>
#include "CPhidgetManager.h"

#ifdef _MSC_EXTENSIONS
#include "hidsdi.h"
#include "hidpi.h"
#else
#include <ddk/hidsdi.h>
#include <ddk/hidpi.h>
#endif

#include <assert.h>

int
CUSBCloseHandle(CPhidgetHandle phid)
{
	DWORD errorcode = 0;

	TESTPTR(phid)

	CPhidget_clearStatusFlag(&phid->status, PHIDGET_ATTACHED_FLAG, &phid->lock);
	if (phid->deviceHandle == INVALID_HANDLE_VALUE)
		return EPHIDGET_NOTATTACHED;

	//This stops any pending async reads/writes
	SetEvent((void *)phid->closeReadEvent);
	//don't signal this because we want any outstanding writes to complete, not be cancelled
	//SetEvent((void *)phid->closeWriteEvent); 

	CThread_join(&phid->readThread);

	if(!CloseHandle((void *)phid->deviceHandle))
	{
		errorcode = GetLastError();
		LOG(PHIDGET_LOG_ERROR, "CloseHandle failed with error code: %d", errorcode);
		return EPHIDGET_UNEXPECTED;
	}

	ResetEvent((void *)phid->closeReadEvent);

	CloseHandle((void *)phid->closeReadEvent);
	CloseHandle((void *)phid->asyncRead.hEvent);
	CloseHandle((void *)phid->asyncWrite.hEvent);

	phid->deviceHandle = INVALID_HANDLE_VALUE;

	return EPHIDGET_OK;
}


int
CUSBSetLabel(CPhidgetHandle phid, char *buffer)
{
	TESTPTR(phid)
	return EPHIDGET_UNSUPPORTED;
}

int
CUSBGetDeviceCapabilities(CPhidgetHandle phid, HANDLE DeviceHandle)
{
	int i;
	unsigned short SerialNumber[20];
	char labelData[22];
	int serNum;
	PHIDP_PREPARSED_DATA PreparsedData;
	HIDP_CAPS Capabilities;
	NTSTATUS result;

	TESTPTR(phid)

	if(!HidD_GetPreparsedData (DeviceHandle, &PreparsedData))
	{
		LOG(PHIDGET_LOG_ERROR, "HidD_GetPreparsedData failed");
		return EPHIDGET_UNEXPECTED;
	}
	
	result = HidP_GetCaps (PreparsedData, &Capabilities);
	switch(result)
	{
		case HIDP_STATUS_SUCCESS:
			break;
		case HIDP_STATUS_INVALID_PREPARSED_DATA:
			LOG(PHIDGET_LOG_ERROR, "HidP_GetCaps failed with HIDP_STATUS_INVALID_PREPARSED_DATA");
			HidD_FreePreparsedData(PreparsedData);
			return EPHIDGET_UNEXPECTED;
		default:
			LOG(PHIDGET_LOG_ERROR, "HidP_GetCaps failed with %d", result);
			HidD_FreePreparsedData(PreparsedData);
			return EPHIDGET_UNEXPECTED;
	}

	phid->inputReportByteLength = Capabilities.InputReportByteLength;
	phid->outputReportByteLength = Capabilities.OutputReportByteLength;
	
	HidD_FreePreparsedData(PreparsedData);

	if(HidD_GetIndexedString(DeviceHandle, 4, labelData, 22))
	{
		for(i=0;i<10;i++)
			phid->label[i] = labelData[i*2];
		phid->label[10] = '\0';
	}
	else ZEROMEM(phid->label, 10);

	if (!HidD_GetSerialNumberString(DeviceHandle, SerialNumber, 20))
	{
		LOG(PHIDGET_LOG_ERROR, "HidD_GetSerialNumberString failed");
		return EPHIDGET_UNEXPECTED;
	}

	if(EOF==(swscanf((wchar_t *)(SerialNumber), L"%d", &serNum)))
	{
		LOG(PHIDGET_LOG_ERROR, "swscanf failed to parse out serial number");
		return EPHIDGET_UNEXPECTED;
	}

	if (phid->specificDevice != FALSE)
		if (phid->serialNumber != serNum)
			return EPHIDGET_NOTFOUND;
	phid->serialNumber = serNum;

	return EPHIDGET_OK;
}

//This will be simplified to just build a list and nothing else...
//it doesn't empty the list, just adds to it
int
CUSBBuildList(CPhidgetList **curList)
{
	GUID HidGuid;
	SP_DEVICE_INTERFACE_DATA devInfoData;
	HIDD_ATTRIBUTES Attributes;
	HANDLE DeviceHandle = NULL;
	HANDLE hDevInfo = NULL;
	PSP_DEVICE_INTERFACE_DETAIL_DATA_W detailData = NULL;
	DWORD Length = 0;
	DWORD MemberIndex = 0;
	DWORD errorcode = 0;
	WCHAR * interface_spot = 0;
	int i = 0, interface_num = 0, res = 0;
	int found = 0;
	CPhidgetList *traverse;
	CPhidgetHandle phid;

	HidD_GetHidGuid(&HidGuid);
	if((hDevInfo = SetupDiGetClassDevs(&HidGuid, NULL, NULL, DIGCF_PRESENT|DIGCF_DEVICEINTERFACE)) == INVALID_HANDLE_VALUE)
	{
		errorcode = GetLastError();
		LOG(PHIDGET_LOG_ERROR, "SetupDiGetClassDevs failed with error code: %d", errorcode);
		return EPHIDGET_UNEXPECTED;
	}
	devInfoData.cbSize = sizeof(devInfoData);

	while(1)
	{
		if(!SetupDiEnumDeviceInterfaces(hDevInfo, 0, &HidGuid, MemberIndex++, &devInfoData))
		{
			errorcode = GetLastError();
			switch(errorcode)
			{
			case ERROR_NO_MORE_ITEMS:
				break;
			default:
				LOG(PHIDGET_LOG_ERROR, "SetupDiEnumDeviceInterfaces failed with error code: %d",errorcode);
				return EPHIDGET_UNEXPECTED;
			}
			break; //leave while loop - we're out of interfaces
		}

		//This gets the required length for detailData
		if(!SetupDiGetDeviceInterfaceDetailW(hDevInfo, &devInfoData, NULL, 0, &Length, NULL))
		{
			errorcode = GetLastError();
			switch(errorcode)
			{
			//this is expected
			case ERROR_INSUFFICIENT_BUFFER:
				break;
			default:
				LOG(PHIDGET_LOG_ERROR, "SetupDiGetDeviceInterfaceDetailW failed with error code: %d", errorcode);
				return EPHIDGET_UNEXPECTED;
			}
		}
		//This actually fills in detailData using the length
		if(!(detailData = (PSP_DEVICE_INTERFACE_DETAIL_DATA_W)malloc(Length)))
		{
			return EPHIDGET_NOMEMORY;
		}
		detailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
		if(!SetupDiGetDeviceInterfaceDetailW(hDevInfo, &devInfoData, detailData, Length, NULL, NULL))
		{
			errorcode = GetLastError();
			LOG(PHIDGET_LOG_ERROR, "SetupDiGetDeviceInterfaceDetailW failed with error code: %d", errorcode);
			return EPHIDGET_UNEXPECTED;
		}

		found = 0;
		if (AttachedDevices) {
			for (traverse = AttachedDevices; traverse; traverse=traverse->next)
			{
				if (!wcsncmp((WCHAR *)traverse->phid->CPhidgetFHandle, detailData->DevicePath, 255))
				{
					if((res=CList_addToList((CListHandle *)curList, traverse->phid, CPhidget_areExtraEqual)))
					{
						if(res!=EPHIDGET_DUPLICATE)
						{
							LOG(PHIDGET_LOG_ERROR, "CList_addToList returned nonzero code: %d",res);
							return EPHIDGET_UNEXPECTED;
						}
					}
					found = 1;
				}
			}
		}
		if (!found) {
			interface_spot = wcsstr(detailData->DevicePath,L"mi_");
			if (interface_spot != 0)
				interface_num = (interface_spot)[4] - 48;
			else
				interface_num = 0;
			
			//Don't request any read/write permissions, but make sure open is not blocked (share read/write)
			DeviceHandle = CreateFileW(detailData->DevicePath,
				0, FILE_SHARE_READ|FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);

			if (DeviceHandle != INVALID_HANDLE_VALUE)
			{
				Attributes.Size = sizeof(Attributes);
			    if(HidD_GetAttributes (DeviceHandle, &Attributes)) 
				{
					const CPhidgetDeviceDef *pdd = Phid_Device_Def;
					i = 0;

					while (pdd->pdd_sdid)
					{
						if (Attributes.VendorID == pdd->pdd_vid
							&& Attributes.ProductID == pdd->pdd_pid
							&& interface_num == pdd->pdd_iid)
						{
							if((res = CPhidget_create(&phid))) return res;

							LOG(PHIDGET_LOG_INFO,"New Phidget found in CUSBBuildList: %ls", detailData->DevicePath);

							if (Attributes.VersionNumber < 0x100) 
								phid->deviceVersion = Attributes.VersionNumber * 100;
							else 
								phid->deviceVersion = ((Attributes.VersionNumber >> 8) * 100) + ((Attributes.VersionNumber & 0xff));

							phid->deviceType = Phid_DeviceName[pdd->pdd_did];
							phid->deviceIDSpec = pdd->pdd_sdid;
							phid->deviceID = pdd->pdd_did;
							phid->ProductID = Attributes.ProductID;
							phid->VendorID = Attributes.VendorID;
							phid->Phid_Device_Def_index = i;

							CPhidget_setStatusFlag(&phid->status, PHIDGET_ATTACHED_FLAG, &phid->lock);
							
							phid->CPhidgetFHandle = malloc(wcslen(detailData->DevicePath)*sizeof(WCHAR)+10);
							wcsncpy((WCHAR *)phid->CPhidgetFHandle, detailData->DevicePath, wcslen(detailData->DevicePath)+1);
							
							phid->serialNumber = -1;
							if(!(res = CUSBGetDeviceCapabilities(phid, DeviceHandle)))
							{
								phid->specificDevice = TRUE;
								if((res=CList_addToList((CListHandle *)curList, phid, CPhidget_areEqual)))
								{
									LOG(PHIDGET_LOG_ERROR, "CList_addToList returned nonzero code: %d",res);
									return EPHIDGET_UNEXPECTED;
								}
							}
							else
							{
								LOG(PHIDGET_LOG_WARNING,"CUSBGetDeviceCapabilities failed with error code: %d", res);
							}
						}
						pdd++;
						i++;
					} //while (pdd->pdd_did)
				} //if(HidD_GetAttributes (DeviceHandle, &Attributes)) 
				else
				{
					errorcode = GetLastError();
					LOG(PHIDGET_LOG_ERROR, "HidD_GetAttributes failed with error code: %d", errorcode);
				}
				if (DeviceHandle != INVALID_HANDLE_VALUE)
				{
					if(!CloseHandle(DeviceHandle))
					{
						errorcode = GetLastError();
						LOG(PHIDGET_LOG_ERROR, "CloseHandle failed with error code: %d", errorcode);
					}
				}
			} //if (DeviceHandle != INVALID_HANDLE_VALUE)
			else
			{
				errorcode = GetLastError();
				//this is expected at times - since we're given a list of all HID devices
				switch(errorcode)
				{
				case ERROR_ACCESS_DENIED:
					break; //probably not a Phidget but a mouse or something
				default:
					LOG(PHIDGET_LOG_INFO, "CreateFileW failed with error code: %d", errorcode);
					LOG(PHIDGET_LOG_DEBUG,"could not create (open) %ls", detailData->DevicePath);
				}
			}

			if (detailData) {
				free(detailData); detailData = NULL;
			}
		}
		if (detailData)
		{
			free(detailData); detailData = NULL;
		}
	}
	if(!SetupDiDestroyDeviceInfoList(hDevInfo))
	{
		errorcode = GetLastError();
		LOG(PHIDGET_LOG_ERROR, "SetupDiDestroyDeviceInfoList failed with error code: %d", errorcode);
	}

	return EPHIDGET_OK;
}

/* 
CHIDOpenHandle takes a CPhidgetInfo structure, with 
device file path filled in.

CHIDOpenHandle should reserve the file handles - in other words,
a future call to CHIDOpenHandle should fail.
*/
int
CUSBOpenHandle(CPhidgetHandle phid)
{
	DWORD errorcode = 0;
	int i = 0, res = 0;
	HIDD_ATTRIBUTES Attributes;
	DWORD extra_attributes = 0;

	//check incoming pointers
	TESTPTR(phid)
	phid->deviceHandle = INVALID_HANDLE_VALUE;

	//Special case for old TextLCD 0/0/8's - open in write-shared mode so both halfs can be used at once.
	switch(phid->deviceIDSpec)
	{
		case PHIDID_INTERFACEKIT_0_8_8_w_LCD:
		case PHIDID_INTERFACEKIT_0_5_7:
		case PHIDID_TEXTLCD_2x20:
		case PHIDID_TEXTLCD_2x20_w_0_8_8:
			extra_attributes = FILE_SHARE_WRITE;
			break;
		default:
			break;
	}

	//We have to enable read sharing so that the manager in other apps can read the device info
	// but don't share write, or the device can be opened twice
	phid->deviceHandle = CreateFileW(phid->CPhidgetFHandle,
	    GENERIC_READ|GENERIC_WRITE, 
	    FILE_SHARE_READ | extra_attributes,
	    NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);

	if (phid->deviceHandle != INVALID_HANDLE_VALUE)
	{	
		Attributes.Size = sizeof(Attributes);
		if (HidD_GetAttributes (phid->deviceHandle, &Attributes))
		{
			if (!(res = CUSBGetDeviceCapabilities(phid, phid->deviceHandle)))
			{
				LOG(PHIDGET_LOG_INFO,"Phidget successfully opened in CUSBOpenHandle");

				phid->asyncRead.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
				phid->closeReadEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
				phid->readPending = FALSE;
				phid->asyncWrite.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);

				if ((!phid->asyncRead.hEvent) || (!phid->asyncWrite.hEvent) || (!phid->closeReadEvent))
				{ 
					errorcode = GetLastError();
					LOG(PHIDGET_LOG_ERROR, "CreateEvent failed with error code: %d", errorcode);
					CloseHandle(phid->deviceHandle);
					return EPHIDGET_UNEXPECTED;
				}

				phid->deviceType = Phid_DeviceName[phid->deviceID];
				phid->attr = Phid_Device_Def[phid->Phid_Device_Def_index].pdd_attr;

				phid->ProductID = Attributes.ProductID;
				phid->VendorID = Attributes.VendorID;

				if (Attributes.VersionNumber < 0x100)
					phid->deviceVersion = Attributes.VersionNumber * 100;
				else
					phid->deviceVersion = ((Attributes.VersionNumber >> 8) * 100) + ((Attributes.VersionNumber & 0xff)); 

				return EPHIDGET_OK;
			} //if (!CUSBGetDeviceCapabilities(phid, DeviceHandle))
			else
			{
				LOG(PHIDGET_LOG_ERROR, "CUSBGetDeviceCapabilities returned nonzero code: %d", res);
			}
		} //if (HidD_GetAttributes (DeviceHandle, &Attributes))
		else
		{
			errorcode = GetLastError();
			LOG(PHIDGET_LOG_ERROR, "HidD_GetAttributes failed with error code: %d", errorcode);
		}

		if(!CloseHandle(phid->deviceHandle))
		{
			errorcode = GetLastError();
			LOG(PHIDGET_LOG_ERROR, "CloseHandle failed with error code: %d", errorcode);
		}
		return EPHIDGET_UNEXPECTED;

	} //if(DeviceHandle != INVALID_HANDLE_VALUE)
	else
	{
		errorcode = GetLastError();
		//this is expected at times - since we're given a list of all HID devices
		switch(errorcode)
		{
		case ERROR_ACCESS_DENIED:
			LOG(PHIDGET_LOG_INFO, "CreateFileW failed with ERROR_ACCESS_DENIED");
			break; //probably not a Phidget but a mouse or something
		case ERROR_SHARING_VIOLATION:
			LOG(PHIDGET_LOG_INFO, "CreateFileW failed with ERROR_SHARING_VIOLATION - probably the Phidget is opened elsewhere.");
			break;
		default:
			LOG(PHIDGET_LOG_INFO, "CreateFileW failed with error code: %d", errorcode);
			LOG(PHIDGET_LOG_DEBUG,"could not create (open) %ls", phid->CPhidgetFHandle);
		}

		return EPHIDGET_NOTFOUND;
	}
}

/* CHIDSendPacket compensates for an oddity in the Windows HID driver.
Windows HID expects the first byte of the packet to be a zero byte,
thus offsetting all the bytes by 1.  None of the other operating systems
have this behaviour, as far as I know.  Rather than offsetting all bytes 
in the functions that format the packets, I deal with it here, for maximum
code reuse on Linux/MAC. */

int CCONV
CUSBSendPacket(CPhidgetHandle phid, unsigned char *buffer)
{
	unsigned char outbuffer[100];
	DWORD wait_return = 0, errorcode=0;
	int i = 0;
	unsigned long BytesWritten = 0;

	TESTPTRS(phid, buffer)

	if (!CPhidget_statusFlagIsSet(phid->status, PHIDGET_ATTACHED_FLAG)
		&& !CPhidget_statusFlagIsSet(phid->status, PHIDGET_ATTACHING_FLAG))
		return EPHIDGET_NOTATTACHED;

	if (phid->deviceHandle == INVALID_HANDLE_VALUE)
		return EPHIDGET_UNEXPECTED;

	ZEROMEM(outbuffer, sizeof(outbuffer));
	memcpy(outbuffer+1, buffer, phid->outputReportByteLength);

	phid->asyncWrite.Offset = 0;
	phid->asyncWrite.OffsetHigh = 0;

	if (!WriteFile((void *)phid->deviceHandle, outbuffer, phid->outputReportByteLength, &BytesWritten, &phid->asyncWrite))
	{
		errorcode = GetLastError();
		switch(errorcode)
		{
		//have to wait for the async return from write
		case ERROR_IO_PENDING:
			goto wait_for_write_complete;
		//can get this is the device is unplugged
		case ERROR_DEVICE_NOT_CONNECTED:
			LOG(PHIDGET_LOG_ERROR,"WriteFile failed with error: ERROR_DEVICE_NOT_CONNECTED");
			return EPHIDGET_NOTATTACHED;
		case ERROR_IO_INCOMPLETE:
			LOG(PHIDGET_LOG_ERROR,"WriteFile failed with error: ERROR_IO_INCOMPLETE");
			return EPHIDGET_UNEXPECTED;
		case ERROR_INVALID_USER_BUFFER:
		case ERROR_WORKING_SET_QUOTA:
		case ERROR_NOT_ENOUGH_QUOTA:
		case ERROR_NOT_ENOUGH_MEMORY:
			LOG(PHIDGET_LOG_ERROR,"WriteFile failed with error: %d - Probably too many outstanding asynchronous I/O requests.", errorcode);
			return EPHIDGET_UNEXPECTED;
		default:
			LOG(PHIDGET_LOG_ERROR,"WriteFile failed with error code: %d", errorcode);
			return EPHIDGET_UNEXPECTED;
		}
	}
	// readfile succeeded right away - no async
	else
	{
		goto write_completed;
	}

	//have to wait for WriteFile async return
wait_for_write_complete:
	wait_return = WaitForSingleObject ((HANDLE)phid->asyncWrite.hEvent, 1000);
	switch(wait_return)
	{
	case WAIT_OBJECT_0: //async write returned
		//we want this to return non-zero
		if(!(GetOverlappedResult((void *)phid->deviceHandle, &phid->asyncWrite, &BytesWritten, FALSE)))
		{
			errorcode = GetLastError();
			switch(errorcode)
			{
			case ERROR_IO_INCOMPLETE:
				LOG(PHIDGET_LOG_ERROR,"GetOverlappedResult failed with error: ERROR_IO_INCOMPLETE");
				return EPHIDGET_UNEXPECTED;
			case ERROR_DEVICE_NOT_CONNECTED:
				LOG(PHIDGET_LOG_INFO,"GetOverlappedResult failed with error: ERROR_DEVICE_NOT_CONNECTED");
				return EPHIDGET_NOTATTACHED;
			case ERROR_GEN_FAILURE:
				//TODO: ESD event
				LOG(PHIDGET_LOG_ERROR,"GetOverlappedResult failed with error: ERROR_GEN_FAILURE (A device attached to the system is not functioning)");
				return EPHIDGET_UNEXPECTED;
			default:
				LOG(PHIDGET_LOG_ERROR,"GetOverlappedResult in CUSBSendPacket failed with error code: %d", errorcode);
				return EPHIDGET_UNEXPECTED;
			}
		}
		//good - the async write returned successfully
		else
		{
			goto write_completed;
		}
		break;
	case WAIT_TIMEOUT: //we don't recover from this like in read
		LOG(PHIDGET_LOG_ERROR,"Wait on closeWriteEvent in CUSBSendPacket timed out.");
		return EPHIDGET_TIMEOUT;
	case WAIT_FAILED:
	case WAIT_ABANDONED:
	default:
		errorcode = GetLastError();
		LOG(PHIDGET_LOG_ERROR,"Wait on closeWriteEvent in CUSBSendPacket failed with error code: %d", errorcode);
		return EPHIDGET_UNEXPECTED;
	}

write_completed:
 
	if (BytesWritten != phid->outputReportByteLength)
	{
		LOG(PHIDGET_LOG_ERROR,"Failure in CHIDSendPacket - Report Length"
			": %d, Bytes Written: %d", (int)phid->outputReportByteLength,
		    (int)BytesWritten);
		return EPHIDGET_UNEXPECTED;
	}

	return EPHIDGET_OK;
}


/* CHIDReadPacket compensates for an oddity in the Windows HID driver.
Windows HID returns an extra byte on the beginning of the packet, always
set to zero.  None of the other operating systems have this behaviour, 
as far as I know.  Rather than offsetting all bytes in the functions that
parse the packets, I deal with it here, for maximum code reuse on Linux/MAC. */

/* Buffer should be at least 8 bytes long */
int CCONV
CUSBReadPacket(CPhidgetHandle phid, unsigned char *buffer)
{
	int tryagain=5;
	DWORD wait_return = 0, errorcode=0;
	HANDLE waitEvents[2];

	unsigned long BytesRead = 0;

	TESTPTR(phid)

	if (!CPhidget_statusFlagIsSet(phid->status, PHIDGET_ATTACHED_FLAG)
		&& !CPhidget_statusFlagIsSet(phid->status, PHIDGET_ATTACHING_FLAG))
		return EPHIDGET_NOTATTACHED;

	if (phid->deviceHandle == INVALID_HANDLE_VALUE)
		return EPHIDGET_UNEXPECTED;

	if(phid->readPending)
		goto wait_for_read_data;

	phid->asyncRead.Offset = 0;
	phid->asyncRead.OffsetHigh = 0;

	//read a packet from the device
	if (!ReadFile((void *)phid->deviceHandle, phid->inbuf, phid->inputReportByteLength, &BytesRead, &phid->asyncRead))
	{
		errorcode = GetLastError();
		switch(errorcode)
		{
		//have to wait for the async return from read
		case ERROR_IO_PENDING:
			phid->readPending = TRUE;
			goto wait_for_read_data;
		//can get this is teh device is unplugged
		case ERROR_DEVICE_NOT_CONNECTED:
			LOG(PHIDGET_LOG_ERROR,"ReadFile failed with error: ERROR_DEVICE_NOT_CONNECTED");
			return EPHIDGET_NOTATTACHED;
		case ERROR_IO_INCOMPLETE:
			LOG(PHIDGET_LOG_ERROR,"ReadFile failed with error: ERROR_IO_INCOMPLETE");
			return EPHIDGET_UNEXPECTED;
		case ERROR_INVALID_USER_BUFFER: //this happens when we try to read a device with size 0 read buffer
			LOG(PHIDGET_LOG_ERROR,"ReadFile failed with error: ERROR_INVALID_USER_BUFFER");
			return EPHIDGET_UNEXPECTED;
		case ERROR_WORKING_SET_QUOTA:
		case ERROR_NOT_ENOUGH_QUOTA:
		case ERROR_NOT_ENOUGH_MEMORY:
			LOG(PHIDGET_LOG_ERROR,"ReadFile failed with error: %d - Probably too many outstanding asynchronous I/O requests.", errorcode);
			return EPHIDGET_UNEXPECTED;
		default:
			LOG(PHIDGET_LOG_ERROR,"ReadFile failed with error code: %d", errorcode);
			return EPHIDGET_UNEXPECTED;
		}
	}
	// readfile succeeded right away - no async
	else
	{
		goto handle_read_data;
	}

	//have to wait for readfile async return
wait_for_read_data:
	waitEvents[0] = (HANDLE)phid->asyncRead.hEvent;
	waitEvents[1] = (HANDLE)phid->closeReadEvent;
	wait_return = WaitForMultipleObjects (2, waitEvents, FALSE, 500);
	switch(wait_return)
	{
	case WAIT_OBJECT_0: //async read returned
		//we want this to return non-zero
		if(!(GetOverlappedResult((void *)phid->deviceHandle, &phid->asyncRead, &BytesRead, FALSE)))
		{
			errorcode = GetLastError();
			switch(errorcode)
			{
			case ERROR_IO_INCOMPLETE:
				LOG(PHIDGET_LOG_ERROR,"GetOverlappedResult failed with error: ERROR_IO_INCOMPLETE");
				return EPHIDGET_UNEXPECTED;
			case ERROR_DEVICE_NOT_CONNECTED:
				LOG(PHIDGET_LOG_INFO,"GetOverlappedResult failed with error: ERROR_DEVICE_NOT_CONNECTED");
				return EPHIDGET_NOTATTACHED;
			case ERROR_GEN_FAILURE:
				LOG(PHIDGET_LOG_ERROR,"GetOverlappedResult failed with error: ERROR_GEN_FAILURE (A device attached to the system is not functioning)");
				return EPHIDGET_UNEXPECTED;
			default:
				LOG(PHIDGET_LOG_ERROR,"GetOverlappedResult in CUSBReadPacket failed with error code: %d", errorcode);
				return EPHIDGET_UNEXPECTED;
			}
		}
		//good - the async read returned successfully
		else
		{
			phid->readPending = FALSE;
			goto handle_read_data;
		}
		break;
	case WAIT_OBJECT_0+1: //CUSBCloseHandle signalled closeReadEvent
		LOG(PHIDGET_LOG_INFO,"closeReadEvent signalled - cancelling outstanding read...");
		//cancel the IO
		if(!CancelIo(phid->deviceHandle))
		{
			errorcode = GetLastError();
			LOG(PHIDGET_LOG_ERROR,"CancelIo in CUSBReadPacket failed with error code: %d", errorcode);
			return EPHIDGET_UNEXPECTED;
		}
		//make sure it got cancelled
		if(!(GetOverlappedResult((void *)phid->deviceHandle, &phid->asyncRead, &BytesRead, TRUE)))
		{
			errorcode = GetLastError();
			switch(errorcode)
			{
			case ERROR_IO_INCOMPLETE:
			case ERROR_OPERATION_ABORTED:
				LOG(PHIDGET_LOG_INFO,"read successfully cancelled");
				break;
			default:
				LOG(PHIDGET_LOG_ERROR,"GetOverlappedResult unexpectedly returned: %d, while trying to cancel read", errorcode);
				return EPHIDGET_UNEXPECTED;
			}
		}
		return EPHIDGET_INTERRUPTED; //don't try to act on any returned data
	case WAIT_TIMEOUT:
		LOG(PHIDGET_LOG_VERBOSE,"Wait on closeReadEvent in CUSBReadPacket timed out."); //verbose because it could happen a LOT
		return EPHIDGET_TIMEOUT; //this might be ok
	case WAIT_FAILED:
	default:
		errorcode = GetLastError();
		LOG(PHIDGET_LOG_ERROR,"Wait on closeReadEvent in CUSBReadPacket failed with error code: %d", errorcode);
		return EPHIDGET_UNEXPECTED;
	}

handle_read_data:

	if (phid->inputReportByteLength != BytesRead) 
	{
		LOG(PHIDGET_LOG_ERROR,"Failure in CHIDReadPacket - Report Length"
			": %d, Bytes Read: %d", (int)phid->inputReportByteLength,
	    (int)BytesRead);
		return EPHIDGET_UNEXPECTED;
	}

	memcpy(buffer, phid->inbuf + 1, phid->inputReportByteLength);
	return EPHIDGET_OK;
}

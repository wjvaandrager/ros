#include "stdafx.h"
#include "cphidgetrfid.h"
#include "stdio.h"
#include "cusb.h"
#include "csocket.h"
#include "cthread.h"

// === Internal Functions === //

CThread_func_return_t tagTimerThreadFunction(CThread_func_arg_t userPtr);

//clearVars - sets all device variables to unknown state
CPHIDGETCLEARVARS(RFID)
	int i = 0;

	phid->antennaEchoState = PUNI_BOOL;
	phid->ledEchoState = PUNI_BOOL;
	phid->tagPresent = PUNI_BOOL;
	phid->fullStateEcho = PUNK_BOOL;
	phid->ledState = PUNK_BOOL;
	phid->antennaState = PUNK_BOOL;

	for (i = 0; i<RFID_MAXOUTPUTS; i++)
	{
		phid->outputEchoState[i] = PUNI_BOOL;
	}

	return EPHIDGET_OK;
}

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
CPHIDGETINIT(RFID)
	int i = 0;
	unsigned char buffer[8] = { 0 };
	int result;
	TESTPTR(phid);

	//setup anything device specific
	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_RFID:
			if (phid->phid.deviceVersion < 200)
			{
				phid->fullStateEcho = PFALSE;
				phid->antennaEchoState = PTRUE;
			}
			else
				return EPHIDGET_UNEXPECTED;
			break;
		case PHIDID_RFID_2OUTPUT: //2-output RFID
			if ((phid->phid.deviceVersion  >= 200) && (phid->phid.deviceVersion  < 201))
			{
				phid->antennaEchoState = PUNK_BOOL;
				phid->fullStateEcho = PFALSE;
			}
			else if ((phid->phid.deviceVersion  >= 201) && (phid->phid.deviceVersion  < 300))
			{
				phid->antennaEchoState = PUNK_BOOL;
				phid->fullStateEcho = PTRUE;
			}
			else
				return EPHIDGET_UNEXPECTED;
			break;
		default:
			return EPHIDGET_UNEXPECTED;
	}

	//set data arrays to unknown
	for (i = 0; i<phid->phid.attr.rfid.numOutputs; i++)
	{
		phid->outputEchoState[i] = PUNK_BOOL;
	}
	phid->ledEchoState = PUNK_BOOL;
	phid->tagPresent = PUNK_BOOL;
	ZEROMEM(phid->lastTag, 10);

	phid->tagEvent = PFALSE;

	//send out any initial pre-read packets
	switch(phid->phid.deviceIDSpec) {
		case PHIDID_RFID:
			if (phid->phid.deviceVersion <= 103)
			{
				ZEROMEM(buffer,8);
				LOG(PHIDGET_LOG_INFO,"Sending workaround startup packet");
				if ((result = CUSBSendPacket((CPhidgetHandle)phid, buffer)) != EPHIDGET_OK)
					return result;
			}
			break;
		case PHIDID_RFID_2OUTPUT:
		default:
			break;
	}

	//issue a read for devices that return output data
	if(phid->fullStateEcho)
	{
		int readtries = 15; //should guarentee a packet with output data - even if a tag is present
		while(readtries-- > 0)
		{
			CPhidget_read((CPhidgetHandle)phid);
			if(phid->outputEchoState[0] != PUNK_BOOL)
					break;
		}
		//one more read guarantees that if there is a tag present, we will see it - output packets only happen every 255ms
		CPhidget_read((CPhidgetHandle)phid);
	}

	//if the antenna is on, and tagPresent is unknown, then it is false
	if(phid->antennaEchoState == PTRUE && phid->tagPresent == PUNK_BOOL)
		phid->tagPresent = PFALSE;

	//recover what we can - if anything isn't filled out, it's PUNK anyways
	for (i = 0; i<phid->phid.attr.rfid.numOutputs; i++)
	{
		phid->outputState[i] = phid->outputEchoState[i];
	}
	phid->antennaState = phid->antennaEchoState;
	phid->ledState = phid->ledEchoState;

	//make sure the tagTimerThread isn't running
	if (phid->tagTimerThread.thread_status == PTRUE)
	{
		phid->tagTimerThread.thread_status = PFALSE;
		CThread_join(&phid->tagTimerThread);
	}

	return EPHIDGET_OK;
}

//dataInput - parses device packets
CPHIDGETDATA(RFID)
	int i = 0, j = 0;
	unsigned char newTag = PFALSE, newOutputs = PFALSE;
	unsigned char tagData[5];
	unsigned char outputs[RFID_MAXOUTPUTS];
	unsigned char lastOutputs[RFID_MAXOUTPUTS];
	unsigned char antennaState = 0, ledState = 0;

	if (length<0) return EPHIDGET_INVALIDARG;
	TESTPTR(phid);
	TESTPTR(buffer);

	ZEROMEM(tagData, sizeof(tagData));
	ZEROMEM(outputs, sizeof(outputs));
	ZEROMEM(lastOutputs, sizeof(lastOutputs));

	//Parse device packets - store data locally
	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_RFID:
			if (phid->phid.deviceVersion < 200)
			{
			#ifdef _WINDOWS
				GetSystemTime(&phid->lastTagTime);
			#else
				gettimeofday(&phid->lastTagTime,NULL);
			#endif
				if(!memcmp(phid->lastTag, buffer+1, 5) && phid->tagPresent == PTRUE)
					newTag = PFALSE;
				else if(!memcmp("\0\0\0\0\0", buffer+1, 5))
					newTag = PFALSE;
				else
				{
					memcpy(tagData, buffer+1, 5);
					newTag = PTRUE;
				}
			}
			else
				return EPHIDGET_UNEXPECTED;
			break;
		case PHIDID_RFID_2OUTPUT:
			if ((phid->phid.deviceVersion  >= 200) && (phid->phid.deviceVersion  < 300))
			{
				switch(buffer[0])
				{
					case RFID_PACKET_TAG:
					#ifdef _WINDOWS
						GetSystemTime(&phid->lastTagTime);
					#else
						gettimeofday(&phid->lastTagTime,NULL);
					#endif
						if(!memcmp(phid->lastTag, buffer+1, 5) && phid->tagPresent == PTRUE)
							newTag = PFALSE;
						else if(!memcmp("\0\0\0\0\0", buffer+1, 5))
							newTag = PFALSE;
						else
						{
							memcpy(tagData, buffer+1, 5);
							newTag = PTRUE;
						}

						break;
					case RFID_PACKET_OUTPUT_ECHO:
						if(phid->fullStateEcho)
						{
							newOutputs = PTRUE;

							for (i = 0, j = 0x01; i < phid->phid.attr.rfid.numOutputs; i++, j<<=1)
							{
								if (buffer[1] & j)
									outputs[i] = PTRUE;
								else
									outputs[i] = PFALSE;
							}

							if(buffer[1] & RFID_LED_FLAG)
								ledState = PTRUE;
							else
								ledState = PFALSE;

							if(buffer[1] & RFID_ANTENNA_FLAG)
								antennaState = PTRUE;
							else
								antennaState = PFALSE;
						}
						break;
					default:
						return EPHIDGET_UNEXPECTED;
				}
			}
			else
				return EPHIDGET_UNEXPECTED;
			break;
		default:
			return EPHIDGET_UNEXPECTED;
	}

	//Make sure values are within defined range, and store to structure
	if(newOutputs)
	{
		for (i = 0; i < phid->phid.attr.rfid.numOutputs; i++)
		{
			lastOutputs[i] = phid->outputEchoState[i];
			phid->outputEchoState[i] = outputs[i];
		}
		phid->ledEchoState = ledState;
		phid->antennaEchoState = antennaState;
	}
	if(newTag)
	{
		phid->tagEvent = PTRUE;
		phid->tagPresent = PTRUE;
	}

	//send out any events
	if(newTag) {
		FIRE(Tag, tagData);

		//This will let us block in a tag event
	#ifdef _WINDOWS
		GetSystemTime(&phid->lastTagTime);
	#else
		gettimeofday(&phid->lastTagTime,NULL);
	#endif
		phid->tagEvent = PFALSE;

		//so they can access the last tag from this tag event
		memcpy(phid->lastTag, tagData, 5);
	}
	if(newOutputs)
	{
		for (i = 0; i < phid->phid.attr.rfid.numOutputs; i++)
		{
			if(lastOutputs[i] != phid->outputEchoState[i])
				FIRE(OutputChange, i, phid->outputEchoState[i]);
		}
	}
	
	return EPHIDGET_OK;
}

//eventsAfterOpen - sends out an event for all valid data, used during attach initialization
CPHIDGETINITEVENTS(RFID)
	int i = 0;

	if(phid->tagPresent == PTRUE)
		FIRE(Tag, phid->lastTag);

	if(phid->fullStateEcho)
	{
		for (i = 0; i < phid->phid.attr.rfid.numOutputs; i++)
		{
			if(phid->outputEchoState[i] != PUNK_BOOL)
				FIRE(OutputChange, i, phid->outputEchoState[i]);
		}
	}

	//if we previously found a tag, update the timestamp (since read hasn't been running to update it) to avoid tagLost
	if(phid->tagPresent == PTRUE)
	{
#ifdef _WINDOWS
		GetSystemTime(&phid->lastTagTime);
#else
		gettimeofday(&phid->lastTagTime,NULL);
#endif
	}

	//Don't start the tag thread if this is a networked Phidget
	if(!CPhidget_statusFlagIsSet(phid->phid.status, PHIDGET_REMOTE_FLAG))
	{
		//Start the tagTimerThread - do it here because we are about to start the read thread, and that will keep it active
		if (CThread_create(&phid->tagTimerThread, tagTimerThreadFunction, phid))
			return EPHIDGET_UNEXPECTED;
	}

	return EPHIDGET_OK;
}

//getPacket - used by write thread to get the next packet to send to device
CGETPACKET_BUF(RFID)

//sendpacket - sends a packet to the device asynchronously, blocking if the 1-packet queue is full
CSENDPACKET_BUF(RFID)

//makePacket - constructs a packet using current device state
CMAKEPACKET(RFID)
	int i = 0, j = 0;

	TESTPTRS(phid, buffer);

	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_RFID_2OUTPUT: //4-output RFID
			if ((phid->phid.deviceVersion  >= 200) && (phid->phid.deviceVersion  < 300))
			{
				//have to make sure that everything to be sent has some sort of default value if the user hasn't set a value
				for (i = 0; i < phid->phid.attr.rfid.numOutputs; i++)
				{
					if (phid->outputState[i] == PUNK_BOOL)
						phid->outputState[i] = PFALSE;
				}
				if(phid->antennaState == PUNK_BOOL)
					phid->antennaState = PFALSE;
				if(phid->ledState == PUNK_BOOL)
					phid->ledState = PFALSE;

				//construct the packet
				for (i = 0, j = 1; i < phid->phid.attr.rfid.numOutputs; i++, j<<=1)
				{
					if (phid->outputState[i])
						buffer[0] |= j;
				}
				if(phid->ledState == PTRUE)
					buffer[0] |= RFID_LED_FLAG;
				if(phid->antennaState == PTRUE)
					buffer[0] |= RFID_ANTENNA_FLAG;
			}
			else
				return EPHIDGET_UNEXPECTED;
			break;
		case PHIDID_RFID:
			return EPHIDGET_UNSUPPORTED; //this version does not have outputs
		default:
			return EPHIDGET_UNEXPECTED;
	}
	return EPHIDGET_OK;
}

//if the time since last tag read > 200ms, fire tagLost event
//NOTE: blocking in data events for too long will cause tagLost events
CThread_func_return_t tagTimerThreadFunction(CThread_func_arg_t userPtr)
{
	CPhidgetRFIDHandle phid = (CPhidgetRFIDHandle)userPtr;

	if(!phid) return (CThread_func_return_t)EPHIDGET_INVALIDARG;

	LOG(PHIDGET_LOG_INFO,"tagTimerThread running");

	phid->tagTimerThread.thread_status = PTRUE;

	while (CPhidget_statusFlagIsSet(phid->phid.status, PHIDGET_ATTACHED_FLAG) && phid->tagTimerThread.thread_status == PTRUE)
	{
		if(phid->tagPresent != PFALSE && phid->tagEvent != PTRUE)
		{
			/* check for tag lost */
		#ifdef _WINDOWS
			TIME now;
			FILETIME nowft, oldft, resft;
			ULARGE_INTEGER nowul, oldul, resul;
			double duration;
			GetSystemTime(&now);
			SystemTimeToFileTime(&now, &nowft);
			SystemTimeToFileTime(&phid->lastTagTime, &oldft);

			nowul.HighPart = nowft.dwHighDateTime;
			nowul.LowPart = nowft.dwLowDateTime;
			oldul.HighPart = oldft.dwHighDateTime;
			oldul.LowPart = oldft.dwLowDateTime;

			resul.HighPart = nowul.HighPart - oldul.HighPart;
			resul.LowPart = nowul.LowPart - oldul.LowPart;

			resft.dwHighDateTime = resul.HighPart;
			resft.dwLowDateTime = resul.LowPart;

			duration = (double)(resft.dwLowDateTime/10000000.0);
		#else
			struct timeval now;
			gettimeofday(&now, NULL);
			double duration = (now.tv_sec - phid->lastTagTime.tv_sec) + (double)((now.tv_usec-phid->lastTagTime.tv_usec)/1000000.0);
		#endif
			if(duration > 0.2)
			{
				if (phid->tagPresent == PTRUE) {
					phid->tagPresent = PFALSE;
					FIRE(TagLost, phid->lastTag);
				}
				else if(phid->antennaEchoState == PTRUE) //could be PUNK_BOOL - don't send event, just set to PFALSE (but only if the antenna is on)
					phid->tagPresent = PFALSE;
			}
		}
		SLEEP(50);
	}

	LOG(PHIDGET_LOG_INFO,"tagTimerThread exiting normally");
	phid->tagTimerThread.thread_status = FALSE;
	return EPHIDGET_OK;
}

// === Exported Functions === //

//create and initialize a device structure
CCREATE(RFID, PHIDCLASS_RFID)

//event setup functions
CFHANDLE(RFID, OutputChange, int, int)
CFHANDLE(RFID, Tag, unsigned char *)
CFHANDLE(RFID, TagLost, unsigned char *)

CGET(RFID,OutputCount,int)
	TESTPTRS(phid,pVal) 
	TESTDEVICETYPE(PHIDCLASS_RFID)
	TESTATTACHED

	MASGN(phid.attr.rfid.numOutputs)
}

CGETINDEX(RFID,OutputState,int)
	TESTPTRS(phid,pVal) 
	TESTDEVICETYPE(PHIDCLASS_RFID)
	TESTATTACHED
	TESTINDEX(phid.attr.rfid.numOutputs)
	TESTMASGN(outputState[Index], PUNK_BOOL)

	MASGN(outputState[Index])
}
CSETINDEX(RFID,OutputState,int)
	TESTPTR(phid)
	TESTDEVICETYPE(PHIDCLASS_RFID)
	TESTATTACHED
	TESTRANGE(PFALSE, PTRUE)
	TESTINDEX(phid.attr.rfid.numOutputs)

	if(CPhidget_statusFlagIsSet(phid->phid.status, PHIDGET_REMOTE_FLAG))
		ADDNETWORKKEYINDEXED(Output, "%d", outputState);
	else
	{
		SENDPACKET(RFID, outputState[Index]);
		//echo back output state if the device doesn't
		//do it here because this is after the packet has been sent off - so blocking in this event will not delay the output
		if (!(phid->fullStateEcho))
		{
			if (phid->outputEchoState[Index] == PUNK_BOOL || phid->outputEchoState[Index] != newVal)
			{
				phid->outputEchoState[Index] = newVal;
				FIRE(OutputChange, Index, newVal);
			}
		}
	}

	return EPHIDGET_OK;
}

CGET(RFID,AntennaOn,int)
	TESTPTRS(phid,pVal) 
	TESTDEVICETYPE(PHIDCLASS_RFID)
	TESTATTACHED
	TESTMASGN(antennaEchoState, PUNK_BOOL)

	MASGN(antennaEchoState)
}
CSET(RFID,AntennaOn,int)
	TESTPTR(phid)
	TESTDEVICETYPE(PHIDCLASS_RFID)
	TESTATTACHED

	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_RFID_2OUTPUT:
			TESTRANGE(PFALSE, PTRUE)

			if(CPhidget_statusFlagIsSet(phid->phid.status, PHIDGET_REMOTE_FLAG))
				ADDNETWORKKEY(AntennaOn, "%d", antennaState);
			else
			{
				SENDPACKET(RFID, antennaState);
				//echo back state if the device doesn't
				if (!(phid->fullStateEcho))
					phid->antennaEchoState = newVal;
			}
			return EPHIDGET_OK;
		case PHIDID_RFID:
		default:
			return EPHIDGET_UNSUPPORTED;
	}
}

CGET(RFID,LEDOn,int)
	TESTPTRS(phid,pVal) 
	TESTDEVICETYPE(PHIDCLASS_RFID)
	TESTATTACHED

	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_RFID_2OUTPUT:
			TESTMASGN(ledEchoState, PUNK_BOOL)
			MASGN(ledEchoState)
		case PHIDID_RFID:
		default:
			return EPHIDGET_UNSUPPORTED;
	}
}
CSET(RFID,LEDOn,int)
	TESTPTR(phid)
	TESTDEVICETYPE(PHIDCLASS_RFID)
	TESTATTACHED

	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_RFID_2OUTPUT:
			TESTRANGE(PFALSE, PTRUE)

			if(CPhidget_statusFlagIsSet(phid->phid.status, PHIDGET_REMOTE_FLAG))
				ADDNETWORKKEY(LEDOn, "%d", ledState);
			else
			{
				SENDPACKET(RFID, ledState);
				//echo back state if the device doesn't
				if (!(phid->fullStateEcho))
					phid->ledEchoState = newVal;
			}
			return EPHIDGET_OK;
		case PHIDID_RFID:
		default:
			return EPHIDGET_UNSUPPORTED;
	}
}

CGET(RFID, LastTag, unsigned char)
	TESTPTRS(phid,pVal) 
	TESTDEVICETYPE(PHIDCLASS_RFID)
	TESTATTACHED

	//if it's all 0's - it's not yet available
	if(!memcmp("\0\0\0\0\0", phid->lastTag, 5))
		return EPHIDGET_UNKNOWNVAL;

	memcpy(pVal, phid->lastTag, 5);

	return EPHIDGET_OK;
}

CGET(RFID, TagStatus, int)
	TESTPTRS(phid,pVal) 
	TESTDEVICETYPE(PHIDCLASS_RFID)
	TESTATTACHED
	TESTMASGN(tagPresent, PUNK_BOOL)

	MASGN(tagPresent)
}

// === Deprecated Functions === //

CGET(RFID,NumOutputs,int)
	return CPhidgetRFID_getOutputCount(phid, pVal);
}

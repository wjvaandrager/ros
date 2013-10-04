#include "stdafx.h"
#include "phidgetinterface.h"
#include "cphidgetlist.h"
#include "pdictserver.h"
#include "pdict.h"
#include "utils.h"
#include "eventhandlers.h"
#include "PhidgetWebservice21.h"
#ifdef USE_ZEROCONF
#include "zeroconf.h"
#endif

pthread_mutex_t PhidgetsAndClientsMutex;
CClientListHandle ConnectedClients = NULL;
CNetworkPhidgetListHandle OpenPhidgets = NULL;

CPhidgetManagerHandle phidm;

int Initialized = PFALSE;
regex_t phidgetsetex;
regex_t phidgetopencloseex;

int CNetworkPhidgetInfo_areEqual(void *arg1, void *arg2)
{
	CNetworkPhidgetInfoHandle phid1 = (CNetworkPhidgetInfoHandle)arg1;
	CNetworkPhidgetInfoHandle phid2 = (CNetworkPhidgetInfoHandle)arg2;
	
	if(!phid1||!phid2||!phid1->phidget||!phid2->phidget)
		return EPHIDGET_INVALIDARG;
		
	return(CPhidget_areEqual(phid1->phidget, phid2->phidget));
}

void CNetworkPhidgetInfo_free(void *arg)
{
	CNetworkPhidgetInfoHandle phid = (CNetworkPhidgetInfoHandle)arg;
	
	CList_emptyList((CListHandle *)&phid->clients, PFALSE, NULL);
	CPhidget_free(phid->phidget); phid->phidget = NULL;
	free(phid);
	
	return;
}

void CClientInfo_free(void *arg)
{
	CClientInfoHandle client = (CClientInfoHandle)arg;
	
	CList_emptyList((CListHandle *)&client->phidgets, PFALSE, NULL);
	if(client->client->port) free(client->client->port); client->client->port=NULL;
	if(client->client->address) free(client->client->address); client->client->address=NULL;
	CPhidgetSocketClient_free(client->client); client->client = NULL;
	free(client);
	
	return;

}

int CClientInfo_areEqual(void *arg1, void *arg2)
{
	CClientInfoHandle client1 = (CClientInfoHandle)arg1;
	CClientInfoHandle client2 = (CClientInfoHandle)arg2;
	
	if(!client1||!client2||!client1->client||!client2->client)
		return EPHIDGET_INVALIDARG;
		
	return(CPhidgetSocketClient_areEqual(client1->client, client2->client));
}


int findNetworkPhidgetInfoHandleInList(CNetworkPhidgetListHandle list, long serialNumber, int deviceID, CNetworkPhidgetInfoHandle *phid)
{
	CPhidgetHandle phidtemp;
	CNetworkPhidgetInfo netphidtemp2;
	int result;
	
	if((result = CPhidget_create(&phidtemp))) return result;
	
	netphidtemp2.phidget = phidtemp;

	phidtemp->deviceID = deviceID;
	phidtemp->serialNumber = serialNumber;
	phidtemp->specificDevice = (serialNumber == -1) ? 0 : 1;
	
	result = CList_findInList((CListHandle)list, &netphidtemp2, CNetworkPhidgetInfo_areEqual, (void **)phid);
	
	CPhidget_free(phidtemp); phidtemp = NULL;
	return result;
}

int findClientInfoHandleInList(CClientListHandle list, const char *ipaddr, const char *port, CClientInfoHandle *client)
{
	CPhidgetSocketClientHandle clienttemp;
	CClientInfo netclienttemp2;
	int result;

	if((result = CPhidgetSocketClient_create(&clienttemp))) return result;
	
	netclienttemp2.client = clienttemp;
	
	if(!(clienttemp->address = strdup(ipaddr)))
		return EPHIDGET_NOMEMORY;
	if(!(clienttemp->port = strdup(port)))
		return EPHIDGET_NOMEMORY;
		
	result = CList_findInList((CListHandle)list, &netclienttemp2, CClientInfo_areEqual, (void **)client);

	//have to free these here because I created them here
	//trying to free them in the CPhidgetSocketClient_free (which is in the dll) causes heap unhappiness
	free(clienttemp->address); clienttemp->address = NULL;
	free(clienttemp->port); clienttemp->port = NULL;
	
	CPhidgetSocketClient_free(clienttemp); clienttemp = NULL;
	return result;
}

int kill_event_handlers(CPhidgetHandle phid)
{
	switch(phid->deviceID)
	{
		case PHIDCLASS_ACCELEROMETER: 
			CPhidgetAccelerometer_set_OnAccelerationChange_Handler((CPhidgetAccelerometerHandle)phid, NULL, NULL);
			break; 
		case PHIDCLASS_ADVANCEDSERVO: 
			CPhidgetAdvancedServo_set_OnPositionChange_Handler((CPhidgetAdvancedServoHandle)phid, NULL, NULL);
			CPhidgetAdvancedServo_set_OnVelocityChange_Handler((CPhidgetAdvancedServoHandle)phid, NULL, NULL);
			CPhidgetAdvancedServo_set_OnCurrentChange_Handler((CPhidgetAdvancedServoHandle)phid, NULL, NULL);
			break;
		case PHIDCLASS_ENCODER: 
			CPhidgetEncoder_set_OnInputChange_Handler((CPhidgetEncoderHandle)phid, NULL, NULL);
			CPhidgetEncoder_set_OnPositionChange_Handler((CPhidgetEncoderHandle)phid, NULL, NULL);
			break;
		/*case PHIDCLASS_GPS: 
			CPhidgetGPS_set_OnNMEAData_Handler((CPhidgetGPSHandle)phid, NULL, NULL);
			break;
		case PHIDCLASS_GYROSCOPE: 
			CPhidgetGyroscope_set_OnAngularRateChange_Handler((CPhidgetGyroscopeHandle)phid, NULL, NULL);
			break;*/
		case PHIDCLASS_INTERFACEKIT:
			CPhidgetInterfaceKit_set_OnInputChange_Handler((CPhidgetInterfaceKitHandle)phid, NULL, NULL);
			CPhidgetInterfaceKit_set_OnOutputChange_Handler((CPhidgetInterfaceKitHandle)phid, NULL, NULL);
			CPhidgetInterfaceKit_set_OnSensorChange_Handler((CPhidgetInterfaceKitHandle)phid, NULL, NULL);
			break;
		case PHIDCLASS_LED: 
			break;
		case PHIDCLASS_MOTORCONTROL: 
			CPhidgetMotorControl_set_OnInputChange_Handler((CPhidgetMotorControlHandle)phid, NULL, NULL);
			CPhidgetMotorControl_set_OnVelocityChange_Handler((CPhidgetMotorControlHandle)phid, NULL, NULL);
			CPhidgetMotorControl_set_OnCurrentChange_Handler((CPhidgetMotorControlHandle)phid, NULL, NULL);
			break;
		case PHIDCLASS_PHSENSOR: 
			CPhidgetPHSensor_set_OnPHChange_Handler((CPhidgetPHSensorHandle)phid, NULL, NULL);
			break;
		case PHIDCLASS_RFID: 
			CPhidgetRFID_set_OnTag_Handler((CPhidgetRFIDHandle)phid, NULL, NULL);
			CPhidgetRFID_set_OnTagLost_Handler((CPhidgetRFIDHandle)phid, NULL, NULL);
			CPhidgetRFID_set_OnOutputChange_Handler((CPhidgetRFIDHandle)phid, NULL, NULL);
			break;
		case PHIDCLASS_SERVO: 
			CPhidgetServo_set_OnPositionChange_Handler((CPhidgetServoHandle)phid, NULL, NULL);
			break;
		case PHIDCLASS_STEPPER: 
			CPhidgetStepper_set_OnInputChange_Handler((CPhidgetStepperHandle)phid, NULL, NULL);
			CPhidgetStepper_set_OnPositionChange_Handler((CPhidgetStepperHandle)phid, NULL, NULL);
			CPhidgetStepper_set_OnVelocityChange_Handler((CPhidgetStepperHandle)phid, NULL, NULL);
			CPhidgetStepper_set_OnCurrentChange_Handler((CPhidgetStepperHandle)phid, NULL, NULL);
			break;
		case PHIDCLASS_TEMPERATURESENSOR: 
			CPhidgetTemperatureSensor_set_OnTemperatureChange_Handler((CPhidgetTemperatureSensorHandle)phid, NULL, NULL);
			break;
		case PHIDCLASS_TEXTLCD: 
			break;
		case PHIDCLASS_TEXTLED: 
			break;
		case PHIDCLASS_WEIGHTSENSOR: 
			CPhidgetWeightSensor_set_OnWeightChange_Handler((CPhidgetWeightSensorHandle)phid, NULL, NULL);
			break;
		default:
			break;
	}
	
	CPhidget_set_OnAttach_Handler(phid, NULL, NULL);
	CPhidget_set_OnDetach_Handler(phid, NULL, NULL);
	
	return EPHIDGET_OK;
}

int close_phidget(void *pdss, int deviceID, long serialNumber, const char *ipaddr, const char *port) {
	int result;
	CNetworkPhidgetInfoHandle newPhid;
	CClientInfoHandle newClient;

	DPRINT("In close Phidget");

	result = findNetworkPhidgetInfoHandleInList(OpenPhidgets, serialNumber, deviceID, &newPhid);
		
	switch(result)
	{
		case EPHIDGET_OK: //device was found
		
			if(!findClientInfoHandleInList(newPhid->clients, ipaddr, port, &newClient))
			{
				/* Here we remove this client from this Phidget's client list */
				CList_removeFromList((CListHandle *)&newPhid->clients, newClient, CClientInfo_areEqual, PFALSE, NULL);
				/* Here we remove this phidget from this client's phidget list */
				CList_removeFromList((CListHandle *)&newClient->phidgets, newPhid, CNetworkPhidgetInfo_areEqual, PFALSE, NULL);
			}
			else
			{
				return EPHIDGET_NOTATTACHED; //this phidget is not opened by this client
			}

			/* no more clients, we can close */
			if(!newPhid->clients)
			{
				int ret;
				char key[MAX_KEY_SIZE];

				snprintf(key, MAX_KEY_SIZE, "^/P[CS]K/%s/%d", newPhid->phidget->deviceType, newPhid->phidget->serialNumber);
				if((ret = remove_key(pdss, key)))
					return ret;

				kill_event_handlers(newPhid->phidget);

				//why do I unlock this?????
				//because if there is a read waiting to add a key, this will deadlock waiting for the read thread to exit
				//but this is not safe! because another close can come along and claim pd_mutex, and then deadlock waiting for 
				//PhidgetsAndClientsMutex, which is owned by this thread, which will now be waiting for pd_mutex.
				//so: we are moving this to the open_close event handler - so that pd_mutex is unlocked before PhidgetsAndClientsMutex is locked.
				//pd_unlock((void *)&pd_mutex);
				DPRINT("Actually closing Phidget");
				CPhidget_close(newPhid->phidget);
				CList_removeFromList((CListHandle *)&OpenPhidgets, newPhid, CNetworkPhidgetInfo_areEqual, PTRUE, CNetworkPhidgetInfo_free);
				//pd_lock((void *)&pd_mutex);
			}
			break;
		case EPHIDGET_NOTFOUND:
			return EPHIDGET_NOTFOUND;
		default:
			return result;
	}
	
	/* free this client if it doesn't have any more open phidgets */
	if(!newClient->phidgets)
		CList_removeFromList((CListHandle *)&ConnectedClients, newClient, CClientInfo_areEqual, PTRUE, CClientInfo_free);
	
	return EPHIDGET_OK;
}

int open_phidget(void *pdss, int deviceID, long serialNumber, const char *ipaddr, const char *port) {
	int result, result2;
	CNetworkPhidgetInfoHandle newNetPhid;
	CClientInfoHandle newClient;

	result = findNetworkPhidgetInfoHandleInList(OpenPhidgets, serialNumber, deviceID, &newNetPhid);
	result2 = findClientInfoHandleInList(ConnectedClients, ipaddr, port, &newClient);
	
	DPRINT("Open Phidget");

	switch(result)
	{
		case EPHIDGET_NOTFOUND:
		{
			CPhidgetHandle newPhid;
			switch(deviceID)
			{
			case PHIDCLASS_ACCELEROMETER: 
				CPhidgetAccelerometer_create((CPhidgetAccelerometerHandle *)&newPhid);
				CPhidgetAccelerometer_set_OnAccelerationChange_Handler((CPhidgetAccelerometerHandle)newPhid, Accelerometer_AccelerationChange, pdss);
				break; 
			case PHIDCLASS_ADVANCEDSERVO: 
				CPhidgetAdvancedServo_create((CPhidgetAdvancedServoHandle *)&newPhid);
				CPhidgetAdvancedServo_set_OnPositionChange_Handler((CPhidgetAdvancedServoHandle)newPhid, AdvancedServo_PositionChange, pdss);
				CPhidgetAdvancedServo_set_OnVelocityChange_Handler((CPhidgetAdvancedServoHandle)newPhid, AdvancedServo_VelocityChange, pdss);
				CPhidgetAdvancedServo_set_OnCurrentChange_Handler((CPhidgetAdvancedServoHandle)newPhid, AdvancedServo_CurrentChange, pdss);
				break;
			case PHIDCLASS_ENCODER: 
				CPhidgetEncoder_create((CPhidgetEncoderHandle *)&newPhid);
				CPhidgetEncoder_set_OnInputChange_Handler((CPhidgetEncoderHandle)newPhid, Encoder_InputChange, pdss);
				CPhidgetEncoder_set_OnPositionChange_Handler((CPhidgetEncoderHandle)newPhid, Encoder_PositionChange, pdss);
				break;
			/*case PHIDCLASS_GPS: 
				CPhidgetGPS_create((CPhidgetGPSHandle *)&newPhid);
				CPhidgetGPS_set_OnNMEAData_Handler((CPhidgetGPSHandle)newPhid, GPS_NMEAData, pdss);
				break;
			case PHIDCLASS_GYROSCOPE: 
				CPhidgetGyroscope_create((CPhidgetGyroscopeHandle *)&newPhid);
				CPhidgetGyroscope_set_OnAngularRateChange_Handler((CPhidgetGyroscopeHandle)newPhid, Gyroscope_AngularRateChange, pdss);
				break;*/
			case PHIDCLASS_INTERFACEKIT:
				CPhidgetInterfaceKit_create((CPhidgetInterfaceKitHandle *)&newPhid);
				CPhidgetInterfaceKit_set_OnInputChange_Handler((CPhidgetInterfaceKitHandle)newPhid, InterfaceKit_InputChange, pdss);
				CPhidgetInterfaceKit_set_OnOutputChange_Handler((CPhidgetInterfaceKitHandle)newPhid, InterfaceKit_OutputChange, pdss);
				CPhidgetInterfaceKit_set_OnSensorChange_Handler((CPhidgetInterfaceKitHandle)newPhid, InterfaceKit_SensorChange, pdss);
				break;
			case PHIDCLASS_LED: 
				CPhidgetLED_create((CPhidgetLEDHandle *)&newPhid);
				break;
			case PHIDCLASS_MOTORCONTROL: 
				CPhidgetMotorControl_create((CPhidgetMotorControlHandle *)&newPhid);
				CPhidgetMotorControl_set_OnInputChange_Handler((CPhidgetMotorControlHandle)newPhid, MotorControl_InputChange, pdss);
				CPhidgetMotorControl_set_OnVelocityChange_Handler((CPhidgetMotorControlHandle)newPhid, MotorControl_VelocityChange, pdss);
				CPhidgetMotorControl_set_OnCurrentChange_Handler((CPhidgetMotorControlHandle)newPhid, MotorControl_CurrentChange, pdss);
				break;
			case PHIDCLASS_PHSENSOR: 
				CPhidgetPHSensor_create((CPhidgetPHSensorHandle *)&newPhid);
				CPhidgetPHSensor_set_OnPHChange_Handler((CPhidgetPHSensorHandle)newPhid, PHSensor_PHChange, pdss);
				break;
			case PHIDCLASS_RFID: 
				CPhidgetRFID_create((CPhidgetRFIDHandle *)&newPhid);
				CPhidgetRFID_set_OnTag_Handler((CPhidgetRFIDHandle)newPhid, RFID_Tag, pdss);
				CPhidgetRFID_set_OnTagLost_Handler((CPhidgetRFIDHandle)newPhid, RFID_TagLost, pdss);
				CPhidgetRFID_set_OnOutputChange_Handler((CPhidgetRFIDHandle)newPhid, RFID_OutputChange, pdss);
				break;
			case PHIDCLASS_SERVO: 
				CPhidgetServo_create((CPhidgetServoHandle *)&newPhid);
				CPhidgetServo_set_OnPositionChange_Handler((CPhidgetServoHandle)newPhid, Servo_PositionChange, pdss);
				break;
			case PHIDCLASS_STEPPER: 
				CPhidgetStepper_create((CPhidgetStepperHandle *)&newPhid);
				CPhidgetStepper_set_OnInputChange_Handler((CPhidgetStepperHandle)newPhid, Stepper_InputChange, pdss);
				CPhidgetStepper_set_OnPositionChange_Handler((CPhidgetStepperHandle)newPhid, Stepper_PositionChange, pdss);
				CPhidgetStepper_set_OnVelocityChange_Handler((CPhidgetStepperHandle)newPhid, Stepper_VelocityChange, pdss);
				CPhidgetStepper_set_OnCurrentChange_Handler((CPhidgetStepperHandle)newPhid, Stepper_CurrentChange, pdss);
				break;
			case PHIDCLASS_TEMPERATURESENSOR: 
				CPhidgetTemperatureSensor_create((CPhidgetTemperatureSensorHandle *)&newPhid);
				CPhidgetTemperatureSensor_set_OnTemperatureChange_Handler((CPhidgetTemperatureSensorHandle)newPhid, TemperatureSensor_TemperatureChange, pdss);
				break;
			case PHIDCLASS_TEXTLCD: 
				CPhidgetTextLCD_create((CPhidgetTextLCDHandle *)&newPhid);
				break;
			case PHIDCLASS_TEXTLED: 
				CPhidgetTextLED_create((CPhidgetTextLEDHandle *)&newPhid);
				break;
			case PHIDCLASS_WEIGHTSENSOR: 
				CPhidgetWeightSensor_create((CPhidgetWeightSensorHandle *)&newPhid);
				CPhidgetWeightSensor_set_OnWeightChange_Handler((CPhidgetWeightSensorHandle)newPhid, WeightSensor_WeightChange, pdss);
				break;
			default:
				break;
			}
			
			CPhidget_set_OnAttach_Handler(newPhid, attach_handler, pdss);
			CPhidget_set_OnDetach_Handler(newPhid, detach_handler, pdss);
			
			if(!(newNetPhid = malloc(sizeof(CNetworkPhidgetInfo))))
			{
				return EPHIDGET_NOMEMORY;
			}
			ZEROMEM(newNetPhid, sizeof(CNetworkPhidgetInfo));
			
			newNetPhid->phidget = newPhid;

			CPhidget_open(newPhid, serialNumber);
				
			CList_addToList((CListHandle *)&OpenPhidgets, newNetPhid, CNetworkPhidgetInfo_areEqual);
		}
		case EPHIDGET_OK: //device was found
			switch(result2)
			{
				case EPHIDGET_NOTFOUND: /* if the client wasn't found in the client list, add it here */
				{
					if(!(newClient = malloc(sizeof(CClientInfo))))
					{
						return EPHIDGET_NOMEMORY;
					}
					ZEROMEM(newClient, sizeof(CClientInfo));
					if((result = CPhidgetSocketClient_create(&newClient->client)))
					{
						return result;
					}
					if(!(newClient->client->address = strdup(ipaddr)))
					{
						return EPHIDGET_NOMEMORY;
					}
					if(!(newClient->client->port = strdup(port)))
					{
						return EPHIDGET_NOMEMORY;
					}
						
					CList_addToList((CListHandle *)&ConnectedClients, newClient, CClientInfo_areEqual);
				}
				case EPHIDGET_OK:
					CList_addToList((CListHandle *)&newNetPhid->clients, newClient, CClientInfo_areEqual);
					CList_addToList((CListHandle *)&newClient->phidgets, newNetPhid, CNetworkPhidgetInfo_areEqual);
					break;
				default:
					return result2;
			}
			break;
		default:
			return EPHIDGET_UNEXPECTED;
	}

	return EPHIDGET_OK;
}

int CCONV manager_attach_handler(CPhidgetHandle phid, void *pdss) {
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/List/%s/%d", 
	phid->deviceType, phid->serialNumber);
	snprintf(val, MAX_VAL_SIZE, "Attached Version=%d ID=%d Label=%s", phid->deviceVersion, phid->deviceIDSpec, phid->label);
	if((ret = add_key(pdss, key, val))) 
		return ret;
	
#ifdef USE_ZEROCONF
	zeroconf_advertise_phidget(phid);
#endif

	return EPHIDGET_OK;
}

int CCONV manager_detach_handler(CPhidgetHandle phid, void *pdss) {
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/List/%s/%d", 
	phid->deviceType, phid->serialNumber);
	snprintf(val, MAX_VAL_SIZE, "Detached Version=%d ID=%d Label=%s", phid->deviceVersion, phid->deviceIDSpec, phid->label);
	if((ret = add_key(pdss, key, val))) 
		return ret;

#ifdef USE_ZEROCONF
	zeroconf_unadvertise_phidget(phid);
#endif

	return EPHIDGET_OK;
}

int start_phidget(pds_session_t *pdss)
{
	int res;
	const char *setpattern = "^/PCK/([a-zA-Z_0-9]*)/([0-9]*)/([a-zA-Z_0-9]*)/?([a-zA-Z_0-9]*)/?([a-zA-Z_0-9]*)$";
	const char *openclosepattern = "^/PCK/Client/([0-9\\.]*)/([0-9]*)/?([a-zA-Z_0-9]*)/?([a-zA-Z_0-9]*)$";
	if(!Initialized)
	{
		if ((res = regcomp(&phidgetsetex, setpattern, REG_EXTENDED)) != 0) {
			fprintf(stderr, "set command pattern compilation error %d\n",
				res);
			abort();
		}
		if ((res = regcomp(&phidgetopencloseex, openclosepattern, REG_EXTENDED)) != 0) {
			fprintf(stderr, "openclose command pattern compilation error %d\n",
				res);
			abort();
		}
		if(pthread_mutex_init(&PhidgetsAndClientsMutex, NULL) != 0)
			return EPHIDGET_UNEXPECTED;
		Initialized = PTRUE;
		add_listener(pdss, "^/PCK/Phidget", phidget_set, pdss);
		add_listener(pdss, "^/PCK/Client/", phidget_openclose, pdss);

#ifdef USE_ZEROCONF
		zeroconf_advertise_ws();
#endif
		CPhidgetManager_create(&phidm);
		CPhidgetManager_set_OnAttach_Handler(phidm, manager_attach_handler, pdss);
		CPhidgetManager_set_OnDetach_Handler(phidm, manager_detach_handler, pdss);
		CPhidgetManager_open(phidm);
	}
	return 0;
}

int stop_phidgets()
{
	if(Initialized)
	{
		CPhidgetManager_close(phidm);
		CPhidgetManager_delete(phidm);
		
#ifdef USE_ZEROCONF
		UninitializeZeroconf();
#endif
	}
	return 0;
}

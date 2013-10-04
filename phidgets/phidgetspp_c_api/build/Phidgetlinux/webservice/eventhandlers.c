/* All event handlers for the various devices */
#include "stdafx.h"
#include "phidgetinterface.h"
#include "cphidgetlist.h"
#include "pdictserver.h"
#include "pdict.h"
#include "utils.h"
#include "eventhandlers.h"
#include "PhidgetWebservice21.h"
#include "zeroconf.h"

/* 
	Initial key setup
	We send every key that wouldn't be sent by an event handler - those are sent by initial events
	We set initKeys to every key we send, plus the initial events
*/
int phidget_accelerometer_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetAccelerometerHandle phid = (CPhidgetAccelerometerHandle)arg;

	int i=0;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfAxes", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.accelerometer.numAxis);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/AccelerationMin", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->accelerationMin);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/AccelerationMax", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->accelerationMax);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	*initKeys += 3;

	for (i = 0; i<phid->phid.attr.accelerometer.numAxis; i++)
	{
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Trigger/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%lE", phid->axisChangeTrigger[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		(*initKeys)++;
	}

	/* acceleration events */
	*initKeys += phid->phid.attr.accelerometer.numAxis;

	return EPHIDGET_OK;
}
int phidget_advancedservo_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetAdvancedServoHandle phid = (CPhidgetAdvancedServoHandle)arg;

	int i=0;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfMotors", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.advancedservo.numMotors);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/AccelerationMin", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->accelerationMin);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/AccelerationMax", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->accelerationMax);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/VelocityMin", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->velocityMin);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/VelocityMaxLimit", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->velocityMaxLimit);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PositionMinLimit", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->motorPositionMinLimit);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PositionMaxLimit", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->motorPositionMaxLimit);
	if((ret = add_key(pdss, key, val))) 
		return ret;
	
	*initKeys += 7;

	for(i=0;i<phid->phid.attr.advancedservo.numMotors;i++)
	{
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PositionMin/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%lE", phid->motorPositionMin[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PositionMax/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%lE", phid->motorPositionMax[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;
		
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Engaged/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%d", phid->motorEngagedStateEcho[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/SpeedRampingOn/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%d", phid->motorSpeedRampingStateEcho[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Stopped/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%d", phid->motorStoppedState[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		/* if the motor isn't engaged, there wouldn't be position events */
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Position/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%lE", phid->motorPositionEcho[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/ServoParameters/%d", phid->phid.deviceType, phid->phid.serialNumber,i);
		snprintf(val, MAX_VAL_SIZE, "%d,%lE,%lE,%lE,%lE", phid->servoParams[i].servoType, phid->servoParams[i].min_us, 
			phid->servoParams[i].max_us, phid->servoParams[i].us_per_degree, phid->servoParams[i].max_us_per_s);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/VelocityMax/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%lE", phid->velocityMax[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		*initKeys += 8;
	}

	/* Velocity, Current events */
	*initKeys += (2 * phid->phid.attr.advancedservo.numMotors);

	return EPHIDGET_OK;
}
int phidget_encoder_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetEncoderHandle phid = (CPhidgetEncoderHandle)arg;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfEncoders", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.encoder.numEncoders);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfInputs", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.encoder.numInputs);
	if((ret = add_key(pdss, key, val))) 
		return ret;
	
	*initKeys += 2;

	return EPHIDGET_OK;
}
/*int phidget_gps_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetGPSHandle phid = (CPhidgetGPSHandle)arg;
	
	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Trigger", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->PositionChangeTrigger);
	if((ret = add_key(pdss, key, val))) 
		return ret;
	
	*initKeys += 1;

	return EPHIDGET_OK;
}
int phidget_gyroscope_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetGyroscopeHandle phid = (CPhidgetGyroscopeHandle)arg;

	int i = 0;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfAxes", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.gyroscope.numAxis);
	if((ret = add_key(pdss, key, val))) 
		return ret;
	
	*initKeys += 1;

	for(i=0;i<phid->phid.attr.gyroscope.numAxis;i++)
	{
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Trigger/%d", phid->phid.deviceType, phid->phid.serialNumber,i);
		snprintf(val, MAX_VAL_SIZE, "%lE", phid->axisChangeTrigger[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;
		
		*initKeys += 1;
	}
	
	// angular rate change events
	*initKeys += phid->phid.attr.gyroscope.numAxis;

	return EPHIDGET_OK;
}*/
int phidget_interfacekit_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetInterfaceKitHandle phid = (CPhidgetInterfaceKitHandle)arg;

	int i = 0;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfSensors", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.ifkit.numSensors);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfInputs", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.ifkit.numInputs);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfOutputs", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.ifkit.numOutputs);
	if((ret = add_key(pdss, key, val))) 
		return ret;
	
	*initKeys += 3;


	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_INTERFACEKIT_8_8_8:
		case PHIDID_INTERFACEKIT_8_8_8_w_LCD:
			snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Ratiometric", phid->phid.deviceType, phid->phid.serialNumber);
			snprintf(val, MAX_VAL_SIZE, "%d", phid->ratiometric);
			if((ret = add_key(pdss, key, val))) 
				return ret;

			*initKeys += 1;

			break;
		default:
			break;
	}

	for(i=0;i<phid->phid.attr.ifkit.numSensors;i++)
	{
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Trigger/%d", phid->phid.deviceType, phid->phid.serialNumber,i);
		snprintf(val, MAX_VAL_SIZE, "%d", phid->sensorChangeTrigger[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/RawSensor/%d", phid->phid.deviceType, phid->phid.serialNumber,i);
		snprintf(val, MAX_VAL_SIZE, "%d", phid->sensorRawValue[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		*initKeys += 2;

		//These can have unknown sensor value, so we have to send it out manually as it wouldn't be sent out in the event
		switch(phid->phid.deviceIDSpec)
		{
			case PHIDID_ROTARY_TOUCH:
			case PHIDID_LINEAR_TOUCH:
				snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Sensor/%d", phid->phid.deviceType, phid->phid.serialNumber,i);
				snprintf(val, MAX_VAL_SIZE, "%d", phid->sensorValue[i]);
				if((ret = add_key(pdss, key, val))) 
					return ret;

				//initKeys incremented below for sensors

				break;
			default:
				break;
		}

	}

	/* have to send these explicitely because events are not guaranteed (for older ifkits) */
	for(i=0;i<phid->phid.attr.ifkit.numOutputs;i++)
	{
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Output/%d", phid->phid.deviceType, phid->phid.serialNumber,i);
		snprintf(val, MAX_VAL_SIZE, "%d", phid->outputEchoStates[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		*initKeys += 1;
	}

	/* input, sensor events */
	*initKeys += phid->phid.attr.ifkit.numSensors;
	*initKeys += phid->phid.attr.ifkit.numInputs;

	return EPHIDGET_OK;
}
int phidget_led_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetLEDHandle phid = (CPhidgetLEDHandle)arg;

	int i = 0;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfLEDs", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.led.numLEDs);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	*initKeys += 1;

	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_LED_64_ADV:
			snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Voltage", phid->phid.deviceType, phid->phid.serialNumber);
			snprintf(val, MAX_VAL_SIZE, "%d", phid->voltage);
			if((ret = add_key(pdss, key, val))) 
				return ret;

			snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/CurrentLimit", phid->phid.deviceType, phid->phid.serialNumber);
			snprintf(val, MAX_VAL_SIZE, "%d", phid->currentLimit);
			if((ret = add_key(pdss, key, val))) 
				return ret;

			*initKeys += 2;

			break;
		default:
			break;
	}

	for(i=0;i<phid->phid.attr.led.numLEDs;i++)
	{
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Brightness/%d", phid->phid.deviceType, phid->phid.serialNumber,i);
		snprintf(val, MAX_VAL_SIZE, "%d", phid->LED_Power[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		*initKeys += 1;
	}

	return EPHIDGET_OK;
}
int phidget_motorcontrol_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetMotorControlHandle phid = (CPhidgetMotorControlHandle)arg;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfMotors", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.motorcontrol.numMotors);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfInputs", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.motorcontrol.numInputs);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/AccelerationMin", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->accelerationMin);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/AccelerationMax", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->accelerationMax);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	*initKeys += 4;
	
	/* velocity events */
	*initKeys += phid->phid.attr.motorcontrol.numMotors;
	/* input events */
	*initKeys += phid->phid.attr.motorcontrol.numInputs;
	/* current events */
	if(phid->phid.deviceIDSpec == PHIDID_MOTORCONTROL_HC_2MOTOR)
		*initKeys += phid->phid.attr.motorcontrol.numMotors;

	return EPHIDGET_OK;
}
int phidget_phsensor_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetPHSensorHandle phid = (CPhidgetPHSensorHandle)arg;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PHMin", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->phMin);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PHMax", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->phMax);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PotentialMin", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->potentialMin);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PotentialMax", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->potentialMax);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PH", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->PH);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Potential", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->Potential);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Trigger", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->PHChangeTrigger);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	*initKeys += 7;

	return EPHIDGET_OK;
}
int phidget_rfid_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetRFIDHandle phid = (CPhidgetRFIDHandle)arg;

	int i = 0;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfOutputs", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.rfid.numOutputs);
	if((ret = add_key(pdss, key, val))) 
		return ret;
	
	/* if tagState is true, send tag, else send tagState */
	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/TagState", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->tagPresent);
	if((ret = add_key(pdss, key, val))) 
		return ret;
	
	*initKeys += 2;

	/* have to send these explicitely because events are not guaranteed (for older readers) */
	for(i=0;i<phid->phid.attr.rfid.numOutputs;i++)
	{
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Output/%d", phid->phid.deviceType, phid->phid.serialNumber,i);
		snprintf(val, MAX_VAL_SIZE, "%d", phid->outputEchoState[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		*initKeys += 1;
	}

	if(phid->phid.attr.rfid.numOutputs > 0)
	{
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/AntennaOn", phid->phid.deviceType, phid->phid.serialNumber);
		snprintf(val, MAX_VAL_SIZE, "%d", phid->antennaEchoState);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/LEDOn", phid->phid.deviceType, phid->phid.serialNumber);
		snprintf(val, MAX_VAL_SIZE, "%d", phid->ledEchoState);
		if((ret = add_key(pdss, key, val))) 
			return ret;
		
		*initKeys += 2;
	}

	return EPHIDGET_OK;
}
int phidget_servo_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetServoHandle phid = (CPhidgetServoHandle)arg;

	int i = 0;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfMotors", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.servo.numMotors);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PositionMinLimit", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->motorPositionMinLimit);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PositionMaxLimit", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->motorPositionMaxLimit);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	*initKeys += 3;

	/* have to send these explicitely because events are not guaranteed (for older servo boards) */
	for(i=0;i<phid->phid.attr.servo.numMotors;i++)
	{
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Position/%d", phid->phid.deviceType, phid->phid.serialNumber,i);
		snprintf(val, MAX_VAL_SIZE, "%lE", phid->motorPositionEcho[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Engaged/%d", phid->phid.deviceType, phid->phid.serialNumber,i);
		snprintf(val, MAX_VAL_SIZE, "%d", phid->motorEngagedStateEcho[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/ServoParameters/%d", phid->phid.deviceType, phid->phid.serialNumber,i);
		snprintf(val, MAX_VAL_SIZE, "%d,%lE,%lE,%lE", phid->servoParams[i].servoType, phid->servoParams[i].min_us, 
			phid->servoParams[i].max_us, phid->servoParams[i].us_per_degree);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		*initKeys += 3;
	}

	return EPHIDGET_OK;
}
int phidget_stepper_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetStepperHandle phid = (CPhidgetStepperHandle)arg;

	int i = 0;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfMotors", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.stepper.numMotors);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfInputs", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.stepper.numInputs);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PositionMin", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lld", phid->motorPositionMin);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PositionMax", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lld", phid->motorPositionMax);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/AccelerationMin", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->accelerationMin);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/AccelerationMax", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->accelerationMax);
	if((ret = add_key(pdss, key, val)))
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/VelocityMin", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->motorSpeedMin);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/VelocityMax", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->motorSpeedMax);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/CurrentMin", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->currentMin);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/CurrentMax", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->currentMax);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	*initKeys += 10;

	for(i=0;i<phid->phid.attr.stepper.numMotors;i++)
	{
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Engaged/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%d", phid->motorEngagedStateEcho[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Stopped/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%d", phid->motorStoppedState[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		/* if the motor isn't engaged, there wouldn't be position events */
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/CurrentPosition/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%lld", phid->motorPositionEcho[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;
		
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/TargetPosition/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%lld", phid->motorPosition[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		*initKeys += 4;
	}

	/* inputs events */
	*initKeys += phid->phid.attr.stepper.numInputs;
	/* velocity events */
	*initKeys += phid->phid.attr.stepper.numMotors;
	/* current events */
	if(phid->phid.deviceIDSpec == PHIDID_BIPOLAR_STEPPER_1MOTOR)
		*initKeys += phid->phid.attr.stepper.numMotors;

	return EPHIDGET_OK;
}
int phidget_temperaturesensor_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret, i=0;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetTemperatureSensorHandle phid = (CPhidgetTemperatureSensorHandle)arg;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfSensors", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.temperaturesensor.numTempInputs);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PotentialMin", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->potentialMin);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PotentialMax", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->potentialMax);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/AmbientTemperatureMin", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->ambientTemperatureMin);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/AmbientTemperatureMax", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->ambientTemperatureMax);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/AmbientTemperature", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->AmbientTemperature);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	*initKeys += 6;

	for(i=0;i<phid->phid.attr.temperaturesensor.numTempInputs;i++)
	{
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/TemperatureMin/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%lE", phid->temperatureMin[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/TemperatureMax/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%lE", phid->temperatureMax[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Potential/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%lE", phid->Potential[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		/* can't trust the event because it could be out of range */
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Temperature/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%lE", phid->Temperature[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Trigger/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%lE", phid->TempChangeTrigger[i]);
		if((ret = add_key(pdss, key, val))) 
			return ret;
		
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/ThermocoupleType/%d", phid->phid.deviceType, phid->phid.serialNumber, i);
		snprintf(val, MAX_VAL_SIZE, "%d", phid->ThermocoupleType[i]);
		if((ret = add_key(pdss, key, val)))
			return ret;

		*initKeys += 6;
	}

	return EPHIDGET_OK;
}
int phidget_textlcd_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetTextLCDHandle phid = (CPhidgetTextLCDHandle)arg;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfRows", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.textlcd.numRows);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfColumns", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.textlcd.numColumns);
	if((ret = add_key(pdss, key, val))) 
		return ret;
	
	*initKeys += 2;

	return EPHIDGET_OK;
}
int phidget_textled_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetTextLEDHandle phid = (CPhidgetTextLEDHandle)arg;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfRows", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.textled.numRows);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/NumberOfColumns", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->phid.attr.textled.numColumns);
	if((ret = add_key(pdss, key, val))) 
		return ret;
	
	*initKeys += 2;

	return EPHIDGET_OK;
}
int phidget_weightsensor_initkeys(CPhidgetHandle arg, pds_session_t *pdss, int *initKeys)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];
	CPhidgetWeightSensorHandle phid = (CPhidgetWeightSensorHandle)arg;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Weight", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->Weight);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Trigger", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->WeightChangeTrigger);
	if((ret = add_key(pdss, key, val))) 
		return ret;
	
	*initKeys += 2;

	return EPHIDGET_OK;
}

int(*fptrInitKeys[PHIDGET_DEVICE_CLASS_COUNT])(CPhidgetHandle phid, pds_session_t *pdss, int *initKeys) = {
NULL,
NULL,
phidget_accelerometer_initkeys,
phidget_advancedservo_initkeys,
phidget_encoder_initkeys,
NULL,//phidget_gps_initkeys,
NULL,//phidget_gyroscope_initkeys,
phidget_interfacekit_initkeys,
phidget_led_initkeys,
phidget_motorcontrol_initkeys,
phidget_phsensor_initkeys,
phidget_rfid_initkeys,
phidget_servo_initkeys,
phidget_stepper_initkeys,
phidget_temperaturesensor_initkeys,
phidget_textlcd_initkeys,
phidget_textled_initkeys,
phidget_weightsensor_initkeys};

//Events from Phidget21
int CCONV attach_handler(CPhidgetHandle phid, void *pdss)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	const char *name;

	/* we send 6 keys in this function */
	int initKeys = 6;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Label", phid->deviceType, phid->serialNumber);
	if((ret = add_key(pdss, key, phid->label)))
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Version", phid->deviceType, phid->serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->deviceVersion);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Name", phid->deviceType, phid->serialNumber);
	CPhidget_getDeviceName(phid, &name);
	if((ret = add_key(pdss, key, name))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/ID", phid->deviceType, phid->serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", phid->deviceIDSpec);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Status", phid->deviceType, phid->serialNumber);
	snprintf(val, MAX_VAL_SIZE, "Attached");
	if((ret = add_key(pdss, key, val))) 
		return ret;

	/* this also adds to initKeys */
	if((ret = fptrInitKeys[phid->deviceID](phid, pdss, &initKeys)))
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/InitKeys", phid->deviceType, phid->serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%d", initKeys);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return EPHIDGET_OK;
}

int CCONV detach_handler(CPhidgetHandle phid, void *pdss)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Status", phid->deviceType, phid->serialNumber);
	snprintf(val, MAX_VAL_SIZE, "Detached");
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "^/P[CS]K/%s/%d", phid->deviceType, phid->serialNumber);
	if((ret = remove_key(pdss, key)))
		return ret;

	return 0;
}

int CCONV Accelerometer_AccelerationChange(CPhidgetAccelerometerHandle phid, void *pdss, int Index, double Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Acceleration/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%lE", Val);
	if((ret = add_key(pdss, key, val)))
		return ret;

	return 0;
}

int CCONV AdvancedServo_PositionChange(CPhidgetAdvancedServoHandle phid, void *pdss, int Index, double Val)
{
	int ret, stopped;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	CPhidgetAdvancedServo_getStopped(phid, Index, &stopped);

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Position/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%lE", servo_degrees_to_us(phid->servoParams[Index], Val));
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Stopped/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%d", stopped);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV AdvancedServo_VelocityChange(CPhidgetAdvancedServoHandle phid, void *pdss, int Index, double Val)
{
	int ret, stopped;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	CPhidgetAdvancedServo_getStopped(phid, Index, &stopped);

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Velocity/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%lE", servo_degrees_to_us_vel(phid->servoParams[Index], Val));
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Stopped/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%d", stopped);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV AdvancedServo_CurrentChange(CPhidgetAdvancedServoHandle phid, void *pdss, int Index, double Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Current/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%lE", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV Encoder_InputChange(CPhidgetEncoderHandle phid, void *pdss, int Index, int Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Input/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%d", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV Encoder_PositionChange(CPhidgetEncoderHandle phid, void *pdss, int Index, int Time, int PositionChange)
{
	int ret;
	int posn;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	CPhidgetEncoder_getPosition(phid, Index, &posn);

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Position/%d/", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%d/%d/%d", Time, PositionChange, posn);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

/*int CCONV GPS_NMEAData(CPhidgetGPSHandle phid, void *pdss, const char *data)
{
	int ret;
	char key[MAX_KEY_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Data", phid->phid.deviceType, phid->phid.serialNumber);
	if((ret = add_key(pdss, key, data)))
		return ret;

	return 0;
}*/

/*int CCONV Gyroscope_AngularRateChange(CPhidgetGyroscopeHandle phid, void *pdss, int Index, double Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/AngularRate/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%lE", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}*/

int CCONV InterfaceKit_InputChange(CPhidgetInterfaceKitHandle phid, void *pdss, int Index, int Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Input/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%d", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	//special case for touch sensors - if touch is removed, sensor will go unknown
	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_ROTARY_TOUCH:
		case PHIDID_LINEAR_TOUCH:
			snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Sensor/0", phid->phid.deviceType, phid->phid.serialNumber);
			snprintf(val, MAX_VAL_SIZE, "%d", phid->sensorValue[0]);
			if((ret = add_key(pdss, key, val))) 
				return ret;
			snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/RawSensor/0", phid->phid.deviceType, phid->phid.serialNumber);
			snprintf(val, MAX_VAL_SIZE, "%d", phid->sensorRawValue[0]);
			if((ret = add_key(pdss, key, val))) 
				return ret;
			break;
		default:
			break;
	}

	return 0;
}

int CCONV InterfaceKit_OutputChange(CPhidgetInterfaceKitHandle phid, void *pdss, int Index, int Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Output/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%d", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV InterfaceKit_SensorChange(CPhidgetInterfaceKitHandle phid, void *pdss, int Index, int Val)
{
	int ret, rawSensorVal;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	CPhidgetInterfaceKit_getSensorRawValue(phid, Index, &rawSensorVal);

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Sensor/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%d", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/RawSensor/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%d", rawSensorVal);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV MotorControl_InputChange(CPhidgetMotorControlHandle phid, void *pdss, int Index, int Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Input/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%d", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV MotorControl_VelocityChange(CPhidgetMotorControlHandle phid, void *pdss, int Index, double Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Velocity/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%lE", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV MotorControl_CurrentChange(CPhidgetMotorControlHandle phid, void *pdss, int Index, double Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Current/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%lE", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV PHSensor_PHChange(CPhidgetPHSensorHandle phid, void *pdss, double Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PH", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Potential", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->Potential);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV RFID_Tag(CPhidgetRFIDHandle phid, void *pdss, unsigned char *Tag)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Tag", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%02x%02x%02x%02x%02x",Tag[0],Tag[1],Tag[2],Tag[3],Tag[4]);
	if((ret = add_key(pdss, key, val))) 
		return ret;
	
	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/TagState", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "1");
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV RFID_TagLost(CPhidgetRFIDHandle phid, void *pdss, unsigned char *Tag)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/TagLoss", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%02x%02x%02x%02x%02x",Tag[0],Tag[1],Tag[2],Tag[3],Tag[4]);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/TagState", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "0");
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV RFID_OutputChange(CPhidgetRFIDHandle phid, void *pdss, int Index, int Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Output/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%d", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV Servo_PositionChange(CPhidgetServoHandle phid, void *pdss, int Index, double Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Position/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%lE", servo_degrees_to_us(phid->servoParams[Index], Val));
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV Stepper_InputChange(CPhidgetStepperHandle phid, void *pdss, int Index, int Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Input/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%d", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV Stepper_PositionChange(CPhidgetStepperHandle phid, void *pdss, int Index, long long Val)
{
	int ret, stopped;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	CPhidgetStepper_getStopped(phid, Index, &stopped);

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/CurrentPosition/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%lld", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Stopped/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%d", stopped);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV Stepper_VelocityChange(CPhidgetStepperHandle phid, void *pdss, int Index, double Val)
{
	int ret, stopped;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	CPhidgetStepper_getStopped(phid, Index, &stopped);

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Velocity/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%lE", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Stopped/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%d", stopped);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV Stepper_CurrentChange(CPhidgetStepperHandle phid, void *pdss, int Index, double Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Current/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%lE", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}


int CCONV TemperatureSensor_TemperatureChange(CPhidgetTemperatureSensorHandle phid, void *pdss, int Index, double Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Temperature/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%lE", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Potential/%d", phid->phid.deviceType, phid->phid.serialNumber, Index);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->Potential[Index]);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/AmbientTemperature", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", phid->AmbientTemperature);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

int CCONV WeightSensor_WeightChange(CPhidgetWeightSensorHandle phid, void *pdss, double Val)
{
	int ret;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Weight", phid->phid.deviceType, phid->phid.serialNumber);
	snprintf(val, MAX_VAL_SIZE, "%lE", Val);
	if((ret = add_key(pdss, key, val))) 
		return ret;

	return 0;
}

//Events from client (Remote Phidget21)

int phidget_accelerometer_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret;
	CPhidgetAccelerometerHandle accel = (CPhidgetAccelerometerHandle)phid;
	double value = strtod(state, NULL);
	char key[MAX_KEY_SIZE];

	if(!strncmp(setThing, "Trigger", sizeof("Trigger")))
	{
		if((ret = CPhidgetAccelerometer_setAccelerationChangeTrigger(accel, index, value)))
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Trigger/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else
	{
		DPRINT("Bad setType for Accelerometer: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return EPHIDGET_OK;
}

int phidget_advancedservo_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret;
	CPhidgetAdvancedServoHandle advservo = (CPhidgetAdvancedServoHandle)phid;
	double value = strtod(state, NULL);
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	if(!strncmp(setThing, "Acceleration", sizeof("Acceleration")))
	{
		if((ret =  CPhidgetAdvancedServo_setAcceleration(advservo, index, servo_us_to_degrees_vel(advservo->servoParams[index], value, PFALSE))))
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Acceleration/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "VelocityLimit", sizeof("VelocityLimit")))
	{
		if((ret = CPhidgetAdvancedServo_setVelocityLimit(advservo, index, servo_us_to_degrees_vel(advservo->servoParams[index], value, PFALSE))))
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/VelocityLimit/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "PositionMin", sizeof("PositionMin")))
	{
		if((ret = CPhidgetAdvancedServo_setPositionMin(advservo, index, servo_us_to_degrees(advservo->servoParams[index], value, PFALSE))))
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PositionMin/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "PositionMax", sizeof("PositionMax")))
	{
		if((ret = CPhidgetAdvancedServo_setPositionMax(advservo, index, servo_us_to_degrees(advservo->servoParams[index], value, PFALSE))))
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PositionMax/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "Position", sizeof("Position")))
	{
		if((ret = CPhidgetAdvancedServo_setPosition(advservo, index, servo_us_to_degrees(advservo->servoParams[index], value, PFALSE)))) 
			return ret;
	}
	else if(!strncmp(setThing, "Engaged", sizeof("Engaged")))
	{
		int val = strtol(state, NULL, 10);
		if((ret = CPhidgetAdvancedServo_setEngaged(advservo, index, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Engaged/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "SpeedRampingOn", sizeof("SpeedRampingOn")))
	{
		int val = strtol(state, NULL, 10);
		if((ret = CPhidgetAdvancedServo_setSpeedRampingOn(advservo, index, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/SpeedRampingOn/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "ServoParameters", sizeof("ServoParameters")))
	{
		CPhidgetServoParameters params;
		char *endptr;
		params.servoType = strtol(state, &endptr, 10);
		params.min_us = strtod(endptr+1, &endptr);
		params.max_us = strtod(endptr+1, &endptr);
		params.us_per_degree = strtod(endptr+1, &endptr);
		params.max_us_per_s = strtod(endptr+1, NULL);
		params.state = PTRUE;

		ret = setupNewAdvancedServoParams(advservo, index, params);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/ServoParameters/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PositionMinLimit", phid->deviceType, phid->serialNumber);
		snprintf(val, MAX_VAL_SIZE, "%lE", advservo->motorPositionMinLimit);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/VelocityMax", phid->deviceType, phid->serialNumber);
		snprintf(val, MAX_VAL_SIZE, "%lE", advservo->velocityMax);
		if((ret = add_key(pdss, key, val))) 
			return ret;
	}
	else{
		DPRINT("Bad setType for AdvancedServo: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return EPHIDGET_OK;
}

int phidget_encoder_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret = EPHIDGET_OK;
	CPhidgetEncoderHandle encoder = (CPhidgetEncoderHandle)phid;
	int value = strtol(state, NULL, 10);
	char key[MAX_KEY_SIZE];

	if(!strncmp(setThing, "ResetPosition", sizeof("ResetPosition")))
	{
		CPhidgetEncoder_setPosition(encoder, index, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/ResetPosition", phid->deviceType, phid->serialNumber);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else{
		DPRINT("Bad setType for Encoder: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return ret;
}
/*
int phidget_gps_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret = EPHIDGET_OK;
	CPhidgetGPSHandle gps = (CPhidgetGPSHandle)phid;
	double value = strtod(state, NULL);
	char key[MAX_KEY_SIZE];

	if(!strncmp(setThing, "Trigger", sizeof("Trigger")))
	{
		ret = CPhidgetGPS_setPositionChangeTrigger(gps, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Trigger", phid->deviceType, phid->serialNumber);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else{
		DPRINT("Bad setType for GPS: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return ret;
}

int phidget_gyroscope_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret = EPHIDGET_OK;
	CPhidgetGyroscopeHandle gyro = (CPhidgetGyroscopeHandle)phid;
	double value = strtod(state, NULL);
	char key[MAX_KEY_SIZE];

	if(!strncmp(setThing, "Trigger", sizeof("Trigger")))
	{
		ret = CPhidgetGyroscope_setAngularRateChangeTrigger(gyro, index, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Trigger", phid->deviceType, phid->serialNumber);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else{
		DPRINT("Bad setType for Gyroscope: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return ret;
}*/

int phidget_interfacekit_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret;
	CPhidgetInterfaceKitHandle ifkit = (CPhidgetInterfaceKitHandle)phid;
	int value = strtol(state, NULL, 10);
	char key[MAX_KEY_SIZE];

	if(!strncmp(setThing, "Output", sizeof("Output")))
	{
		if((ret = CPhidgetInterfaceKit_setOutputState(ifkit, index, value)))
			return ret;
	}
	else if(!strncmp(setThing, "Ratiometric", sizeof("Ratiometric")))
	{
		if((ret = CPhidgetInterfaceKit_setRatiometric(ifkit, value)))
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Ratiometric", phid->deviceType, phid->serialNumber);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "Trigger", sizeof("Trigger")))
	{
		if((ret = CPhidgetInterfaceKit_setSensorChangeTrigger(ifkit, index, value)))
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Trigger/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else{
		DPRINT("Bad setType for InterfaceKit: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return EPHIDGET_OK;
}

int phidget_led_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret = EPHIDGET_OK;
	CPhidgetLEDHandle led = (CPhidgetLEDHandle)phid;
	int value = strtol(state, NULL, 10);
	char key[MAX_KEY_SIZE];

	if(!strncmp(setThing, "Brightness", sizeof("Brightness")))
	{
		ret = CPhidgetLED_setDiscreteLED(led, index, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Brightness/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	if(!strncmp(setThing, "Voltage", sizeof("Voltage")))
	{
		ret = CPhidgetLED_setVoltage(led, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Voltage", phid->deviceType, phid->serialNumber);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	if(!strncmp(setThing, "CurrentLimit", sizeof("CurrentLimit")))
	{
		ret = CPhidgetLED_setCurrentLimit(led, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/CurrentLimit", phid->deviceType, phid->serialNumber);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else{
		DPRINT("Bad setType for LED: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return ret;
}

int phidget_motorcontrol_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret = EPHIDGET_OK;
	CPhidgetMotorControlHandle motor = (CPhidgetMotorControlHandle)phid;
	double value = strtod(state, NULL);
	char key[MAX_KEY_SIZE];

	if(!strncmp(setThing, "Acceleration", sizeof("Acceleration")))
	{
		ret = CPhidgetMotorControl_setAcceleration(motor, index, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Acceleration/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "Velocity", sizeof("Velocity")))
	{
		ret = CPhidgetMotorControl_setVelocity(motor, index, value);
	}
	else{
		DPRINT("Bad setType for MotorControl: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return ret;
}

int phidget_phsensor_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret = EPHIDGET_OK;
	CPhidgetPHSensorHandle ph = (CPhidgetPHSensorHandle)phid;
	double value = strtod(state, NULL);
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	if(!strncmp(setThing, "Trigger", sizeof("Trigger")))
	{
		ret = CPhidgetPHSensor_setPHChangeTrigger(ph, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Trigger", phid->deviceType, phid->serialNumber);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	if(!strncmp(setThing, "Temperature", sizeof("Temperature")))
	{
		ret = CPhidgetPHSensor_setTemperature(ph, value);
		
		//these will have changed
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PHMin", phid->deviceType, phid->serialNumber);
		snprintf(val, MAX_VAL_SIZE, "%lE", ph->phMin);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PHMax", phid->deviceType, phid->serialNumber);
		snprintf(val, MAX_VAL_SIZE, "%lE", ph->phMax);
		if((ret = add_key(pdss, key, val))) 
			return ret;
	}
	else{
		DPRINT("Bad setType for PHSensor: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return ret;
}

int phidget_rfid_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret = EPHIDGET_OK;
	CPhidgetRFIDHandle rfid = (CPhidgetRFIDHandle)phid;
	int value = strtol(state, NULL, 10);
	char key[MAX_KEY_SIZE];

	if(!strncmp(setThing, "Output", sizeof("Output")))
	{
		ret = CPhidgetRFID_setOutputState(rfid, index, value);
	}
	else if(!strncmp(setThing, "AntennaOn", sizeof("AntennaOn")))
	{
		ret = CPhidgetRFID_setAntennaOn(rfid, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/AntennaOn", phid->deviceType, phid->serialNumber);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "LEDOn", sizeof("LEDOn")))
	{
		ret = CPhidgetRFID_setLEDOn(rfid, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/LEDOn", phid->deviceType, phid->serialNumber);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else{
		DPRINT("Bad setType for RFID: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return ret;
}

int phidget_servo_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret = EPHIDGET_OK;
	CPhidgetServoHandle servo = (CPhidgetServoHandle)phid;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	if(!strncmp(setThing, "Position", sizeof("Position")))
	{
		double value = strtod(state, NULL);
		ret = CPhidgetServo_setPosition(servo, index, servo_us_to_degrees(servo->servoParams[index], value, PFALSE));
	}
	else if(!strncmp(setThing, "Engaged", sizeof("Engaged")))
	{
		int val = strtol(state, NULL, 10);
		//need to convert the us to degrees first
		ret = CPhidgetServo_setEngaged(servo, index, val);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Engaged/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "ServoParameters", sizeof("ServoParameters")))
	{
		CPhidgetServoParameters params;
		char *endptr;
		params.servoType = strtol(state, &endptr, 10);
		params.min_us = strtod(endptr+1, &endptr);
		params.max_us = strtod(endptr+1, &endptr);
		params.us_per_degree = strtod(endptr+1, NULL);
		params.state = PTRUE;

		ret = setupNewServoParams(servo, index, params);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/ServoParameters/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/PositionMinLimit", phid->deviceType, phid->serialNumber);
		snprintf(val, MAX_VAL_SIZE, "%lE", servo->motorPositionMinLimit);
		if((ret = add_key(pdss, key, val))) 
			return ret;
	}
	else{
		DPRINT("Bad setType for Servo: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return ret;
}

int phidget_stepper_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret = EPHIDGET_OK;
	CPhidgetStepperHandle stepper = (CPhidgetStepperHandle)phid;
	char key[MAX_KEY_SIZE];

	if(!strncmp(setThing, "Acceleration", sizeof("Acceleration")))
	{
		double value = strtod(state, NULL);
		ret = CPhidgetStepper_setAcceleration(stepper, index, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Acceleration/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "VelocityLimit", sizeof("VelocityLimit")))
	{
		double value = strtod(state, NULL);
		ret = CPhidgetStepper_setVelocityLimit(stepper, index, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/VelocityLimit/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "TargetPosition", sizeof("TargetPosition")))
	{
#if defined(_WINDOWS)
		__int64 value = (__int64)_strtoi64(state, NULL, 10);
#else
		__int64 value = strtoll(state, NULL, 10);
#endif
		ret = CPhidgetStepper_setTargetPosition(stepper, index, value);
	}
	else if(!strncmp(setThing, "CurrentPosition", sizeof("CurrentPosition")))
	{
#if defined(_WINDOWS)
		__int64 value = (__int64)_strtoi64(state, NULL, 10);
#else
		__int64 value = strtoll(state, NULL, 10);
#endif
		ret = CPhidgetStepper_setCurrentPosition(stepper, index, value);
	}
	else if(!strncmp(setThing, "CurrentLimit", sizeof("CurrentLimit")))
	{
		double value = strtod(state, NULL);
		ret = CPhidgetStepper_setCurrentLimit(stepper, index, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/CurrentLimit/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "Engaged", sizeof("Engaged")))
	{
		long value = strtol(state, NULL, 10);
		ret = CPhidgetStepper_setEngaged(stepper, index, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Engaged/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else{
		DPRINT("Bad setType for Stepper: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return ret;
}

int phidget_temperaturesensor_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret = EPHIDGET_OK;
	CPhidgetTemperatureSensorHandle temperature = (CPhidgetTemperatureSensorHandle)phid;
	char key[MAX_KEY_SIZE];
	char val[MAX_VAL_SIZE];

	if(!strncmp(setThing, "Trigger", sizeof("Trigger")))
	{
		double value = strtod(state, NULL);
		ret = CPhidgetTemperatureSensor_setTemperatureChangeTrigger(temperature, index, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Trigger/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "ThermocoupleType", sizeof("ThermocoupleType")))
	{
		double value = strtol(state, NULL, 10);
		ret = CPhidgetTemperatureSensor_setThermocoupleType(temperature, index, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/ThermocoupleType/%d", phid->deviceType, phid->serialNumber, index);
		if((ret = add_key(pdss, key, state)))
			return ret;
		
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/TemperatureMin/%d", phid->deviceType, phid->serialNumber, index);
		snprintf(val, MAX_VAL_SIZE, "%lE", temperature->temperatureMin[index]);
		if((ret = add_key(pdss, key, val))) 
			return ret;

		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/TemperatureMax/%d", phid->deviceType, phid->serialNumber, index);
		snprintf(val, MAX_VAL_SIZE, "%lE", temperature->temperatureMax[index]);
		if((ret = add_key(pdss, key, val))) 
			return ret;
	}
	else{
		DPRINT("Bad setType for TemperatureSensor: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return ret;
}

int phidget_textlcd_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret = EPHIDGET_OK;
	CPhidgetTextLCDHandle lcd = (CPhidgetTextLCDHandle)phid;
	int value;
	char key[MAX_KEY_SIZE];

	if(!strncmp(setThing, "Backlight", sizeof("Backlight")))
	{
		value = strtol(state, NULL, 10);
		ret = CPhidgetTextLCD_setBacklight(lcd, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Backlight", phid->deviceType, phid->serialNumber);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "Contrast", sizeof("Contrast")))
	{
		value = strtol(state, NULL, 10);
		ret = CPhidgetTextLCD_setContrast(lcd, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Contrast", phid->deviceType, phid->serialNumber);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "CursorOn", sizeof("CursorOn")))
	{
		value = strtol(state, NULL, 10);
		ret = CPhidgetTextLCD_setCursorOn(lcd, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/CursorOn", phid->deviceType, phid->serialNumber);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "CursorBlink", sizeof("CursorBlink")))
	{
		value = strtol(state, NULL, 10);
		ret = CPhidgetTextLCD_setCursorBlink(lcd, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/CursorBlink", phid->deviceType, phid->serialNumber);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "DisplayString", sizeof("DisplayString")))
	{
		ret = CPhidgetTextLCD_setDisplayString(lcd, index, (char *)state);
	}
	else if(!strncmp(setThing, "DisplayCharacter", sizeof("DisplayCharacter")))
	{
		int row, column, columns;

		CPhidgetTextLCD_getColumnCount(lcd, &columns);

		row = index / columns;
		column = (index / (row + 1)) - 1;

		ret = CPhidgetTextLCD_setDisplayCharacter(lcd, row, column, state[0]);
	}
	else if(!strncmp(setThing, "CustomCharacter", sizeof("CustomCharacter")))
	{
		int val1, val2;
		char *endptr;
		val1 = strtol(state, &endptr, 10);
		val2 = strtol(endptr+1, NULL, 10);
		ret = CPhidgetTextLCD_setCustomCharacter(lcd, index, val1, val2);
	}
	else{
		DPRINT("Bad setType for TextLCD: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return ret;
}

int phidget_textled_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret = EPHIDGET_OK;
	CPhidgetTextLEDHandle textled = (CPhidgetTextLEDHandle)phid;
	int value;
	char key[MAX_KEY_SIZE];

	if(!strncmp(setThing, "Brightness", sizeof("Brightness")))
	{
		value = strtol(state, NULL, 10);
		ret = CPhidgetTextLED_setBrightness(textled, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Brightness", phid->deviceType, phid->serialNumber);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else if(!strncmp(setThing, "DisplayString", sizeof("DisplayString")))
	{
		ret = CPhidgetTextLED_setDisplayString(textled, index, (char *)state);
	}
	else{
		DPRINT("Bad setType for TextLED: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return ret;
}

int phidget_weightsensor_set(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss)
{
	int ret = EPHIDGET_OK;
	CPhidgetWeightSensorHandle weight = (CPhidgetWeightSensorHandle)phid;
	double value = strtod(state, NULL);
	char key[MAX_KEY_SIZE];

	if(!strncmp(setThing, "Trigger", sizeof("Trigger")))
	{
		ret = CPhidgetWeightSensor_setWeightChangeTrigger(weight, value);
		snprintf(key, MAX_KEY_SIZE, "/PSK/%s/%d/Trigger", phid->deviceType, phid->serialNumber);
		if((ret = add_key(pdss, key, state)))
			return ret;
	}
	else{
		DPRINT("Bad setType for WeightSensor: %s\n", setThing);
		return EPHIDGET_INVALIDARG;
	}
	return ret;
}

int(*fptrSet[PHIDGET_DEVICE_CLASS_COUNT])(CPhidgetHandle phid, const char *setThing, int index, const char *state, void *pdss) = {
NULL,
NULL,
phidget_accelerometer_set,
phidget_advancedservo_set,
phidget_encoder_set,
NULL,//phidget_gps_set,
NULL,//phidget_gyroscope_set,
phidget_interfacekit_set,
phidget_led_set,
phidget_motorcontrol_set,
phidget_phsensor_set,
phidget_rfid_set,
phidget_servo_set,
phidget_stepper_set,
phidget_temperaturesensor_set,
phidget_textlcd_set,
phidget_textled_set,
phidget_weightsensor_set};

// format is: "/PCK/(devicetype)/(serial number)/(thing to set)[/(index)[/(extra info)]]"
void phidget_set(const char *k, const char *escapedv, pdict_reason_t r, const char *pde_oldval, void *pdss)
{
	regmatch_t pmatch[6];
	char *deviceType = NULL;
	char *serialNumber = NULL;
	char *setThing = NULL; //ie output
	char *index = NULL;
	char *setThingSpecific = NULL;
	char *v;
	
	int deviceID;

	long serial;
	CNetworkPhidgetInfoHandle device;

	int res, ind=-1, ret = EPHIDGET_OK;

	unescape(escapedv, &v, NULL);

	if(!strncmp(v, "\001", 1) && strlen(v) == 1)
	{
		memset(v,0,1);
	}

	if(r!=PDR_ENTRY_REMOVING)
	{
		if ((res = regexec(&phidgetsetex, k, 6, pmatch, 0)) != 0) {
			DPRINT("Error in phidget_set - pattern not met\n");
			return;
		}
		getmatchsub(k, &deviceType, pmatch, 1);
		getmatchsub(k, &serialNumber, pmatch, 2);
		getmatchsub(k, &setThing, pmatch, 3);
		getmatchsub(k, &index, pmatch, 4);
		getmatchsub(k, &setThingSpecific, pmatch, 5);// - not needed?

		//look for a device with that serial number / device type
		pd_unlock((void *)&pd_mutex);
		pthread_mutex_lock(&PhidgetsAndClientsMutex);
		if(deviceType && serialNumber)
		{
			deviceID=phidget_type_to_id(deviceType);
			serial = strtol(serialNumber, NULL, 10);
			if((findNetworkPhidgetInfoHandleInList(OpenPhidgets, serial, deviceID, &device)))
			{
				DPRINT("Couldn't find device\n");
				//TODO: send an error
				free(deviceType); deviceType = NULL;
				free(serialNumber); serialNumber = NULL;
				free(setThing); setThing = NULL;
				free(index); index = NULL;
				free(setThingSpecific); setThingSpecific = NULL;
				free(v); v = NULL;
				pthread_mutex_unlock(&PhidgetsAndClientsMutex);
				pd_lock((void *)&pd_mutex);
				return;
			}
		}

		if(!strncmp(setThing, "Label", sizeof("Label")))
		{
			ret = CPhidget_setDeviceLabel(device->phidget, v);
		}
		else if(fptrSet[device->phidget->deviceID] && setThing)
		{
			if(index)
				ind = strtol(index, NULL, 10);
			ret = fptrSet[device->phidget->deviceID](device->phidget, setThing, ind, v, pdss);
		}
		else
			ret = EPHIDGET_INVALIDARG;//TODO: error
		
		pthread_mutex_unlock(&PhidgetsAndClientsMutex);
		pd_lock((void *)&pd_mutex);
		free(deviceType); deviceType = NULL;
		free(serialNumber); serialNumber = NULL;
		free(setThing); setThing = NULL;
		free(index); index = NULL;
		free(setThingSpecific); setThingSpecific = NULL;
	}
	free(v); v = NULL;
	//TODO: send some sort of error response
	if(ret)
	{
		DPRINT("Error in set: %d", ret);
	}
}

// format is: "/PCK/Client/(ip address)/(client port)/(device type)/(serial number)"
void phidget_openclose(const char *k, const char *escapedv, pdict_reason_t r, const char *pde_oldval, void *pdss)
{
	regmatch_t pmatch[5];
	char *ipaddr = NULL;
	char *port = NULL;
	char *deviceType = NULL; //ie output
	char *serialNumber = NULL;
	char *v;

	long serial;
	int deviceID;

	int res;

	unescape(escapedv, &v, NULL);

	if(!strncmp(v, "\001", 1) && strlen(v) == 1)
	{
		memset(v,0,1);
	}

	if ((res = regexec(&phidgetopencloseex, k, 5, pmatch, 0)) != 0) {
		DPRINT("Error in phidget_openclose - pattern not met (%s:%s)\n",k,v);
		return;
	}
	getmatchsub(k, &ipaddr, pmatch, 1);
	getmatchsub(k, &port, pmatch, 2);
	getmatchsub(k, &deviceType, pmatch, 3);
	getmatchsub(k, &serialNumber, pmatch, 4);

	switch(r)
	{
		case PDR_ENTRY_REMOVING:
			if(!strcmp(v, "Open")) //An open device was detached
			{
				//close devices referenced by this client
				CNetworkPhidgetListHandle trav = 0;
				CPhidgetHandle devices_to_close[128];
				int num_to_close = 0, i;
				CClientInfoHandle newClient;
				
				pd_unlock((void *)&pd_mutex);
				pthread_mutex_lock(&PhidgetsAndClientsMutex);
				if(!findClientInfoHandleInList(ConnectedClients, ipaddr, port, &newClient))
				{
					for (trav=newClient->phidgets; trav; trav = trav->next) {
						devices_to_close[num_to_close++] = trav->phidgetInfo->phidget;
					}
				}

				for(i=0;i<num_to_close;i++)
				{
					close_phidget(pdss, devices_to_close[i]->deviceID, devices_to_close[i]->serialNumber, ipaddr, port);
				}
				pthread_mutex_unlock(&PhidgetsAndClientsMutex);
				pd_lock((void *)&pd_mutex);
			}
			break;
		default:
			if(deviceType && v)
			{
				if(!strcmp(v, "Open"))
				{
					if(serialNumber)
						serial = strtol(serialNumber, NULL, 10);
					else
						serial = -1;
					deviceID = phidget_type_to_id(deviceType);
					
					pd_unlock((void *)&pd_mutex);
					pthread_mutex_lock(&PhidgetsAndClientsMutex);
					if(open_phidget(pdss, deviceID, serial, ipaddr, port))
					{
						DPRINT("Couldn't open device\n");
						//TODO: send an error
						pthread_mutex_unlock(&PhidgetsAndClientsMutex);
						pd_lock((void *)&pd_mutex);
						free(ipaddr); ipaddr = NULL;
						free(port); port = NULL;
						free(deviceType); deviceType = NULL;
						free(serialNumber); serialNumber = NULL;
						return;
					}
					pthread_mutex_unlock(&PhidgetsAndClientsMutex);
					pd_lock((void *)&pd_mutex);
				}
				if(!strcmp(v, "Close") && serialNumber)
				{
					serial = strtol(serialNumber, NULL, 10);
					deviceID = phidget_type_to_id(deviceType);
					
					pd_unlock((void *)&pd_mutex);
					pthread_mutex_lock(&PhidgetsAndClientsMutex);
					if(close_phidget(pdss, deviceID, serial, ipaddr, port))
					{
						DPRINT("Couldn't close device\n");
						//TODO: send an error
						pthread_mutex_unlock(&PhidgetsAndClientsMutex);
						pd_lock((void *)&pd_mutex);
						free(ipaddr); ipaddr = NULL;
						free(port); port = NULL;
						free(deviceType); deviceType = NULL;
						free(serialNumber); serialNumber = NULL;
						free(v); v = NULL;
						return;
					}
					pthread_mutex_unlock(&PhidgetsAndClientsMutex);
					pd_lock((void *)&pd_mutex);
				}
			}
			if(!strcmp(ipaddr, "0.0.0.0"))
			{
				DPRINT("Phidget %s: (AS3 Client, ID: %s), (Device: %s, %s%s)", v, port, deviceType, serialNumber?"Serial Number: ":"Any Serial", serialNumber?serialNumber:"");
			}
			else
			{
				DPRINT("Phidget %s: (Client: %s:%s), (Device: %s, %s%s)", v, ipaddr, port, deviceType, serialNumber?"Serial Number: ":"Any Serial", serialNumber?serialNumber:"");
			}
	}

	free(ipaddr); ipaddr = NULL;
	free(port); port = NULL;
	free(deviceType); deviceType = NULL;
	free(serialNumber); serialNumber = NULL;
	free(v); v = NULL;
}

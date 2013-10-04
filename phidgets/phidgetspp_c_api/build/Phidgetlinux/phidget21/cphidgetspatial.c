#include "stdafx.h"
#include "cphidgetspatial.h"
#include "cusb.h"
#include "math.h"
#include "csocket.h"
#include "cthread.h"

// === Internal Functions === //

//clearVars - sets all device variables to unknown state
CPHIDGETCLEARVARS(Spatial)
	int i = 0;

	//TODO: finish initializations
	phid->accelerationMax = PUNI_DBL;
	phid->accelerationMin = PUNI_DBL;

	for (i = 0; i<SPATIAL_MAX_ACCELAXES; i++)
	{
		phid->accelAxis[i] = PUNI_DBL;
	}
	return EPHIDGET_OK;
}

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
CPHIDGETINIT(Spatial)
	int i = 0;

	TESTPTR(phid);

	//Setup max/min values
	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_SPACIAL_ACCEL_3AXIS:
			if (phid->phid.deviceVersion < 200)
			{
				phid->accelerationMax = 12.1;
				phid->accelerationMin = -12.1;
				phid->interruptRate = 8;
				phid->dataRateMin = SPATIAL_MIN_DATA_RATE;
				phid->dataRate = phid->interruptRate;
				phid->dataRateMax = 1; //actual data rate
				phid->angularRateMax = 0;
				phid->angularRateMin = 0;
				phid->magneticFieldMax = 0;
				phid->magneticFieldMin = 0;
				phid->calDataValid = PFALSE;
			}
			else
				return EPHIDGET_UNEXPECTED;
			break;
		case PHIDID_SPACIAL_ACCEL_GYRO_COMPASS:
			if (phid->phid.deviceVersion < 200)
			{
				phid->accelerationMax = 12.1;
				phid->accelerationMin = -12.1;
				phid->interruptRate = 8;
				phid->dataRateMin = SPATIAL_MIN_DATA_RATE;
				phid->dataRate = phid->interruptRate;
				phid->dataRateMax = 4; //actual data rate
				phid->angularRateMax = 400.1;
				phid->angularRateMin = -400.1;
				//TODO: real range is probably less
				phid->magneticFieldMax = 6;
				phid->magneticFieldMin = -6;
				phid->calDataValid = PFALSE;
			}
			else
				return EPHIDGET_UNEXPECTED;
			break;
		default:
			return EPHIDGET_UNEXPECTED;
	}

	//initialize triggers, set data arrays to unknown
	for (i = 0; i<phid->phid.attr.spatial.numAccelAxes; i++)
	{
		phid->accelAxis[i] = PUNK_DBL;
		phid->accelGain1[i] = PUNK_DBL;
		phid->accelGain2[i] = PUNK_DBL;
		phid->accelOffset[i] = PUNK_INT;
	}
	for (i = 0; i<phid->phid.attr.spatial.numGyroAxes; i++)
	{
		phid->gyroAxis[i] = PUNK_DBL;
		phid->gryoCorrection[i] = 0;
		phid->gyroGain1[i] = PUNK_DBL;
		phid->gyroGain2[i] = PUNK_DBL;
		phid->gyroOffset[i] = PUNK_INT;
	}
	for (i = 0; i<phid->phid.attr.spatial.numCompassAxes; i++)
	{
		phid->compassAxis[i] = PUNK_DBL;
	}
	phid->bufferReadPtr = 0;
	phid->bufferWritePtr = 0;
	phid->timestamp.seconds = 0;
	phid->timestamp.microseconds = 0;
	phid->lastEventTime.seconds = 0;
	phid->lastEventTime.microseconds = 0;
	phid->latestDataTime.seconds = 0;
	phid->latestDataTime.microseconds = 0;

	phid->lastTimeCounterValid = PFALSE;
	phid->doZeroGyro = PFALSE;

	//get calibration values
	switch(phid->phid.deviceIDSpec) {
		case PHIDID_SPACIAL_ACCEL_3AXIS:
		case PHIDID_SPACIAL_ACCEL_GYRO_COMPASS:
			{
				unsigned char buffer[8] = { 0 };
				int result;
				int readCount = 125; // up to 1 second of data - should be PLENTY
				//ask for calibration values
				buffer[0] = SPACIAL_READCALIB;
				if ((result = CUSBSendPacket((CPhidgetHandle)phid, buffer)) != EPHIDGET_OK)
					return result;
				while(phid->calDataValid == PFALSE && readCount--)
				{
					//note that Windows queues up to 32 packets, so we need to read at least this many to get the calibration packet
					CPhidget_read((CPhidgetHandle)phid);
				}
				if(!phid->calDataValid)
					return EPHIDGET_UNEXPECTED;
			}
			break;
		default:
			break;
	}

	//issue one read
	//this should fill in the data because the dataRate is the interrupt rate
	CPhidget_read((CPhidgetHandle)phid);

	return EPHIDGET_OK;
}

//dataInput - parses device packets
CPHIDGETDATA(Spatial)
	int i = 0, j = 0, count = 0, dataRate = phid->dataRate;
	unsigned char doneGyroZero = PFALSE;
	double accelAvg[SPATIAL_MAX_ACCELAXES], angularRateAvg[SPATIAL_MAX_ACCELAXES], magneticFieldAvg[SPATIAL_MAX_ACCELAXES];
	CPhidgetSpatial_SpatialEventDataHandle *eventData;
	
	ZEROMEM(accelAvg, sizeof(accelAvg));
	ZEROMEM(angularRateAvg, sizeof(angularRateAvg));
	ZEROMEM(magneticFieldAvg, sizeof(magneticFieldAvg));

	if (length<0) return EPHIDGET_INVALIDARG;
	TESTPTR(phid);
	TESTPTR(buffer);

	//Parse device packets - store data locally
	switch(phidG->deviceIDSpec)
	{
		case PHIDID_SPACIAL_ACCEL_3AXIS:
			if (phid->phid.deviceVersion < 200)
			{
				int data;
				double dataD;
				int time;
				
				//top 2 bits in buffer[0] are packet type
				switch(buffer[0] & 0xc0)
				{
					case SPACIAL_PACKET_DATA:
						if(phid->calDataValid)
						{
							count = buffer[0] / 3;
							if(count == 0)
								goto done;

							//this timestamp is for the latest data
							time = ((unsigned short)buffer[1]<<8) + (unsigned short)buffer[2];
							if(phid->lastTimeCounterValid)
							{
								//0-255 ms
								int timechange = (unsigned short)((unsigned short)time - (unsigned short)phid->lastTimeCounter);
								timechange *= 1000; //us

								phid->timestamp.seconds = phid->timestamp.seconds + (phid->timestamp.microseconds + timechange) / 1000000;
								phid->timestamp.microseconds = (phid->timestamp.microseconds + timechange) % 1000000;
							}
							else
							{
								phid->lastTimeCounterValid = PTRUE;
							}
							phid->lastTimeCounter = time;

							//add data to data buffer
							for(i=0;i<count;i++)
							{
								//X
								data = ((unsigned short)buffer[3 + i * 6]<<8) + (unsigned short)buffer[4 + i * 6];
								dataD = ((double)data - 0x0fff + phid->accelOffset[0]) / 207.5;
								if(dataD > 0)
									phid->dataBuffer[phid->bufferWritePtr].acceleration[0] = dataD * phid->accelGain1[0];
								else
									phid->dataBuffer[phid->bufferWritePtr].acceleration[0] = dataD * phid->accelGain2[0];
								//Y
								data = ((unsigned short)buffer[5 + i * 6]<<8) + (unsigned short)buffer[6 + i * 6];
								dataD = ((double)data - 0x0fff + phid->accelOffset[1]) / 207.5;
								if(dataD > 0)
									phid->dataBuffer[phid->bufferWritePtr].acceleration[1] = dataD * phid->accelGain1[1];
								else
									phid->dataBuffer[phid->bufferWritePtr].acceleration[1] = dataD * phid->accelGain2[1];
								//Z
								data = ((unsigned short)buffer[7 + i * 6]<<8) + (unsigned short)buffer[8 + i * 6];
								dataD = ((double)data - 0x0fff + phid->accelOffset[2]) / 207.5;
								if(dataD > 0)
									phid->dataBuffer[phid->bufferWritePtr].acceleration[2] = dataD * phid->accelGain1[2];
								else
									phid->dataBuffer[phid->bufferWritePtr].acceleration[2] = dataD * phid->accelGain2[2];

								phid->latestDataTime.seconds = phid->timestamp.seconds + (phid->timestamp.microseconds + (i + 1) * phid->dataRateMax * 1000) / 1000000;
								phid->latestDataTime.microseconds = (phid->timestamp.microseconds + (i + 1) * phid->dataRateMax * 1000) % 1000000;

								phid->dataBuffer[phid->bufferWritePtr].timestamp = phid->latestDataTime;

								phid->bufferWritePtr++;
								if(phid->bufferWritePtr >= SPATIAL_DATA_BUFFER_SIZE)
									phid->bufferWritePtr = 0;
							}
						}
						break;
					case SPACIAL_PACKET_CALIB:
						for (i = 0; i<phid->phid.attr.spatial.numAccelAxes; i++)
						{
							phid->accelOffset[i] = (signed short)((unsigned short)buffer[i*8 + 1]<<8) + (unsigned short)buffer[i*8 + 2];
							phid->accelGain1[i] = ((unsigned short)buffer[i*8 + 3]<<16) + ((unsigned short)buffer[i*8 + 4]<<8) + (unsigned short)buffer[i*8 + 5];
							phid->accelGain1[i] /= 65536.0;
							phid->accelGain2[i] = ((unsigned short)buffer[i*8 + 6]<<16) + ((unsigned short)buffer[i*8 + 7]<<8) + (unsigned short)buffer[i*8 + 8];
							phid->accelGain2[i] /= 65536.0;
						}
						phid->calDataValid = PTRUE;
						break;
				}
			}
			else
				return EPHIDGET_UNEXPECTED;
			break;
		case PHIDID_SPACIAL_ACCEL_GYRO_COMPASS:
			if (phidG->deviceVersion < 200)
			{
				//top 2 bits in buffer[0] are packet type
				switch(buffer[0])
				{
					case SPACIAL_PACKET_DATA:
						if(phid->calDataValid)
						{
							int data;
							double dataD;
							int time;
							
							count = (buffer[1] & 0x1f) / 9;
							if(count == 0)
								goto done;

							//this timestamp is for the latest data
							time = ((unsigned short)buffer[2]<<8) + (unsigned short)buffer[3];
							if(phid->lastTimeCounterValid)
							{
								//0-255 ms
								int timechange = (unsigned short)((unsigned short)time - (unsigned short)phid->lastTimeCounter);
								timechange *= 1000; //us

								phid->timestamp.seconds = phid->timestamp.seconds + (phid->timestamp.microseconds + timechange) / 1000000;
								phid->timestamp.microseconds = (phid->timestamp.microseconds + timechange) % 1000000;
							}
							else
							{
								phid->lastTimeCounterValid = PTRUE;
							}
							phid->lastTimeCounter = time;

							//add data to data buffer
							for(i=0;i<count;i++)
							{
								//X
								data = ((unsigned short)buffer[4 + i * 18]<<8) + (unsigned short)buffer[5 + i * 18];
								dataD = ((double)data-0x7fff + phid->accelOffset[0]) / 1660.0;
								if(dataD > 0)
									phid->dataBuffer[phid->bufferWritePtr].acceleration[0] = dataD * phid->accelGain1[0];
								else
									phid->dataBuffer[phid->bufferWritePtr].acceleration[0] = dataD * phid->accelGain2[0];
								//Y
								data = ((unsigned short)buffer[6 + i * 18]<<8) + (unsigned short)buffer[7 + i * 18];
								dataD = ((double)data-0x7fff + phid->accelOffset[1]) / 1660.0;
								if(dataD > 0)
									phid->dataBuffer[phid->bufferWritePtr].acceleration[1] = dataD * phid->accelGain1[1];
								else
									phid->dataBuffer[phid->bufferWritePtr].acceleration[1] = dataD * phid->accelGain2[1];
								//Z
								data = ((unsigned short)buffer[8 + i * 18]<<8) + (unsigned short)buffer[9 + i * 18];
								dataD = ((double)data-0x7fff + phid->accelOffset[2]) / 1660.0;
								if(dataD > 0)
									phid->dataBuffer[phid->bufferWritePtr].acceleration[2] = dataD * phid->accelGain1[2];
								else
									phid->dataBuffer[phid->bufferWritePtr].acceleration[2] = dataD * phid->accelGain2[2];

								//ADC 62.5uV/bit, gyro zero rate is 1.23V, 2.5mV/deg/s
								// 1 / 0.0000625 = 16000
								// 1.23 * 16000 = 19680
								// 0.0025 * 16000 = 40bits/deg/s
								data = ((unsigned short)buffer[10 + i * 18]<<8) + (unsigned short)buffer[11 + i * 18];
								dataD = ((double)data-19680 + phid->gyroOffset[0]) / 40.0;
								if(dataD > 0)
									phid->dataBuffer[phid->bufferWritePtr].angularRate[0] = dataD * phid->gyroGain1[0];
								else
									phid->dataBuffer[phid->bufferWritePtr].angularRate[0] = dataD * phid->gyroGain2[0];

								data = ((unsigned short)buffer[12 + i * 18]<<8) + (unsigned short)buffer[13 + i * 18];
								dataD = ((double)data-19680 + phid->gyroOffset[1]) / 40.0;
								if(dataD > 0)
									phid->dataBuffer[phid->bufferWritePtr].angularRate[1] = dataD * phid->gyroGain1[1];
								else
									phid->dataBuffer[phid->bufferWritePtr].angularRate[1] = dataD * phid->gyroGain2[1];

								data = ((unsigned short)buffer[14 + i * 18]<<8) + (unsigned short)buffer[15 + i * 18];
								dataD = ((double)data-19680 + phid->gyroOffset[2]) / 40.0;
								if(dataD > 0)
									phid->dataBuffer[phid->bufferWritePtr].angularRate[2] = dataD * phid->gyroGain1[2];
								else
									phid->dataBuffer[phid->bufferWritePtr].angularRate[2] = dataD * phid->gyroGain2[2];

								//checks if compass data is valid
								if(buffer[1] & (0x80 >> i))
								{
									data = (short)(((unsigned short)buffer[16 + i * 18]<<8) + (unsigned short)buffer[17 + i * 18]);
									phid->dataBuffer[phid->bufferWritePtr].magneticField[0] = data / 6500.0;
									data = (short)(((unsigned short)buffer[18 + i * 18]<<8) + (unsigned short)buffer[19 + i * 18]);
									phid->dataBuffer[phid->bufferWritePtr].magneticField[1] = data / 6500.0;
									data = (short)(((unsigned short)buffer[20 + i * 18]<<8) + (unsigned short)buffer[21 + i * 18]);
									phid->dataBuffer[phid->bufferWritePtr].magneticField[2] = data / 6500.0;
								}
								else
								{
									phid->dataBuffer[phid->bufferWritePtr].magneticField[0] = PUNK_DBL;
									phid->dataBuffer[phid->bufferWritePtr].magneticField[1] = PUNK_DBL;
									phid->dataBuffer[phid->bufferWritePtr].magneticField[2] = PUNK_DBL;
								}

								phid->latestDataTime.seconds = phid->timestamp.seconds + (phid->timestamp.microseconds + (i + 1) * phid->dataRateMax * 1000) / 1000000;
								phid->latestDataTime.microseconds = (phid->timestamp.microseconds + (i + 1) * phid->dataRateMax * 1000) % 1000000;

								phid->dataBuffer[phid->bufferWritePtr].timestamp = phid->latestDataTime;

								phid->bufferWritePtr++;
								if(phid->bufferWritePtr >= SPATIAL_DATA_BUFFER_SIZE)
									phid->bufferWritePtr = 0;
							}
						}
						break;
					case SPACIAL_PACKET_CALIB:
						for (i = 0; i<phid->phid.attr.spatial.numAccelAxes; i++)
						{
							phid->accelOffset[i] = (signed short)((unsigned short)buffer[i*8 + 1]<<8) + (unsigned short)buffer[i*8 + 2];
							phid->accelGain1[i] = ((unsigned short)buffer[i*8 + 3]<<16) + ((unsigned short)buffer[i*8 + 4]<<8) + (unsigned short)buffer[i*8 + 5];
							phid->accelGain1[i] /= 65536.0;
							phid->accelGain2[i] = ((unsigned short)buffer[i*8 + 6]<<16) + ((unsigned short)buffer[i*8 + 7]<<8) + (unsigned short)buffer[i*8 + 8];
							phid->accelGain2[i] /= 65536.0;
						}
						for (j=0; j<phid->phid.attr.spatial.numGyroAxes; i++,j++)
						{
							phid->gyroOffset[j] = (signed short)((unsigned short)buffer[i*8 + 1]<<8) + (unsigned short)buffer[i*8 + 2];
							phid->gyroGain1[j] = ((unsigned short)buffer[i*8 + 3]<<16) + ((unsigned short)buffer[i*8 + 4]<<8) + (unsigned short)buffer[i*8 + 5];
							phid->gyroGain1[j] /= 65536.0;
							phid->gyroGain2[j] = ((unsigned short)buffer[i*8 + 6]<<16) + ((unsigned short)buffer[i*8 + 7]<<8) + (unsigned short)buffer[i*8 + 8];
							phid->gyroGain2[j] /= 65536.0;
						}
						phid->calDataValid = PTRUE;
						break;
				}
			}
			else
				return EPHIDGET_UNEXPECTED;
			break;
		default:
			return EPHIDGET_UNEXPECTED;
	}

	if(phid->doZeroGyro)
	{
		//done
		if(timestampdiff(phid->latestDataTime, phid->dataBuffer[phid->gyroZeroReadPtr].timestamp) >= SPATIAL_ZERO_GYRO_TIME)
		{
			double gryoCorrectionTemp[SPATIAL_MAX_GYROAXES] = {0,0,0};
			int gryoCorrectionCount = 0;

			while(phid->gyroZeroReadPtr != phid->bufferWritePtr)
			{
				for (i = 0; i<phid->phid.attr.spatial.numGyroAxes; i++)
				{
					gryoCorrectionTemp[i] += phid->dataBuffer[phid->gyroZeroReadPtr].angularRate[i];
				}

				phid->gyroZeroReadPtr++;
				if(phid->gyroZeroReadPtr >= SPATIAL_DATA_BUFFER_SIZE)
					phid->gyroZeroReadPtr = 0;

				gryoCorrectionCount++;
			}
			
			for (i = 0; i<phid->phid.attr.spatial.numGyroAxes; i++)
			{
				phid->gryoCorrection[i] = gryoCorrectionTemp[i] / (double)gryoCorrectionCount;
			}

			doneGyroZero = PTRUE;
		}
	}

	//see if it's time for an event
	if(timestampdiff(phid->latestDataTime, phid->lastEventTime) >= dataRate)
	{
		CPhidget_Timestamp tempTime;
		//int lastPtr;
		int accelCounter[SPATIAL_MAX_ACCELAXES], angularRateCounter[SPATIAL_MAX_ACCELAXES], magneticFieldCounter[SPATIAL_MAX_ACCELAXES];

		int dataPerEvent = 0;

		int multipleDataPerEvent = PFALSE;

		if(dataRate < phid->interruptRate)
			multipleDataPerEvent = PTRUE;

		//max of 16 data per event
		eventData = malloc(16 * sizeof(CPhidgetSpatial_SpatialEventDataHandle));
		
		for(j=0;;j++)
		{
			//makes sure we read all data
			if(phid->bufferReadPtr == phid->bufferWritePtr || j>=16)
			{
				dataPerEvent = j;
				break;
			}

			eventData[j] = malloc(sizeof(CPhidgetSpatial_SpatialEventData));
			ZEROMEM(accelCounter, sizeof(accelCounter));
			ZEROMEM(angularRateCounter, sizeof(angularRateCounter));
			ZEROMEM(magneticFieldCounter, sizeof(magneticFieldCounter));

			tempTime = phid->dataBuffer[phid->bufferReadPtr].timestamp;

			//average data for each stage
			while(phid->bufferReadPtr != phid->bufferWritePtr && 
				(!multipleDataPerEvent || timestampdiff(phid->dataBuffer[phid->bufferReadPtr].timestamp, tempTime) < dataRate))
			{
				for (i = 0; i<phid->phid.attr.spatial.numAccelAxes; i++)
				{
					if(phid->dataBuffer[phid->bufferReadPtr].acceleration[i] != PUNK_DBL)
					{
						if(phid->dataBuffer[phid->bufferReadPtr].acceleration[i] > phid->accelerationMax)
							phid->dataBuffer[phid->bufferReadPtr].acceleration[i] = phid->accelerationMax;
						if(phid->dataBuffer[phid->bufferReadPtr].acceleration[i] < phid->accelerationMin) 
							phid->dataBuffer[phid->bufferReadPtr].acceleration[i] = phid->accelerationMin;
						accelAvg[i] += phid->dataBuffer[phid->bufferReadPtr].acceleration[i];
						accelCounter[i]++;
					}
				}
				for (i = 0; i<phid->phid.attr.spatial.numGyroAxes; i++)
				{
					if(phid->dataBuffer[phid->bufferReadPtr].angularRate[i] != PUNK_DBL)
					{
						double rate = phid->dataBuffer[phid->bufferReadPtr].angularRate[i] - phid->gryoCorrection[i];

						if(rate > phid->angularRateMax) 
							angularRateAvg[i] += phid->angularRateMax;
						else if(rate < phid->angularRateMin) 
							angularRateAvg[i] += phid->angularRateMin;
						else
							angularRateAvg[i] += rate;
						angularRateCounter[i]++;
					}
				}
				for (i = 0; i<phid->phid.attr.spatial.numCompassAxes; i++)
				{
					if(phid->dataBuffer[phid->bufferReadPtr].magneticField[i] != PUNK_DBL)
					{
						if(phid->dataBuffer[phid->bufferReadPtr].magneticField[i] > phid->magneticFieldMax) 
							phid->dataBuffer[phid->bufferReadPtr].magneticField[i] = phid->magneticFieldMax;
						if(phid->dataBuffer[phid->bufferReadPtr].magneticField[i] < phid->magneticFieldMin) 
							phid->dataBuffer[phid->bufferReadPtr].magneticField[i] = phid->magneticFieldMin;
						magneticFieldAvg[i] += phid->dataBuffer[phid->bufferReadPtr].magneticField[i];
						magneticFieldCounter[i]++;
					}
				}

				//lastPtr = phid->bufferReadPtr;

				phid->bufferReadPtr++;
				if(phid->bufferReadPtr >= SPATIAL_DATA_BUFFER_SIZE)
					phid->bufferReadPtr = 0;
			}

			for (i = 0; i<phid->phid.attr.spatial.numAccelAxes; i++)
			{
				if(accelCounter[i] > 0)
					eventData[j]->acceleration[i] = round_double(accelAvg[i] / (double)accelCounter[i], 5);
				else
					eventData[j]->acceleration[i] = PUNK_DBL;
				accelAvg[i] = 0;
			}
			for (i = 0; i<phid->phid.attr.spatial.numGyroAxes; i++)
			{
				if(angularRateCounter[i] > 0)
				{
					if(phid->doZeroGyro && !doneGyroZero)
						eventData[j]->angularRate[i] = 0;
					else
						eventData[j]->angularRate[i] = round_double(angularRateAvg[i] / (double)angularRateCounter[i], 5);
				}
				else
					eventData[j]->angularRate[i] = PUNK_DBL;
				angularRateAvg[i] = 0;
			}
			for (i = 0; i<phid->phid.attr.spatial.numCompassAxes; i++)
			{
				if(magneticFieldCounter[i] > 0)
					eventData[j]->magneticField[i] = round_double(magneticFieldAvg[i] / (double)magneticFieldCounter[i], 5);
				else
					eventData[j]->magneticField[i] = PUNK_DBL;
				magneticFieldAvg[i] = 0;
			}
			eventData[j]->timestamp = tempTime;
		}

		ZEROMEM(accelCounter, sizeof(accelCounter));
		ZEROMEM(angularRateCounter, sizeof(angularRateCounter));
		ZEROMEM(magneticFieldCounter, sizeof(magneticFieldCounter));

		//store to local structure
		for( j = 0; j < dataPerEvent; j++)
		{
			for (i = 0; i<phid->phid.attr.spatial.numAccelAxes; i++)
			{
				if(eventData[j]->acceleration[i] != PUNK_DBL)
				{
					accelAvg[i] += eventData[j]->acceleration[i];
					accelCounter[i]++;
				}
			}
			for (i = 0; i<phid->phid.attr.spatial.numGyroAxes; i++)
			{
				if(eventData[j]->angularRate[i] != PUNK_DBL)
				{
					angularRateAvg[i] += eventData[j]->angularRate[i];
					angularRateCounter[i]++;
				}
			}
			for (i = 0; i<phid->phid.attr.spatial.numCompassAxes; i++)
			{
				if(eventData[j]->magneticField[i] != PUNK_DBL)
				{
					magneticFieldAvg[i] += eventData[j]->magneticField[i];
					magneticFieldCounter[i]++;
				}
			}
		}

		//Set local get data to averages
		for (i = 0; i<phid->phid.attr.spatial.numAccelAxes; i++)
		{
			if(accelCounter[i] > 0)
				phid->accelAxis[i] = round_double(accelAvg[i] / (double)accelCounter[i], 5);
			else
				phid->accelAxis[i] = PUNK_DBL;
		}
		for (i = 0; i<phid->phid.attr.spatial.numGyroAxes; i++)
		{
			if(angularRateCounter[i] > 0)
			{
				if(phid->doZeroGyro && !doneGyroZero)
					phid->gyroAxis[i] = 0;
				else
					phid->gyroAxis[i] = round_double(angularRateAvg[i] / (double)angularRateCounter[i], 5);
			}
			else
				phid->gyroAxis[i] = PUNK_DBL;
		}
		for (i = 0; i<phid->phid.attr.spatial.numCompassAxes; i++)
		{
			if(magneticFieldCounter[i] > 0)
				phid->compassAxis[i] = round_double(magneticFieldAvg[i] / (double)magneticFieldCounter[i], 5);
			else
				phid->compassAxis[i] = PUNK_DBL;
		}
		
		//send out any events
		FIRE(SpatialData, eventData, dataPerEvent);

		phid->lastEventTime = phid->latestDataTime;

		for(i=0;i<dataPerEvent;i++)
			free(eventData[i]);
		free(eventData);
	}
done:

	//this will signal the zero function to return;
	if(doneGyroZero)
		phid->doZeroGyro = PFALSE;

	return EPHIDGET_OK;
}

//eventsAfterOpen - sends out an event for all valid data, used during attach initialization
CPHIDGETINITEVENTS(Spatial)
	TESTPTR(phid);
	//don't need to worry, because the interrupts come at a set rate
	return EPHIDGET_OK;
}

//getPacket - not used for spatial
CGETPACKET(Spatial)
	return EPHIDGET_UNEXPECTED;
}

// === Exported Functions === //

//create and initialize a device structure
CCREATE(Spatial, PHIDCLASS_SPATIAL)

//event setup functions
CFHANDLE(Spatial, SpatialData, CPhidgetSpatial_SpatialEventDataHandle *, int)

CGET(Spatial,AccelerationAxisCount,int)
	TESTPTRS(phid,pVal) 
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED

	MASGN(phid.attr.spatial.numAccelAxes)
}
CGET(Spatial,GyroAxisCount,int)
	TESTPTRS(phid,pVal) 
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED

	MASGN(phid.attr.spatial.numGyroAxes)
}
CGET(Spatial,CompassAxisCount,int)
	TESTPTRS(phid,pVal) 
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED

	MASGN(phid.attr.spatial.numCompassAxes)
}

CGETINDEX(Spatial,Acceleration,double)
	TESTPTRS(phid,pVal) 	
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED
	TESTINDEX(phid.attr.spatial.numAccelAxes)
	TESTMASGN(accelAxis[Index], PUNK_DBL)

	MASGN(accelAxis[Index])
}

CGETINDEX(Spatial,AccelerationMax,double)
	TESTPTRS(phid,pVal) 	
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED
	TESTINDEX(phid.attr.spatial.numAccelAxes)
	TESTMASGN(accelerationMax, PUNK_DBL)

	MASGN(accelerationMax)
}

CGETINDEX(Spatial,AccelerationMin,double)
	TESTPTRS(phid,pVal) 	
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED
	TESTINDEX(phid.attr.spatial.numAccelAxes)
	TESTMASGN(accelerationMin, PUNK_DBL)

	MASGN(accelerationMin)
}

CGETINDEX(Spatial,AngularRate,double)
	TESTPTRS(phid,pVal) 	
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED

	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_SPACIAL_ACCEL_3_GYRO_1:
		case PHIDID_SPACIAL_ACCEL_GYRO_COMPASS:
			TESTINDEX(phid.attr.spatial.numGyroAxes)
			TESTMASGN(gyroAxis[Index], PUNK_DBL)
			MASGN(gyroAxis[Index])
		case PHIDID_SPACIAL_ACCEL_3AXIS:
		default:
			return EPHIDGET_UNSUPPORTED;
	}
}

CGETINDEX(Spatial,AngularRateMax,double)
	TESTPTRS(phid,pVal) 	
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED

	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_SPACIAL_ACCEL_3_GYRO_1:
		case PHIDID_SPACIAL_ACCEL_GYRO_COMPASS:
			TESTINDEX(phid.attr.spatial.numGyroAxes)
			TESTMASGN(angularRateMax, PUNK_DBL)
			MASGN(angularRateMax)
		case PHIDID_SPACIAL_ACCEL_3AXIS:
		default:
			return EPHIDGET_UNSUPPORTED;
	}
}

CGETINDEX(Spatial,AngularRateMin,double)
	TESTPTRS(phid,pVal) 	
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED

	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_SPACIAL_ACCEL_3_GYRO_1:
		case PHIDID_SPACIAL_ACCEL_GYRO_COMPASS:
			TESTINDEX(phid.attr.spatial.numGyroAxes)
			TESTMASGN(angularRateMin, PUNK_DBL)
			MASGN(angularRateMin)
		case PHIDID_SPACIAL_ACCEL_3AXIS:
		default:
			return EPHIDGET_UNSUPPORTED;
	}
}

CGETINDEX(Spatial,MagneticField,double)
	TESTPTRS(phid,pVal) 	
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED

	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_SPACIAL_ACCEL_GYRO_COMPASS:
			TESTINDEX(phid.attr.spatial.numCompassAxes)
			TESTMASGN(compassAxis[Index], PUNK_DBL)
			MASGN(compassAxis[Index])
		case PHIDID_SPACIAL_ACCEL_3_GYRO_1:
		case PHIDID_SPACIAL_ACCEL_3AXIS:
		default:
			return EPHIDGET_UNSUPPORTED;
	}
}

CGETINDEX(Spatial,MagneticFieldMax,double)
	TESTPTRS(phid,pVal) 	
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED

	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_SPACIAL_ACCEL_GYRO_COMPASS:
			TESTINDEX(phid.attr.spatial.numCompassAxes)
			TESTMASGN(magneticFieldMax, PUNK_DBL)
			MASGN(magneticFieldMax)
		case PHIDID_SPACIAL_ACCEL_3_GYRO_1:
		case PHIDID_SPACIAL_ACCEL_3AXIS:
		default:
			return EPHIDGET_UNSUPPORTED;
	}
}

CGETINDEX(Spatial,MagneticFieldMin,double)
	TESTPTRS(phid,pVal) 	
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED

	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_SPACIAL_ACCEL_GYRO_COMPASS:
			TESTINDEX(phid.attr.spatial.numCompassAxes)
			TESTMASGN(magneticFieldMin, PUNK_DBL)
			MASGN(magneticFieldMin)
		case PHIDID_SPACIAL_ACCEL_3_GYRO_1:
		case PHIDID_SPACIAL_ACCEL_3AXIS:
		default:
			return EPHIDGET_UNSUPPORTED;
	}
}

CSET(Spatial,DataRate,int)
	TESTPTR(phid)
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED
	TESTRANGE(phid->dataRateMax, phid->dataRateMin)

	if(CPhidget_statusFlagIsSet(phid->phid.status, PHIDGET_REMOTE_FLAG))
		;//TODO
	else
		phid->dataRate = newVal;

	return EPHIDGET_OK;
}
CGET(Spatial,DataRate,int)
	TESTPTRS(phid,pVal) 	
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED
	TESTMASGN(dataRate, PUNK_INT)

	MASGN(dataRate)
}

CGET(Spatial,DataRateMax,int)
	TESTPTRS(phid,pVal) 	
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED
	TESTMASGN(dataRateMax, PUNK_INT)

	MASGN(dataRateMax)
}

CGET(Spatial,DataRateMin,int)
	TESTPTRS(phid,pVal) 	
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED
	TESTMASGN(dataRateMin, PUNK_INT)

	MASGN(dataRateMin)
}

PHIDGET21_API int CCONV CPhidgetSpatial_zeroGyro(CPhidgetSpatialHandle phid)
{
	TESTPTR(phid) 
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED
	if(phid->phid.attr.spatial.numGyroAxes==0)
		return EPHIDGET_UNSUPPORTED;

	if(!phid->doZeroGyro)
	{
		phid->gyroZeroReadPtr = phid->bufferReadPtr;
		phid->doZeroGyro = PTRUE;
	}

	return EPHIDGET_OK;
}

//Maybe add these later
/*
CGET(Spatial,GyroHeading,double)
	TESTPTRS(phid,pVal) 	
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED
	TESTMASGN(gyroHeading, PUNK_DBL)

	MASGN(gyroHeading)
}

CGET(Spatial,CompassHeading,double)
	TESTPTRS(phid,pVal) 	
	TESTDEVICETYPE(PHIDCLASS_SPATIAL)
	TESTATTACHED
	TESTMASGN(compassHeading, PUNK_DBL)

	MASGN(compassHeading)
}*/

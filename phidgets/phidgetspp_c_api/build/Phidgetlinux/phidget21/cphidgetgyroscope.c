#include "stdafx.h"
#include "cphidgetgyroscope.h"
#include "math.h"
#include "cusb.h"
#include "csocket.h"
#include "cthread.h"

//Don't compile in any Gyro code
#if 0
// === Internal Functions === //

//clearVars - sets all device variables to unknown state
CPHIDGETCLEARVARS(Gyroscope)
	int i = 0;
	for (i = 0; i<GYRO_MAXAXES; i++)
	{
		phid->axis[i] = PUNI_DBL;
		phid->axisLastTrigger[i] = PUNK_DBL;
		phid->axisChangeTrigger[i] = PUNI_DBL;
		phid->axisCalibration[i] = 0;
	}
	phid->calibrated = PFALSE;
	phid->calibCount = 0;
	return EPHIDGET_OK;
}

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
CPHIDGETINIT(Gyroscope)
	int i = 0;

	TESTPTR(phid);

	//TODO:Setup max/min values

	//initialize triggers, set data arrays to unknown
	for (i = 0; i<phid->phid.attr.gyroscope.numAxis; i++)
	{
		phid->axis[i] = PUNK_DBL;
		phid->axisLastTrigger[i] = PUNK_DBL;
		phid->axisChangeTrigger[i] = 0;
	}

	//issue one read
	CPhidget_read((CPhidgetHandle)phid);

	return EPHIDGET_OK;
}

//dataInput - parses device packets
CPHIDGETDATA(Gyroscope)
	int x,y,z,i;
	double Vx, Vy, Vz;
	double axis[GYRO_MAXAXES];

	if (length<0) return EPHIDGET_INVALIDARG;
	TESTPTR(phid);
	TESTPTR(buffer);

	ZEROMEM(axis, sizeof(axis));
	
	//Parse device packets - store data locally
	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_GYROSCOPE_w_ACCELEROMETER:
			if (phid->phid.deviceVersion < 200)
			{
				/* -- old
				i = ((unsigned short)buffer[0]+((unsigned short)buffer[1]<<8));
				axis[0] = round_double((((double)(i-32768)) / 196.605), 4);
				i = ((unsigned short)buffer[2]+((unsigned short)buffer[3]<<8));
				axis[1] = round_double((((double)(i-32768)) / 196.605), 4);
				i = ((unsigned short)buffer[4]+((unsigned short)buffer[5]<<8));
				axis[2] = round_double((((double)(i-32768)) / 196.605), 4);
				 */

#if 1 // This is the 2-axis Invensense Gyro with AD7799 ADC
				x = ((unsigned short)buffer[2]+((unsigned short)buffer[1]<<8)+((unsigned int)buffer[0]<<16)) - 0x800000;
				y = ((unsigned short)buffer[5]+((unsigned short)buffer[4]<<8)+((unsigned int)buffer[3]<<16)) - 0x800000;
				
				//vref
				z = ((unsigned short)buffer[8]+((unsigned short)buffer[7]<<8)+((unsigned int)buffer[6]<<16)) - 0x800000;
				
				
				Vx = x / (0x7FFFFF / 3.0);
				Vy = y / (0x7FFFFF / 3.0);
				
				Vz = z / (0x7FFFFF / 3.0);
				
				if (!phid->calibrated)
				{
					phid->calibCount++;
					phid->axisCalibration[0]+=x;
					phid->axisCalibration[1]+=y;
					phid->axisCalibration[2]+=z;
					if(phid->calibCount>=50)
					{
						phid->calibrated = PTRUE;
						phid->axisCalibration[0] /= 50;
						phid->axisCalibration[1] /= 50;
						phid->axisCalibration[2] /= 50;
						
						LOG(PHIDGET_LOG_INFO, "Gyro X axis zero point: 0x%06x",phid->axisCalibration[0]);
						LOG(PHIDGET_LOG_INFO, "Gyro Y axis zero point: 0x%06x",phid->axisCalibration[1]);
						LOG(PHIDGET_LOG_INFO, "Gyro Vref zero point: 0x%06x",phid->axisCalibration[2]);
						
						phid->calibCount = 0;
					}
				}
				else
				{
					axis[0] = round_double((((double)(x-phid->axisCalibration[0])) / ((0x7FFFFF / 3.0) * 0.002) ), 8);
					axis[1] = round_double((((double)(y-phid->axisCalibration[1])) / ((0x7FFFFF / 3.0) * 0.002)), 8);
					
					//to graph
					printf("%d, %d, %d\n",x-phid->axisCalibration[0],y-phid->axisCalibration[1],z-phid->axisCalibration[2]);
				}
				
				LOG(PHIDGET_LOG_INFO, "Gyro X axis: 0x%06x (%1.4lfV) (%2.4lfdeg/sec)  Y axis: 0x%06x (%1.4lfV) (%2.4lfdeg/sec)  Vref: %1.4lfV",x,Vx,axis[0],y,Vy,axis[1],Vz);
				
#endif
			}
			else if (phid->phid.deviceVersion >= 200 && phid->phid.deviceVersion < 300)
			{

#if 1 // This is the 1-axis ADXRS Gyro with AD7799 ADC
				z = ((unsigned short)buffer[2]+((unsigned short)buffer[1]<<8)+((unsigned int)buffer[0]<<16)) - 0x800000;
				
				//tempv
				y = ((unsigned short)buffer[5]+((unsigned short)buffer[4]<<8)+((unsigned int)buffer[3]<<16)) - 0x800000;
				
				Vz = z / (0x7FFFFF / 5.0);
				Vy = y / (0x7FFFFF / 5.0);
				
				if (!phid->calibrated)
				{
					phid->calibCount++;
					phid->axisCalibration[2]+=z;
					phid->axisCalibration[1]+=y;
					if(phid->calibCount>=150)
					{
						phid->calibrated = PTRUE;
						phid->axisCalibration[2] /= 150;
						phid->axisCalibration[1] /= 150;
						
						LOG(PHIDGET_LOG_INFO, "Gyro Z axis zero point: 0x%06x",phid->axisCalibration[2]);
						
						phid->calibCount = 0;
					}
				}
				else
				{
					axis[2] = ((double)(z-phid->axisCalibration[2])) / ((0x7FFFFF / 5.0) * 0.015);
					if(axis[2] > 75 || axis[2] < -75)
					{
						LOG(PHIDGET_LOG_ERROR, "Out of range Rate: Gyro Z axis: 0x%06x (%1.4lfV) (%02.4lfdeg/sec)",z,Vz,axis[2]);
						return 0;
					}
					
					//for graphing
					//printf("%d, %d\n",z-phid->axisCalibration[2], y-phid->axisCalibration[1]);
				}
				
				LOG(PHIDGET_LOG_INFO, "Gyro Z axis: 0x%06x (%1.4lfV) (%02.4lfdeg/sec)  TempV: %1.4lfV",z,Vz,axis[2], Vy);
#endif
			}
			break;
			break;
		default:
			break;
	}

	//Make sure values are within defined range, and store to structure
	for (i = 0; i<phid->phid.attr.accelerometer.numAxis; i++)
	{
		phid->axis[i] = axis[i];
	}

	//send out any events that exceed or match the trigger
	for (i = 0; i<phid->phid.attr.gyroscope.numAxis; i++)
	{
		if (fabs(phid->axis[i] - phid->axisLastTrigger[i]) >= phid->axisChangeTrigger[i]
			|| phid->axisLastTrigger[i] == PUNK_DBL)
		{
			FIRE(AngularRateChange, i, phid->axis[i]);
			phid->axisLastTrigger[i] = phid->axis[i];
		}
	}

	return EPHIDGET_OK;
}

//eventsAfterOpen - sends out an event for all valid data, used during attach initialization
CPHIDGETINITEVENTS(Gyroscope)
	int i = 0;

	for (i = 0; i<phid->phid.attr.gyroscope.numAxis; i++)
	{
		if(phid->axis[i] != PUNK_DBL)
			FIRE(AngularRateChange, i, phid->axis[i]);
	}

	return EPHIDGET_OK;
}

//getPacket - not used for Gyroscope
CGETPACKET(Gyroscope)
	return EPHIDGET_UNEXPECTED;
}

// === Exported Functions === //

//create and initialize a device structure
CCREATE(Gyroscope, PHIDCLASS_GYROSCOPE)

//event setup functions
CFHANDLE(Gyroscope, AngularRateChange, int, double)

CGET(Gyroscope,AxisCount,int)
	TESTPTRS(phid,pVal) 
	TESTDEVICETYPE(PHIDCLASS_GYROSCOPE)
	TESTATTACHED

	MASGN(phid.attr.gyroscope.numAxis)
}

CGETINDEX(Gyroscope,AngularRate,double)
	TESTPTRS(phid,pVal) 
	TESTDEVICETYPE(PHIDCLASS_GYROSCOPE)
	TESTATTACHED
	TESTINDEX(phid.attr.gyroscope.numAxis)
	TESTMASGN(axis[Index], PUNK_DBL)

	MASGN(axis[Index])
}

CGETINDEX(Gyroscope,AngularRateChangeTrigger,double)
	TESTPTRS(phid,pVal) 
	TESTDEVICETYPE(PHIDCLASS_GYROSCOPE)
	TESTATTACHED
	TESTINDEX(phid.attr.gyroscope.numAxis)
	TESTMASGN(axisChangeTrigger[Index], PUNK_DBL)

	MASGN(axisChangeTrigger[Index])
}
CSETINDEX(Gyroscope,AngularRateChangeTrigger,double)
	TESTPTR(phid) 
	TESTDEVICETYPE(PHIDCLASS_GYROSCOPE)
	TESTATTACHED
	TESTINDEX(phid.attr.gyroscope.numAxis)

	if(CPhidget_statusFlagIsSet(phid->phid.status, PHIDGET_REMOTE_FLAG))
		ADDNETWORKKEYINDEXED(Trigger, "%lE", axisChangeTrigger);
	else
		phid->axisChangeTrigger[Index] = newVal;

	return EPHIDGET_OK;
}

//stubs for exports file
#else

void CPhidgetGyroscope_create(){}
void CPhidgetGyroscope_getAngularRateChangeTrigger(){}
void CPhidgetGyroscope_setAngularRateChangeTrigger(){}
void CPhidgetGyroscope_getAxisCount(){}
void CPhidgetGyroscope_getAngularRate(){}
void CPhidgetGyroscope_set_OnAngularRateChange_Handler(){}

#endif

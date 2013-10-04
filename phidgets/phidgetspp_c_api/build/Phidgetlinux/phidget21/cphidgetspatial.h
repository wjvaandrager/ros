#ifndef __CPHIDGETSPATIAL
#define __CPHIDGETSPATIAL
#include "cphidget.h"

/** \defgroup phidaccel Phidget Spatial 
 * \ingroup phidgets
 * Calls specific to the Phidget Spatial. See the product manual for more specific API details, supported functionality, units, etc.
 * @{
 */

DPHANDLE(Spatial)
CHDRSTANDARD(Spatial)

#define SPATIAL_MAX_ACCELAXES 3
#define SPATIAL_MAX_GYROAXES 3
#define SPATIAL_MAX_COMPASSAXES 3

typedef struct _CPhidgetSpatial_SpatialEventData
{
	double acceleration[SPATIAL_MAX_ACCELAXES];
	double angularRate[SPATIAL_MAX_GYROAXES];
	double magneticField[SPATIAL_MAX_COMPASSAXES];
	CPhidget_Timestamp timestamp;
} CPhidgetSpatial_SpatialEventData, *CPhidgetSpatial_SpatialEventDataHandle;

CHDRGET(Spatial,AccelerationAxisCount,int *count)
CHDRGET(Spatial,GyroAxisCount,int *count)
CHDRGET(Spatial,CompassAxisCount,int *count)

CHDRGETINDEX(Spatial,Acceleration,double *acceleration)
CHDRGETINDEX(Spatial,AccelerationMax,double *max)
CHDRGETINDEX(Spatial,AccelerationMin,double *min)

CHDRGETINDEX(Spatial,AngularRate,double *angularRate)
CHDRGETINDEX(Spatial,AngularRateMax,double *max)
CHDRGETINDEX(Spatial,AngularRateMin,double *min)

CHDRGETINDEX(Spatial,MagneticField,double *magneticField)
CHDRGETINDEX(Spatial,MagneticFieldMax,double *max)
CHDRGETINDEX(Spatial,MagneticFieldMin,double *min)

//Add these later
//CHDRGET(Spatial, GyroHeading, double *heading)
//CHDRGET(Spatial, CompassHeading, double *heading)

PHIDGET21_API int CCONV CPhidgetSpatial_zeroGyro(CPhidgetSpatialHandle phid);

//This is the event rate
//You only get more then one set of data per event if the event rate is > the interrupt rate
//since we're not going to run an extra thread, the accuracy of the data rate is limited by the interrupt endpoint data rate (>=8ms)
CHDRGET(Spatial, DataRate, int *milliseconds)
CHDRSET(Spatial, DataRate, int milliseconds)
CHDRGET(Spatial, DataRateMax, int *max)
CHDRGET(Spatial, DataRateMin, int *min)

CHDREVENT(Spatial,SpatialData,CPhidgetSpatial_SpatialEventDataHandle *data, int dataCount)

#ifndef EXTERNALPROTO

//in milliseconds - this is the fastest hardware rate of any device
#define SPATIAL_MAX_DATA_RATE 2
//1 second is the longest between events that we support
#define SPATIAL_MIN_DATA_RATE 2000
//add 200ms for timing differences (late events, etc) - should be plenty
#define SPATIAL_DATA_BUFFER_SIZE ((SPATIAL_MIN_DATA_RATE + 200)/SPATIAL_MAX_DATA_RATE)
//1 second of data to zero the gyro - make sure DATA_BUFFER_SIZE is big enough to hold this much data
#define SPATIAL_ZERO_GYRO_TIME 2000

//packet types
//IN
#define SPACIAL_PACKET_DATA	0x00
#define SPACIAL_PACKET_CALIB 0x80
//OUT
#define SPACIAL_READCALIB 0x01

struct _CPhidgetSpatial {
	CPhidget phid;
	int (CCONV *fptrSpatialData)(CPhidgetSpatialHandle, void *, CPhidgetSpatial_SpatialEventDataHandle *, int);           
	void *fptrSpatialDataptr;

	//double gyroHeading;
	//double compassHeading;

	double accelAxis[SPATIAL_MAX_ACCELAXES];
	double gyroAxis[SPATIAL_MAX_GYROAXES];
	double compassAxis[SPATIAL_MAX_COMPASSAXES];

	double gryoCorrection[SPATIAL_MAX_GYROAXES];
	unsigned char doZeroGyro;
	int gyroZeroReadPtr;

	CPhidgetSpatial_SpatialEventData dataBuffer[SPATIAL_DATA_BUFFER_SIZE];
	int bufferReadPtr, bufferWritePtr;

	double accelerationMax, accelerationMin;
	double angularRateMax, angularRateMin;
	double magneticFieldMax, magneticFieldMin;

	unsigned char calDataValid;
	double accelGain1[SPATIAL_MAX_ACCELAXES];
	double accelGain2[SPATIAL_MAX_ACCELAXES];
	int accelOffset[SPATIAL_MAX_ACCELAXES];
	double gyroGain1[SPATIAL_MAX_GYROAXES];
	double gyroGain2[SPATIAL_MAX_GYROAXES];
	int gyroOffset[SPATIAL_MAX_GYROAXES];
	
	CPhidget_Timestamp timestamp, lastEventTime, latestDataTime;

	unsigned char lastTimeCounterValid;
	int lastTimeCounter;

	int dataRate, interruptRate;
	int dataRateMax, dataRateMin;

} typedef CPhidgetSpatialInfo;
#endif

/** @} */

#endif

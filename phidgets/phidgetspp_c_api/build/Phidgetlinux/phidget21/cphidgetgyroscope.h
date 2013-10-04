
#ifndef __CPHIDGETGYROSCOPE
#define __CPHIDGETGYROSCOPE
#include "cphidget.h"

/** \defgroup phidgyro Phidget Gyroscope 
 * \ingroup phidgets
 * Calls specific to the Phidget Gyroscope. See the product manual for more specific API details, supported functionality, units, etc.
 * @{
 */

DPHANDLE(Gyroscope)
#if 0 //Don't include
CHDRSTANDARD(Gyroscope)

CHDRGET(Gyroscope,AxisCount,int *)

CHDRGETINDEX(Gyroscope,AngularRate,double *)
CHDREVENTINDEX(Gyroscope,AngularRateChange,double angularRate)
CHDRGETINDEX(Gyroscope,AngularRateChangeTrigger,double *)
CHDRSETINDEX(Gyroscope,AngularRateChangeTrigger,double)
#endif

#ifndef EXTERNALPROTO
#define GYRO_MAXAXES 3
struct _CPhidgetGyroscope {
	CPhidget phid;

	int (CCONV *fptrAngularRateChange)(CPhidgetGyroscopeHandle, void *, int, double);           
	void *fptrAngularRateChangeptr;

	double axis[GYRO_MAXAXES];
	double axisChangeTrigger[GYRO_MAXAXES];
	double axisLastTrigger[GYRO_MAXAXES];
	
	int axisCalibration[GYRO_MAXAXES];
	unsigned char calibrated;
	int calibCount;
} typedef CPhidgetGyroscopeInfo;
#endif

/** @} */

#endif

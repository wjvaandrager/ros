#ifndef _EVENTHANDLERS
#define _EVENTHANDLERS
#include "stdafx.h"

int CCONV attach_handler(CPhidgetHandle phid, void *pdss);
int CCONV detach_handler(CPhidgetHandle phid, void *pdss);
int CCONV Accelerometer_AccelerationChange(CPhidgetAccelerometerHandle phid, void *pdss, int Index, double Val);
int CCONV AdvancedServo_PositionChange(CPhidgetAdvancedServoHandle phid, void *pdss, int Index, double Val);
int CCONV AdvancedServo_VelocityChange(CPhidgetAdvancedServoHandle phid, void *pdss, int Index, double Val);
int CCONV AdvancedServo_CurrentChange(CPhidgetAdvancedServoHandle phid, void *pdss, int Index, double Val);
int CCONV Encoder_InputChange(CPhidgetEncoderHandle phid, void *pdss, int Index, int Val);
int CCONV Encoder_PositionChange(CPhidgetEncoderHandle phid, void *pdss, int Index, int Time, int PositionChange);
//int CCONV GPS_NMEAData(CPhidgetGPSHandle phid, void *pdss, const char *data);
//int CCONV Gyroscope_AngularRateChange(CPhidgetGyroscopeHandle phid, void *pdss, int Index, double Val);
int CCONV InterfaceKit_InputChange(CPhidgetInterfaceKitHandle phid, void *pdss, int Index, int Val);
int CCONV InterfaceKit_OutputChange(CPhidgetInterfaceKitHandle phid, void *pdss, int Index, int Val);
int CCONV InterfaceKit_SensorChange(CPhidgetInterfaceKitHandle phid, void *pdss, int Index, int Val);
int CCONV MotorControl_InputChange(CPhidgetMotorControlHandle phid, void *pdss, int Index, int Val);
int CCONV MotorControl_VelocityChange(CPhidgetMotorControlHandle phid, void *pdss, int Index, double Val);
int CCONV MotorControl_CurrentChange(CPhidgetMotorControlHandle phid, void *pdss, int Index, double val);
int CCONV PHSensor_PHChange(CPhidgetPHSensorHandle phid, void *pdss, double Val);
int CCONV RFID_Tag(CPhidgetRFIDHandle phid, void *pdss, unsigned char *Tag);
int CCONV RFID_TagLost(CPhidgetRFIDHandle phid, void *pdss, unsigned char *Tag);
int CCONV RFID_OutputChange(CPhidgetRFIDHandle phid, void *pdss, int Index, int Val);
int CCONV Servo_PositionChange(CPhidgetServoHandle phid, void *pdss, int Index, double Val);
int CCONV Stepper_InputChange(CPhidgetStepperHandle phid, void *pdss, int Index, int Val);
int CCONV Stepper_PositionChange(CPhidgetStepperHandle phid, void *pdss, int Index, long long Val);
int CCONV Stepper_VelocityChange(CPhidgetStepperHandle phid, void *pdss, int Index, double Val);
int CCONV Stepper_CurrentChange(CPhidgetStepperHandle phid, void *pdss, int Index, double Val);
int CCONV TemperatureSensor_TemperatureChange(CPhidgetTemperatureSensorHandle phid, void *pdss, int Index, double Val);
int CCONV WeightSensor_WeightChange(CPhidgetWeightSensorHandle phid, void *pdss, double Val);

void phidget_set(const char *k, const char *v, pdict_reason_t r, const char *pde_oldval, void *arg);
void phidget_openclose(const char *k, const char *v, pdict_reason_t r, const char *pde_oldval, void *arg);

#endif

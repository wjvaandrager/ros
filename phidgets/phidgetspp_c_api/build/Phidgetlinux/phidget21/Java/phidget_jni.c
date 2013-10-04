#include "../stdafx.h"
#include <stdlib.h>
#include "../cphidget.h"
#include "phidget_jni.h"
#include <stdio.h>

jclass ph_exception_class;
jmethodID ph_exception_cons;
JavaVM *ph_vm = 0;

jint JNICALL
JNI_OnLoad(JavaVM *vm, void *reserved)
{
	JNIEnv *env;
	jint version = 0;
	jint result;

	ph_vm = vm;

	result = (*vm)->GetEnv(vm, (void **)&env, JNI_VERSION_1_4);
	
	if(result == JNI_EDETACHED)
	{
		if ((*vm)->AttachCurrentThread(vm, (void **)&env, NULL))
		{
			LOG(PHIDGET_LOG_WARNING,"Couldn't Attach Thread");
			(*env)->ExceptionDescribe(env);
			(*env)->ExceptionClear(env);
			abort();
		}
	}
	
	if(!(version = (*env)->GetVersion(env)))
	{
		LOG(PHIDGET_LOG_WARNING,"Couldn't get version");
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	LOG(PHIDGET_LOG_DEBUG,"JNI Version: %08x",version);

	if (!(ph_exception_class = (*env)->FindClass(env, 
	  "com/phidgets/PhidgetException")))
	{
		LOG(PHIDGET_LOG_WARNING,"Coulnd't find class \"com/phidgets/PhidgetException\"");
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}

	if (!(ph_exception_class = (jclass)(*env)->NewGlobalRef(env, 
	  ph_exception_class)))
	{
		LOG(PHIDGET_LOG_WARNING,"Couldn't create global ref ph_exception_class");
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(ph_exception_cons = (*env)->GetMethodID(env, ph_exception_class, "<init>",
	  "(ILjava/lang/String;)V")))
	{
		LOG(PHIDGET_LOG_WARNING,"Couldn't get Method ID for init in ph_exception_class");
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}

	com_phidgets_Phidget_OnLoad(env);
	com_phidgets_InterfaceKitPhidget_OnLoad(env);
	com_phidgets_AccelerometerPhidget_OnLoad(env);
	com_phidgets_AdvancedServoPhidget_OnLoad(env);
	com_phidgets_EncoderPhidget_OnLoad(env);
	com_phidgets_LEDPhidget_OnLoad(env);
	com_phidgets_MotorControlPhidget_OnLoad(env);
	com_phidgets_PHSensorPhidget_OnLoad(env);
	com_phidgets_ServoPhidget_OnLoad(env);
	com_phidgets_StepperPhidget_OnLoad(env);
	com_phidgets_TemperatureSensorPhidget_OnLoad(env);
	com_phidgets_TextLCDPhidget_OnLoad(env);
	com_phidgets_TextLEDPhidget_OnLoad(env);
	com_phidgets_WeightSensorPhidget_OnLoad(env);
	com_phidgets_RFIDPhidget_OnLoad(env);
	com_phidgets_Manager_OnLoad(env);
	com_phidgets_Dictionary_OnLoad(env);
	com_phidgets_DictionaryKeyListener_OnLoad(env);

	//(*vm)->DetachCurrentThread(vm); //this seems to break java 1.4.2 - why?
	//because this is always connected - and shouldn't be disconnected...


	return JNI_VERSION_1_4;
}

jlong
updateGlobalRef(JNIEnv *env, jobject obj, jfieldID fid, jboolean b)
{
	/*
	 * Manages the global reference held by phidget21 to the handler
	 * target.
	 */
	jlong gr;

	if ((gr = (*env)->GetLongField(env, obj, fid)) != 0)
		(*env)->DeleteGlobalRef(env, (jobject)(uintptr_t)gr);
	gr = b ? (jlong)(uintptr_t)(*env)->NewGlobalRef(env, obj) : 0;
	(*env)->SetLongField(env, obj, fid, gr);

	return gr;
}

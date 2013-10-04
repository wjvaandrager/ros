
#include "../stdafx.h"
#include "phidget_jni.h"
#include "com_phidgets_InterfaceKitPhidget.h"
#include "../cphidgetinterfacekit.h"

extern jfieldID handle_fid;

static jmethodID inputChangeEvent_cons;
static jmethodID outputChangeEvent_cons;
static jmethodID sensorChangeEvent_cons;
static jmethodID fireInputChange_mid;
static jmethodID fireOutputChange_mid;
static jmethodID fireSensorChange_mid;
static jclass ifkit_class;
static jclass inputChangeEvent_class;
static jclass outputChangeEvent_class;
static jclass sensorChangeEvent_class;
static jfieldID nativeInputChangeHandler_fid;
static jfieldID nativeOutputChangeHandler_fid;
static jfieldID nativeSensorChangeHandler_fid;

void
com_phidgets_InterfaceKitPhidget_OnLoad(JNIEnv *env)
{
	if (!(ifkit_class = (*env)->FindClass(env,
	    "com/phidgets/InterfaceKitPhidget")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(ifkit_class = (jclass)(*env)->NewGlobalRef(env, ifkit_class)))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(inputChangeEvent_class = (*env)->FindClass(env, 
	    "com/phidgets/event/InputChangeEvent")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(inputChangeEvent_class = (jclass)(*env)->NewGlobalRef(env, 
	    inputChangeEvent_class)))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(outputChangeEvent_class = (*env)->FindClass(env, 
	    "com/phidgets/event/OutputChangeEvent")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(outputChangeEvent_class = (jclass)(*env)->NewGlobalRef(env, 
	    outputChangeEvent_class)))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(sensorChangeEvent_class = (*env)->FindClass(env, 
	    "com/phidgets/event/SensorChangeEvent")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(sensorChangeEvent_class = (jclass)(*env)->NewGlobalRef(env,
	    sensorChangeEvent_class)))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(fireInputChange_mid = (*env)->GetMethodID(env, ifkit_class,
	    "fireInputChange", "(Lcom/phidgets/event/InputChangeEvent;)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(fireOutputChange_mid = (*env)->GetMethodID(env, ifkit_class,
	    "fireOutputChange", "(Lcom/phidgets/event/OutputChangeEvent;)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(fireSensorChange_mid = (*env)->GetMethodID(env, ifkit_class,
	    "fireSensorChange", "(Lcom/phidgets/event/SensorChangeEvent;)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(inputChangeEvent_cons = (*env)->GetMethodID(env,
	    inputChangeEvent_class, "<init>", "(Lcom/phidgets/Phidget;IZ)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(outputChangeEvent_cons = (*env)->GetMethodID(env,
	    outputChangeEvent_class, "<init>", "(Lcom/phidgets/Phidget;IZ)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(sensorChangeEvent_cons = (*env)->GetMethodID(env,
	    sensorChangeEvent_class, "<init>", "(Lcom/phidgets/Phidget;II)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(nativeInputChangeHandler_fid = (*env)->GetFieldID(env,
	    ifkit_class, "nativeInputChangeHandler", "J")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(nativeOutputChangeHandler_fid = (*env)->GetFieldID(env,
	    ifkit_class, "nativeOutputChangeHandler", "J")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(nativeSensorChangeHandler_fid = (*env)->GetFieldID(env,
	    ifkit_class, "nativeSensorChangeHandler", "J")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
}

#define DEFINE_BASIC_EVENT_HANDLER_FUNCS(event, Event, cphidgetSetHandlerFunc) \
static int CCONV event##_handler(CPhidgetInterfaceKitHandle h, void *arg, \
  int, int); \
\
JNIEXPORT void JNICALL \
Java_com_phidgets_InterfaceKitPhidget_enable##Event##Events(JNIEnv *env, \
  jobject obj, jboolean b) \
{ \
	jlong gr = updateGlobalRef(env, obj, native##Event##Handler_fid, b); \
	CPhidgetInterfaceKitHandle h = (CPhidgetInterfaceKitHandle)(uintptr_t) \
	    (*env)->GetLongField(env, obj, handle_fid); \
\
	cphidgetSetHandlerFunc(h, b ? event##_handler : 0, \
	    (void *)(uintptr_t)gr); \
} \
\
static int CCONV \
event##_handler(CPhidgetInterfaceKitHandle h, void *arg, int index, int v) \
{ \
	JNIEnv *env; \
	jobject obj; \
	jobject event##Ev; \
\
	if ((*ph_vm)->AttachCurrentThread(ph_vm, (void **)&env, NULL)) \
		abort(); \
\
	obj = (jobject)arg; \
\
	if (!(event##Ev = (*env)->NewObject(env, event##Event_class, \
	    event##Event_cons, obj, index, v))) { \
		(*ph_vm)->DetachCurrentThread(ph_vm); \
		return -1; \
	} \
	(*env)->CallVoidMethod(env, obj, fire##Event##_mid, event##Ev); \
	(*env)->DeleteLocalRef(env, event##Ev); \
	(*ph_vm)->DetachCurrentThread(ph_vm); \
\
	return 0; \
}

DEFINE_BASIC_EVENT_HANDLER_FUNCS(inputChange, InputChange,
  CPhidgetInterfaceKit_set_OnInputChange_Handler)
DEFINE_BASIC_EVENT_HANDLER_FUNCS(outputChange, OutputChange,
  CPhidgetInterfaceKit_set_OnOutputChange_Handler)
DEFINE_BASIC_EVENT_HANDLER_FUNCS(sensorChange, SensorChange,
  CPhidgetInterfaceKit_set_OnSensorChange_Handler)

JNIEXPORT jlong JNICALL
Java_com_phidgets_InterfaceKitPhidget_create(JNIEnv *env, jclass cls)
{
	CPhidgetInterfaceKitHandle phid; 
	int error;

	if ((error = CPhidgetInterfaceKit_create(&phid)) != EPHIDGET_OK) {
		PH_THROW(error);
		return 0;
	}

	return (jlong)(uintptr_t)phid; 
}

JNIEXPORT jint JNICALL
Java_com_phidgets_InterfaceKitPhidget_getOutputCount (JNIEnv *env, jobject obj)
{
	CPhidgetInterfaceKitHandle h = (CPhidgetInterfaceKitHandle)(uintptr_t)
	    (*env)->GetLongField( env, obj, handle_fid);
	int error;
	int no;
	
	if ((error = CPhidgetInterfaceKit_getOutputCount(h, &no)))
		PH_THROW(error);

	return no;
}

JNIEXPORT jint JNICALL
Java_com_phidgets_InterfaceKitPhidget_getInputCount(JNIEnv *env, jobject obj)
{
	CPhidgetInterfaceKitHandle h = (CPhidgetInterfaceKitHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, handle_fid);
	int error;
	int ni;
	
	if ((error = CPhidgetInterfaceKit_getInputCount(h, &ni)))
		PH_THROW(error);

	return ni;
}

JNIEXPORT jint JNICALL
Java_com_phidgets_InterfaceKitPhidget_getSensorCount(JNIEnv *env, jobject obj)
{
	CPhidgetInterfaceKitHandle h = (CPhidgetInterfaceKitHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, handle_fid);
	int error;
	int ns;
	
	if ((error = CPhidgetInterfaceKit_getSensorCount(h, &ns)))
		PH_THROW(error);

	return ns;
}

JNIEXPORT jboolean JNICALL
Java_com_phidgets_InterfaceKitPhidget_getInputState(JNIEnv *env, jobject obj,
  jint index)
{
	CPhidgetInterfaceKitHandle h = (CPhidgetInterfaceKitHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, handle_fid);
	int error;
	int v;
	
	if ((error = CPhidgetInterfaceKit_getInputState(h, index, &v)))
		PH_THROW(error);

	return v;
}

JNIEXPORT jboolean JNICALL
Java_com_phidgets_InterfaceKitPhidget_getOutputState(JNIEnv *env, jobject obj,
  jint index)
{
	CPhidgetInterfaceKitHandle h = (CPhidgetInterfaceKitHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, handle_fid);
	int error;
	int v;
	
	if ((error = CPhidgetInterfaceKit_getOutputState(h, index, &v)))
		PH_THROW(error);

	return v;
}

JNIEXPORT jint JNICALL
Java_com_phidgets_InterfaceKitPhidget_getSensorValue(JNIEnv *env, jobject obj,
  jint index)
{
	CPhidgetInterfaceKitHandle h = (CPhidgetInterfaceKitHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, handle_fid);
	int error;
	int v;
	
	if ((error = CPhidgetInterfaceKit_getSensorValue(h, index, &v)))
		PH_THROW(error);

	return v;
}

JNIEXPORT jint JNICALL
Java_com_phidgets_InterfaceKitPhidget_getSensorRawValue(JNIEnv *env, jobject obj,
  jint index)
{
	CPhidgetInterfaceKitHandle h = (CPhidgetInterfaceKitHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, handle_fid);
	int error;
	int v;
	
	if ((error = CPhidgetInterfaceKit_getSensorRawValue(h, index, &v)))
		PH_THROW(error);

	return v;
}

JNIEXPORT jint JNICALL
Java_com_phidgets_InterfaceKitPhidget_getSensorChangeTrigger(JNIEnv *env,
  jobject obj, jint index)
{
	CPhidgetInterfaceKitHandle h = (CPhidgetInterfaceKitHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, handle_fid);
	int error;
	int v;
	
	if ((error = CPhidgetInterfaceKit_getSensorChangeTrigger(h, index, &v)))
		PH_THROW(error);

	return v;
}

JNIEXPORT jboolean JNICALL
Java_com_phidgets_InterfaceKitPhidget_getRatiometric(JNIEnv *env,
  jobject obj)
{
	CPhidgetInterfaceKitHandle h = (CPhidgetInterfaceKitHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, handle_fid);
	int error;
	int v;
	
	if ((error = CPhidgetInterfaceKit_getRatiometric(h, &v)))
		PH_THROW(error);

	if(v) return 1;
	return 0;
}

JNIEXPORT void JNICALL
Java_com_phidgets_InterfaceKitPhidget_setOutputState(JNIEnv *env, jobject obj,
  jint index, jboolean b)
{
	CPhidgetInterfaceKitHandle h = (CPhidgetInterfaceKitHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, handle_fid);
	int error;
	
	if ((error = CPhidgetInterfaceKit_setOutputState(h, index, b))) {
		PH_THROW(error);
	}
}

JNIEXPORT void JNICALL
Java_com_phidgets_InterfaceKitPhidget_setSensorChangeTrigger(JNIEnv *env,
  jobject obj, jint index, jint v)
{
	CPhidgetInterfaceKitHandle h = (CPhidgetInterfaceKitHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, handle_fid);
	int error;
	
	if ((error = CPhidgetInterfaceKit_setSensorChangeTrigger(h, index, v)))
		PH_THROW(error);
}

JNIEXPORT void JNICALL
Java_com_phidgets_InterfaceKitPhidget_setRatiometric(JNIEnv *env,
  jobject obj, jboolean v)
{
	CPhidgetInterfaceKitHandle h = (CPhidgetInterfaceKitHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, handle_fid);
	int error;
	
	if ((error = CPhidgetInterfaceKit_setRatiometric(h, v)))
		PH_THROW(error);
}

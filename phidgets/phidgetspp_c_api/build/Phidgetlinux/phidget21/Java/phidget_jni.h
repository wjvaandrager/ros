#ifndef PHIDGET_JNI_H
#define PHIDGET_JNI_H

#include <jni.h>


extern jclass ph_exception_class;
extern jmethodID ph_exception_cons;
extern JavaVM *ph_vm;

void com_phidgets_Phidget_OnLoad(JNIEnv *);
void com_phidgets_InterfaceKitPhidget_OnLoad(JNIEnv *);
void com_phidgets_AccelerometerPhidget_OnLoad(JNIEnv *);
void com_phidgets_AdvancedServoPhidget_OnLoad(JNIEnv *);
void com_phidgets_EncoderPhidget_OnLoad(JNIEnv *);
void com_phidgets_LEDPhidget_OnLoad(JNIEnv *);
void com_phidgets_MotorControlPhidget_OnLoad(JNIEnv *);
void com_phidgets_PHSensorPhidget_OnLoad(JNIEnv *);
void com_phidgets_ServoPhidget_OnLoad(JNIEnv *);
void com_phidgets_StepperPhidget_OnLoad(JNIEnv *);
void com_phidgets_TemperatureSensorPhidget_OnLoad(JNIEnv *);
void com_phidgets_TextLCDPhidget_OnLoad(JNIEnv *);
void com_phidgets_TextLEDPhidget_OnLoad(JNIEnv *);
void com_phidgets_WeightSensorPhidget_OnLoad(JNIEnv *);
void com_phidgets_RFIDPhidget_OnLoad(JNIEnv *);
void com_phidgets_Manager_OnLoad(JNIEnv *);
void com_phidgets_Dictionary_OnLoad(JNIEnv *);
void com_phidgets_DictionaryKeyListener_OnLoad(JNIEnv *);

jlong updateGlobalRef(JNIEnv *env, jobject obj, jfieldID fid, jboolean b);



#define EVENT_VARS(event, Event) static jmethodID event##Event_cons; \
static jmethodID fire##Event##_mid; \
static jclass event##Event_class; \
static jfieldID native##Event##Handler_fid;

#define JNI_LOAD(name, Pname) extern jfieldID handle_fid; \
static jclass name##_class; \
void \
com_phidgets_##Pname##Phidget_OnLoad(JNIEnv *env) \
{ \
	if (!(name##_class = (*env)->FindClass(env, \
		"com/phidgets/" #Pname "Phidget"))) \
	{ \
		LOG(PHIDGET_LOG_WARNING,"Couldn't FindClass com/phidgets/" #Pname "Phidget"); \
		(*env)->ExceptionDescribe(env); \
		(*env)->ExceptionClear(env); \
		abort(); \
	} \
	if (!(name##_class = (jclass)(*env)->NewGlobalRef(env, name##_class))) \
	{ \
		LOG(PHIDGET_LOG_WARNING,"Couldn't create NewGlobalRef " #name "_class"); \
		(*env)->ExceptionDescribe(env); \
		(*env)->ExceptionClear(env); \
		abort(); \
	} \

#define EVENT_VAR_SETUP(name, event, Event, parameters, returntype) if (!(event##Event_class = (*env)->FindClass(env, \
"com/phidgets/event/" #Event "Event"))) \
		abort(); \
	if (!(event##Event_class = (jclass)(*env)->NewGlobalRef(env, \
	    event##Event_class))) \
	{ \
		LOG(PHIDGET_LOG_WARNING,"Couldn't create global ref " #event "Event_class"); \
		(*env)->ExceptionDescribe(env); \
		(*env)->ExceptionClear(env); \
		abort(); \
	} \
	if (!(fire##Event##_mid = (*env)->GetMethodID(env, name##_class, \
	"fire" #Event , "(Lcom/phidgets/event/" #Event "Event;)" #returntype ))) \
	{ \
		LOG(PHIDGET_LOG_WARNING,"Couldn't get method ID fire" #Event); \
		(*env)->ExceptionDescribe(env); \
		(*env)->ExceptionClear(env); \
		abort(); \
	} \
	if (!(event##Event_cons = (*env)->GetMethodID(env, \
		event##Event_class, "<init>", "(Lcom/phidgets/Phidget;" #parameters ")" #returntype ))) \
	{ \
		LOG(PHIDGET_LOG_WARNING,"Couldn't get method ID <init> from " #event "Event_class"); \
		(*env)->ExceptionDescribe(env); \
		(*env)->ExceptionClear(env); \
		abort(); \
	} \
	if (!(native##Event##Handler_fid = (*env)->GetFieldID(env, \
	name##_class, "native" #Event "Handler", "J"))) \
	{ \
		LOG(PHIDGET_LOG_WARNING,"Couldn't get Field ID native" #Event "Handler from " #name "_class"); \
		(*env)->ExceptionDescribe(env); \
		(*env)->ExceptionClear(env); \
		abort(); \
	}

#define PH_THROW(errno) { \
	jobject eobj; \
	jstring edesc; \
 \
	if (!(edesc = (*env)->NewStringUTF(env, CPhidget_strerror(error)))) \
	{ \
		LOG(PHIDGET_LOG_WARNING,"Couldn't get NewStringUTF"); \
		(*env)->ExceptionDescribe(env); \
		(*env)->ExceptionClear(env); \
		abort(); \
	} \
	if (!(eobj = (*env)->NewObject(env, ph_exception_class, \
	  ph_exception_cons, error, edesc))) \
	{ \
		LOG(PHIDGET_LOG_WARNING,"Couldn't get NewObject ph_exception_class"); \
		(*env)->ExceptionDescribe(env); \
		(*env)->ExceptionClear(env); \
		abort(); \
	} \
	(*env)->Throw(env, (jthrowable)eobj); \
}

#define JNI_INDEXED_SETFUNC(pname, fname, lfname, type) JNIEXPORT void JNICALL \
	Java_com_phidgets_##pname##Phidget_set##fname(JNIEnv *env, jobject obj, jint index, type v) \
{ \
	CPhidget##pname##Handle h = (CPhidget##pname##Handle)(uintptr_t) \
	    (*env)->GetLongField(env, obj, handle_fid); \
	int error; \
	if ((error = CPhidget##pname##_set##lfname(h, index, v))) \
		PH_THROW(error); \
}

#define JNI_SETFUNC(pname, fname, lfname, type) JNIEXPORT void JNICALL \
	Java_com_phidgets_##pname##Phidget_set##fname(JNIEnv *env, jobject obj, type v) \
{ \
	CPhidget##pname##Handle h = (CPhidget##pname##Handle)(uintptr_t) \
	    (*env)->GetLongField(env, obj, handle_fid); \
	int error; \
	if ((error = CPhidget##pname##_set##lfname(h, v))) \
		PH_THROW(error); \
}

#define JNI_INDEXED_GETFUNC(pname, fname, lfname, type) JNIEXPORT type JNICALL \
Java_com_phidgets_##pname##Phidget_get##fname(JNIEnv *env, jobject obj, jint index) \
{ \
	CPhidget##pname##Handle h = (CPhidget##pname##Handle)(uintptr_t) \
	    (*env)->GetLongField(env, obj, handle_fid); \
	int error; \
	type v; \
	if ((error = CPhidget##pname##_get##lfname(h, index, &v))) \
		PH_THROW(error); \
	return v; \
}

#define JNI_INDEXED_GETFUNCBOOL(pname, fname, lfname) JNIEXPORT jboolean JNICALL \
Java_com_phidgets_##pname##Phidget_get##fname(JNIEnv *env, jobject obj, jint index) \
{ \
	CPhidget##pname##Handle h = (CPhidget##pname##Handle)(uintptr_t) \
	    (*env)->GetLongField(env, obj, handle_fid); \
	int error; \
	int v; \
	if ((error = CPhidget##pname##_get##lfname(h, index, &v))) \
		PH_THROW(error); \
	if (v) return 1; \
	return 0; \
}

#define JNI_GETFUNC(pname, fname, lfname, type) JNIEXPORT type JNICALL \
Java_com_phidgets_##pname##Phidget_get##fname (JNIEnv *env, jobject obj) \
{ \
	CPhidget##pname##Handle h = (CPhidget##pname##Handle)(uintptr_t) \
	    (*env)->GetLongField( env, obj, handle_fid); \
	int error; \
	type no; \
	if ((error = CPhidget##pname##_get##lfname(h, &no))) \
		PH_THROW(error); \
	return no; \
}

#define JNI_GETFUNCBOOL(pname, fname, lfname) JNIEXPORT jboolean JNICALL \
Java_com_phidgets_##pname##Phidget_get##fname (JNIEnv *env, jobject obj) \
{ \
	CPhidget##pname##Handle h = (CPhidget##pname##Handle)(uintptr_t) \
	    (*env)->GetLongField( env, obj, handle_fid); \
	int error; \
	int no; \
	if ((error = CPhidget##pname##_get##lfname(h, &no))) \
		PH_THROW(error); \
	if(no) return 1; \
	return 0; \
}

#define JNI_CREATE(Pname) JNIEXPORT jlong JNICALL \
	Java_com_phidgets_##Pname##Phidget_create(JNIEnv *env, jclass cls) \
{ \
	CPhidget##Pname##Handle phid; \
	int error; \
	if ((error = CPhidget##Pname##_create(&phid)) != EPHIDGET_OK) { \
		PH_THROW(error); \
		return 0; \
	} \
	return (jlong)(uintptr_t)phid; \
}
#define EVENT_HANDLER(pname, event, Event, cphidgetSetHandlerFunc, type) \
	static int CCONV event##_handler(CPhidget##pname##Handle h, void *arg, \
	type); \
\
JNIEXPORT void JNICALL \
Java_com_phidgets_##pname##Phidget_enable##Event##Events(JNIEnv *env, \
  jobject obj, jboolean b) \
{ \
	jlong gr = updateGlobalRef(env, obj, native##Event##Handler_fid, b); \
	CPhidget##pname##Handle h = (CPhidget##pname##Handle)(uintptr_t) \
	    (*env)->GetLongField(env, obj, handle_fid); \
	cphidgetSetHandlerFunc(h, b ? event##_handler : 0, \
	    (void *)(uintptr_t)gr); \
} \
\
static int CCONV \
event##_handler(CPhidget##pname##Handle h, void *arg, type v) \
{ \
	JNIEnv *env; \
	jobject obj; \
	jobject event##Ev; \
\
	if ((*ph_vm)->AttachCurrentThread(ph_vm, (void **)&env, NULL)) \
	{ \
		LOG(PHIDGET_LOG_WARNING,"Couldn't AttachCurrentThread"); \
		(*env)->ExceptionDescribe(env); \
		(*env)->ExceptionClear(env); \
		abort(); \
	} \
\
	obj = (jobject)arg; \
\
	if (!(event##Ev = (*env)->NewObject(env, event##Event_class, \
	    event##Event_cons, obj, v))) \
		return -1; \
	(*env)->CallVoidMethod(env, obj, fire##Event##_mid, event##Ev); \
	(*env)->DeleteLocalRef(env, event##Ev); \
	(*ph_vm)->DetachCurrentThread(ph_vm); \
\
	return 0; \
}

#define EVENT_HANDLER_INDEXED(pname, event, Event, cphidgetSetHandlerFunc, type) \
	static int CCONV event##_handler(CPhidget##pname##Handle h, void *arg, \
	int, type); \
\
JNIEXPORT void JNICALL \
Java_com_phidgets_##pname##Phidget_enable##Event##Events(JNIEnv *env, \
  jobject obj, jboolean b) \
{ \
	jlong gr = updateGlobalRef(env, obj, native##Event##Handler_fid, b); \
	CPhidget##pname##Handle h = (CPhidget##pname##Handle)(uintptr_t) \
	    (*env)->GetLongField(env, obj, handle_fid); \
	cphidgetSetHandlerFunc(h, b ? event##_handler : 0, \
	    (void *)(uintptr_t)gr); \
} \
\
static int CCONV \
event##_handler(CPhidget##pname##Handle h, void *arg, int index, type v) \
{ \
	JNIEnv *env; \
	jobject obj; \
	jobject event##Ev; \
\
	if ((*ph_vm)->AttachCurrentThread(ph_vm, (void **)&env, NULL)) \
	{ \
		LOG(PHIDGET_LOG_WARNING,"Couldn't AttachCurrentThread"); \
		(*env)->ExceptionDescribe(env); \
		(*env)->ExceptionClear(env); \
		abort(); \
	} \
\
	obj = (jobject)arg; \
\
	if (!(event##Ev = (*env)->NewObject(env, event##Event_class, \
	    event##Event_cons, obj, index, v))) \
		return -1; \
	(*env)->CallVoidMethod(env, obj, fire##Event##_mid, event##Ev); \
	(*env)->DeleteLocalRef(env, event##Ev); \
	(*ph_vm)->DetachCurrentThread(ph_vm); \
\
	return 0; \
}

#define EVENT_HANDLER_INDEXED2(pname, event, Event, cphidgetSetHandlerFunc, type, type2) \
	static int CCONV event##_handler(CPhidget##pname##Handle h, void *arg, \
	int, type, type2); \
\
JNIEXPORT void JNICALL \
Java_com_phidgets_##pname##Phidget_enable##Event##Events(JNIEnv *env, \
  jobject obj, jboolean b) \
{ \
	jlong gr = updateGlobalRef(env, obj, native##Event##Handler_fid, b); \
	CPhidget##pname##Handle h = (CPhidget##pname##Handle)(uintptr_t) \
	    (*env)->GetLongField(env, obj, handle_fid); \
	cphidgetSetHandlerFunc(h, b ? event##_handler : 0, \
	    (void *)(uintptr_t)gr); \
} \
\
static int CCONV \
event##_handler(CPhidget##pname##Handle h, void *arg, int index, type v, type2 w) \
{ \
	JNIEnv *env; \
	jobject obj; \
	jobject event##Ev; \
\
	if ((*ph_vm)->AttachCurrentThread(ph_vm, (void **)&env, NULL)) \
	{ \
		LOG(PHIDGET_LOG_WARNING,"Couldn't AttachCurrentThread"); \
		(*env)->ExceptionDescribe(env); \
		(*env)->ExceptionClear(env); \
		abort(); \
	} \
\
	obj = (jobject)arg; \
\
	if (!(event##Ev = (*env)->NewObject(env, event##Event_class, \
	    event##Event_cons, obj, index, v, w))) \
		return -1; \
	(*env)->CallVoidMethod(env, obj, fire##Event##_mid, event##Ev); \
	(*env)->DeleteLocalRef(env, event##Ev); \
	(*ph_vm)->DetachCurrentThread(ph_vm); \
\
	return 0; \
}


#endif

/*
	jlong gr = updateGlobalRef(env, obj, native##Event##Handler_fid, b); \
	CPhidget##pname##Handle h = (CPhidget##pname##Handle)(uintptr_t) \
	    (*env)->GetLongField(env, obj, handle_fid); \
\
	cphidgetSetHandlerFunc(h, b ? event##_handler : 0, \
	    (void *)(uintptr_t)gr); \
*/

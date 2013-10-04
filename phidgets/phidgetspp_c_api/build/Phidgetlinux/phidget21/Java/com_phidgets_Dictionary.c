#include "../stdafx.h"
#include <stdlib.h>
#include "com_phidgets_Dictionary.h"
#include "../cphidgetdictionary.h"
#include "../csocket.h"
#include "phidget_jni.h"

static int CCONV serverConnect_handler(CPhidgetDictionaryHandle h, void *arg);
static int CCONV serverDisconnect_handler(CPhidgetDictionaryHandle h, void *arg);

static jfieldID dictionary_handle_fid;
static jfieldID nativeServerConnectHandler_fid;
static jfieldID nativeServerDisconnectHandler_fid;
static jmethodID serverConnectEvent_cons;
static jmethodID serverDisconnectEvent_cons;
static jmethodID fireServerConnect_mid;
static jmethodID fireServerDisconnect_mid;
static jclass serverConnectEvent_class;
static jclass serverDisconnectEvent_class;
static jclass dictionary_class;

void
com_phidgets_Dictionary_OnLoad(JNIEnv *env)
{
	if (!(dictionary_class = (*env)->FindClass(env, "com/phidgets/Dictionary")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(dictionary_class = (jclass)(*env)->NewGlobalRef(env, dictionary_class)))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(dictionary_handle_fid = (*env)->GetFieldID(env, dictionary_class,
	  "handle", "J")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(serverConnectEvent_class = (*env)->FindClass(env, 
	  "com/phidgets/event/ServerConnectEvent")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(serverConnectEvent_class = (jclass)(*env)->NewGlobalRef(env, serverConnectEvent_class)))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(serverDisconnectEvent_class = (*env)->FindClass(env, 
	  "com/phidgets/event/ServerDisconnectEvent")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(serverDisconnectEvent_class = (jclass)(*env)->NewGlobalRef(env, serverDisconnectEvent_class)))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(fireServerConnect_mid = (*env)->GetMethodID(env, dictionary_class,
	  "fireServerConnect", "(Lcom/phidgets/event/ServerConnectEvent;)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(fireServerDisconnect_mid = (*env)->GetMethodID(env, dictionary_class,
	  "fireServerDisconnect", "(Lcom/phidgets/event/ServerDisconnectEvent;)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(fireServerConnect_mid = (*env)->GetMethodID(env, dictionary_class,
	  "fireServerConnect", "(Lcom/phidgets/event/ServerConnectEvent;)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(fireServerDisconnect_mid = (*env)->GetMethodID(env, dictionary_class,
	  "fireServerDisconnect", "(Lcom/phidgets/event/ServerDisconnectEvent;)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(serverConnectEvent_cons = (*env)->GetMethodID(env, serverConnectEvent_class,
	  "<init>", "(Ljava/lang/Object;)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(serverDisconnectEvent_cons = (*env)->GetMethodID(env, serverDisconnectEvent_class,
	  "<init>", "(Ljava/lang/Object;)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(nativeServerConnectHandler_fid = (*env)->GetFieldID(env, dictionary_class,
	  "nativeServerConnectHandler", "J")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(nativeServerDisconnectHandler_fid = (*env)->GetFieldID(env, dictionary_class,
	  "nativeServerDisconnectHandler", "J")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
}


JNIEXPORT void JNICALL
Java_com_phidgets_Dictionary_enableServerConnectEvents(JNIEnv *env, jobject obj,
  jboolean b)
{
	jlong gr = updateGlobalRef(env, obj, nativeServerConnectHandler_fid, b);
	CPhidgetDictionaryHandle h = (CPhidgetDictionaryHandle)(uintptr_t)(*env)->GetLongField(env,
	    obj, dictionary_handle_fid);

	CPhidgetDictionary_set_OnServerConnect_Handler(h, b ? serverConnect_handler : 0,
	    (void *)(uintptr_t)gr);
}

static int CCONV
serverConnect_handler(CPhidgetDictionaryHandle h, void *arg)
{
	JNIEnv *env;
	jobject obj;
	jobject serverConnectEvent;
	jint result;

	result = (*ph_vm)->GetEnv(ph_vm, (void **)&env, JNI_VERSION_1_4);
	
	if(result == JNI_EDETACHED)
	{
	if ((*ph_vm)->AttachCurrentThread(ph_vm, (void **)&env, NULL))
		abort();
	}

	obj = (jobject)arg;

	if (!(serverConnectEvent = (*env)->NewObject(env, serverConnectEvent_class, serverConnectEvent_cons,
	  obj)))
		return -1;
	(*env)->CallVoidMethod(env, obj, fireServerConnect_mid, serverConnectEvent);
	(*env)->DeleteLocalRef(env, serverConnectEvent);
	(*ph_vm)->DetachCurrentThread(ph_vm);

	return 0;
}

JNIEXPORT void JNICALL
Java_com_phidgets_Dictionary_enableServerDisconnectEvents(JNIEnv *env, jobject obj,
  jboolean b)
{
	jlong gr = updateGlobalRef(env, obj, nativeServerDisconnectHandler_fid, b);
	CPhidgetDictionaryHandle h = (CPhidgetDictionaryHandle)(uintptr_t)(*env)->GetLongField(env,
	    obj, dictionary_handle_fid);

	CPhidgetDictionary_set_OnServerDisconnect_Handler(h, b ? serverDisconnect_handler : 0,
	    (void *)(uintptr_t)gr);
}

static int CCONV
serverDisconnect_handler(CPhidgetDictionaryHandle h, void *arg)
{
	JNIEnv *env;
	jobject obj;
	jobject serverDisconnectEvent;
	jint result;

	result = (*ph_vm)->GetEnv(ph_vm, (void **)&env, JNI_VERSION_1_4);
	
	if(result == JNI_EDETACHED)
	{
	if ((*ph_vm)->AttachCurrentThread(ph_vm, (void **)&env, NULL))
		abort();
	}

	obj = (jobject)arg;

	if (!(serverDisconnectEvent = (*env)->NewObject(env, serverDisconnectEvent_class, serverDisconnectEvent_cons,
	  obj)))
		return -1;
	(*env)->CallVoidMethod(env, obj, fireServerDisconnect_mid, serverDisconnectEvent);
	(*env)->DeleteLocalRef(env, serverDisconnectEvent);
	(*ph_vm)->DetachCurrentThread(ph_vm);

	return 0;
}

JNIEXPORT jboolean JNICALL
Java_com_phidgets_Dictionary_isAttached(JNIEnv *env, jobject obj)
{
	CPhidgetHandle h = (CPhidgetHandle)(uintptr_t)(*env)->GetLongField(env,
	    obj, dictionary_handle_fid);
	int error;
	int s;
	
	if ((error = CPhidget_getDeviceStatus(h, &s)))
		PH_THROW(error);

	return s == PHIDGET_ATTACHED;
}

JNIEXPORT jboolean JNICALL
Java_com_phidgets_Dictionary_isAttachedToServer(JNIEnv *env, jobject obj)
{
	CPhidgetDictionaryHandle h = (CPhidgetDictionaryHandle)(uintptr_t)(*env)->GetLongField(env,
	    obj, dictionary_handle_fid);
	int error;
	int s;
	
	if ((error = CPhidgetDictionary_getServerStatus(h, &s)))
		PH_THROW(error);

	return s == PHIDGET_ATTACHED;
}

JNIEXPORT jint JNICALL
Java_com_phidgets_Dictionary_getServerPort(JNIEnv *env, jobject obj)
{
	CPhidgetDictionaryHandle h = (CPhidgetDictionaryHandle)(uintptr_t)(*env)->GetLongField(env,
	    obj, dictionary_handle_fid);
	int error;
	int port;
	const char *addr;

	if ((error = CPhidgetDictionary_getServerAddress(h, &addr, &port)))
		PH_THROW(error);

	return port;
}

JNIEXPORT jstring JNICALL
Java_com_phidgets_Dictionary_getServerAddress(JNIEnv *env, jobject obj)
{
	CPhidgetDictionaryHandle h = (CPhidgetDictionaryHandle)(uintptr_t)(*env)->GetLongField(env,
	    obj, dictionary_handle_fid);
	int error;
	int port;
	const char *addr;

	if ((error = CPhidgetDictionary_getServerAddress(h, &addr, &port)))
		PH_THROW(error);

	return (*env)->NewStringUTF(env, addr);
}

JNIEXPORT jstring JNICALL
Java_com_phidgets_Dictionary_getServerID(JNIEnv *env, jobject obj)
{
	CPhidgetDictionaryHandle h = (CPhidgetDictionaryHandle)(uintptr_t)(*env)->GetLongField(env,
	    obj, dictionary_handle_fid);
	int error;
	const char *id;

	if ((error = CPhidgetDictionary_getServerID(h, &id)))
		PH_THROW(error);

	return (*env)->NewStringUTF(env, id);
}

JNIEXPORT void JNICALL
Java_com_phidgets_Dictionary_nativeOpenRemote(JNIEnv *env, jobject obj, jstring serverID, jstring pass)
{
	int error;
    jboolean iscopy;
    const char *serverIDcopy = serverID?(*env)->GetStringUTFChars(
				env, serverID, &iscopy):0;
    const char *passcopy = (*env)->GetStringUTFChars(
                env, pass, &iscopy);

	CPhidgetDictionaryHandle h = (CPhidgetDictionaryHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, dictionary_handle_fid);

	if ((error = CPhidgetDictionary_openRemote(h, serverIDcopy, passcopy)))
		PH_THROW(error);
	
	if(serverID)
		(*env)->ReleaseStringUTFChars(env, serverID, serverIDcopy);
	(*env)->ReleaseStringUTFChars(env, pass, passcopy);
}

JNIEXPORT void JNICALL
Java_com_phidgets_Dictionary_nativeOpenRemoteIP(JNIEnv *env, jobject obj, jstring ipAddr, jint port, jstring pass)
{
	int error;
    jboolean iscopy;
    const char *ipAddrcopy = (*env)->GetStringUTFChars(
                env, ipAddr, &iscopy);
    const char *passcopy = (*env)->GetStringUTFChars(
                env, pass, &iscopy);

	CPhidgetDictionaryHandle h = (CPhidgetDictionaryHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, dictionary_handle_fid);

	if ((error = CPhidgetDictionary_openRemoteIP(h, ipAddrcopy, port, passcopy)))
		PH_THROW(error);

	(*env)->ReleaseStringUTFChars(env, ipAddr, ipAddrcopy);
	(*env)->ReleaseStringUTFChars(env, pass, passcopy);
}

JNIEXPORT void JNICALL
Java_com_phidgets_Dictionary_nativeDelete(JNIEnv *env, jobject obj)
{
	int error;
	
	CPhidgetDictionaryHandle h = (CPhidgetDictionaryHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, dictionary_handle_fid);

	if ((error = CPhidgetDictionary_delete(h)))
		PH_THROW(error);
}

JNIEXPORT void JNICALL
Java_com_phidgets_Dictionary_nativeClose(JNIEnv *env, jobject obj)
{
	int error;
	
	CPhidgetDictionaryHandle h = (CPhidgetDictionaryHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, dictionary_handle_fid);

	if ((error = CPhidgetDictionary_close(h)))
		PH_THROW(error);
}

JNIEXPORT void JNICALL 
Java_com_phidgets_Dictionary_nativeAddKey(JNIEnv *env, jobject obj, jstring key, jstring val, jint persistent)
{
	int error;
	
    jboolean iscopy;
    const char *keyString = (*env)->GetStringUTFChars(
                env, key, &iscopy);
    const char *valString = (*env)->GetStringUTFChars(
                env, val, &iscopy);

	CPhidgetDictionaryHandle h = (CPhidgetDictionaryHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, dictionary_handle_fid);

	if ((error = CPhidgetDictionary_addKey(h, keyString, valString, persistent)))
		PH_THROW(error);

	(*env)->ReleaseStringUTFChars(env, key, keyString);
	(*env)->ReleaseStringUTFChars(env, val, valString);
}

JNIEXPORT void JNICALL 
Java_com_phidgets_Dictionary_nativeRemoveKey(JNIEnv *env, jobject obj, jstring pattern)
{
	int error;
	
    jboolean iscopy;
    const char *patternString = (*env)->GetStringUTFChars(
                env, pattern, &iscopy);

	CPhidgetDictionaryHandle h = (CPhidgetDictionaryHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, dictionary_handle_fid);

	if ((error = CPhidgetDictionary_removeKey(h, patternString)))
		PH_THROW(error);

	(*env)->ReleaseStringUTFChars(env, pattern, patternString);
}

JNIEXPORT jstring JNICALL 
Java_com_phidgets_Dictionary_nativeGetKey(JNIEnv *env, jobject obj, jstring key)
{
	int error;

	char val[1024]; //TODO: unimpose this limit
	
    jboolean iscopy;
    const char *keyString = (*env)->GetStringUTFChars(
                env, key, &iscopy);

	CPhidgetDictionaryHandle h = (CPhidgetDictionaryHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, dictionary_handle_fid);

	if ((error = CPhidgetDictionary_getKey(h, keyString, val, sizeof(val))))
	{
		PH_THROW(error);
		(*env)->ReleaseStringUTFChars(env, key, keyString);
		return (*env)->NewStringUTF(env, "");
	}
	else
	{
		(*env)->ReleaseStringUTFChars(env, key, keyString);
		return (*env)->NewStringUTF(env, val);
	}
}

JNIEXPORT jlong JNICALL
Java_com_phidgets_Dictionary_create(JNIEnv *env, jobject obj)
{
	int error;
	CPhidgetDictionaryHandle dict;
	
	if ((error = CPhidgetDictionary_create(&dict)))
		PH_THROW(error);
	return (jlong)(uintptr_t)dict; 
}

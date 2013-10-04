#include "../stdafx.h"
#include <stdlib.h>
#include "com_phidgets_Dictionary.h"
#include "../cphidgetdictionary.h"
#include "../csocket.h"
#include "phidget_jni.h"

static int CCONV
key_handler(CPhidgetDictionaryHandle h, void *arg, const char *key, const char *val, CPhidgetDictionary_keyChangeReason reason);

static jfieldID dictionaryKeyListener_handle_fid;
static jfieldID nativeHandler_fid;
static jfieldID nativeListener_fid;
static jmethodID keyChangeEvent_cons; //constructor
static jmethodID keyRemovalEvent_cons;
static jmethodID fireKeyChange_mid;
static jmethodID fireKeyRemoval_mid;
static jclass dicitonaryKeyListener_class;
static jclass keyChangeEvent_class;
static jclass keyRemovalEvent_class;

void
com_phidgets_DictionaryKeyListener_OnLoad(JNIEnv *env)
{
	if (!(dicitonaryKeyListener_class = (*env)->FindClass(env, "com/phidgets/DictionaryKeyListener")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(dicitonaryKeyListener_class = (jclass)(*env)->NewGlobalRef(env, dicitonaryKeyListener_class)))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(keyChangeEvent_class = (*env)->FindClass(env, 
	  "com/phidgets/event/KeyChangeEvent")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(keyChangeEvent_class = (jclass)(*env)->NewGlobalRef(env, keyChangeEvent_class)))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(keyRemovalEvent_class = (*env)->FindClass(env, 
	  "com/phidgets/event/KeyRemovalEvent")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(keyRemovalEvent_class = (jclass)(*env)->NewGlobalRef(env, keyRemovalEvent_class)))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(fireKeyChange_mid = (*env)->GetMethodID(env, dicitonaryKeyListener_class,
	  "fireKeyChange", "(Lcom/phidgets/event/KeyChangeEvent;)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(fireKeyRemoval_mid = (*env)->GetMethodID(env, dicitonaryKeyListener_class,
	  "fireKeyRemoval", "(Lcom/phidgets/event/KeyRemovalEvent;)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(keyChangeEvent_cons = (*env)->GetMethodID(env, keyChangeEvent_class,
	  "<init>", "(Lcom/phidgets/Dictionary;Ljava/lang/String;Ljava/lang/String;)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(keyRemovalEvent_cons = (*env)->GetMethodID(env, keyRemovalEvent_class,
	  "<init>", "(Lcom/phidgets/Dictionary;Ljava/lang/String;Ljava/lang/String;)V")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(nativeHandler_fid = (*env)->GetFieldID(env, dicitonaryKeyListener_class,
	  "nativeHandler", "J")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(nativeListener_fid = (*env)->GetFieldID(env, dicitonaryKeyListener_class,
	  "listenerhandle", "J")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
	if (!(dictionaryKeyListener_handle_fid = (*env)->GetFieldID(env, dicitonaryKeyListener_class,
	  "handle", "J")))
	{
		(*env)->ExceptionDescribe(env);
		(*env)->ExceptionClear(env);
		abort();
	}
}

JNIEXPORT jlong JNICALL
Java_com_phidgets_DictionaryKeyListener_enableDictionaryKeyEvents(JNIEnv *env, jobject obj,
  jboolean b, jstring pattern)
{
	CPhidgetDictionaryListenerHandle keylistener;
	jlong gr = updateGlobalRef(env, obj, nativeHandler_fid, b);
	
    jboolean iscopy;
    const char *textString = (*env)->GetStringUTFChars(
                env, pattern, &iscopy);

	CPhidgetDictionaryHandle h = (CPhidgetDictionaryHandle)(uintptr_t)
	    (*env)->GetLongField(env, obj, dictionaryKeyListener_handle_fid);

	if(b)
	{
		CPhidgetDictionary_set_OnKeyChange_Handler(h, &keylistener, textString, b ? key_handler : 0,
			(void *)(uintptr_t)gr);
	}
	else
	{
		keylistener = (CPhidgetDictionaryListenerHandle)(uintptr_t)
			(*env)->GetLongField(env, obj, nativeListener_fid);
		CPhidgetDictionary_remove_OnKeyChange_Handler(keylistener);
		keylistener = NULL;
	}
	
	(*env)->ReleaseStringUTFChars(env, pattern, textString);

	return (jlong)(uintptr_t)keylistener;
}

static int CCONV
key_handler(CPhidgetDictionaryHandle h, void *arg, const char *key, const char *val, CPhidgetDictionary_keyChangeReason reason)
{
	JNIEnv *env;
	jobject obj;
	jobject keyEvent;
	jstring k;
	jstring v;

	if ((*ph_vm)->AttachCurrentThread(ph_vm, (void **)&env, NULL))
		abort();

	obj = (jobject)arg;

	if ((*ph_vm)->AttachCurrentThread (ph_vm, (void **) &env, ((void *) 0)))
		abort ();
	obj = (jobject) arg;

	k=(*env)->NewStringUTF(env, key);
	v=(*env)->NewStringUTF(env, val);

	switch(reason)
	{
		case PHIDGET_DICTIONARY_ENTRY_REMOVING:
		{
			if (!(keyEvent = (*env)->NewObject(env, keyRemovalEvent_class, keyRemovalEvent_cons,
			  obj, k, v)))
				return -1;
			(*env)->CallVoidMethod(env, obj, fireKeyRemoval_mid, keyEvent);
			break;
		}
		default:
		{
			if (!(keyEvent = (*env)->NewObject(env, keyChangeEvent_class, keyChangeEvent_cons,
			  obj, k, v)))
				return -1;
			(*env)->CallVoidMethod(env, obj, fireKeyChange_mid, keyEvent);
		}
	}

	(*env)->DeleteLocalRef(env, keyEvent);
	(*ph_vm)->DetachCurrentThread(ph_vm);

	return 0;
}

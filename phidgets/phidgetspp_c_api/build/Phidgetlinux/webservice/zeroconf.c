#include "stdafx.h"
#include "zeroconf.h"
#include "phidgetinterface.h"
#include "PhidgetWebservice21.h"
#include "dns_sd.h"

#ifdef ZEROCONF_RUNTIME_LINKING
//function prototypes for run-time loaded library
typedef DNSServiceErrorType (DNSSD_API * DNSServiceRegisterType) 
	(DNSServiceRef,DNSServiceFlags,uint32_t,const char *,
	 const char *,const char *,const char *,uint16_t,uint16_t,
	 const void *,DNSServiceRegisterReply,void *);
typedef DNSServiceErrorType (DNSSD_API * DNSServiceProcessResultType) (DNSServiceRef);
typedef void (DNSSD_API * DNSServiceRefDeallocateType) (DNSServiceRef);
typedef DNSServiceErrorType (DNSSD_API * DNSServiceAddRecordType)
	(DNSServiceRef, DNSRecordRef *, DNSServiceFlags, 
	 uint16_t, uint16_t, const void *, uint32_t);
typedef DNSServiceErrorType (DNSSD_API * DNSServiceUpdateRecordType)
	(DNSServiceRef, DNSRecordRef, DNSServiceFlags, 
	 uint16_t, const void *, uint32_t);
typedef DNSServiceErrorType (DNSSD_API * DNSServiceRemoveRecordType)
	(DNSServiceRef, DNSRecordRef, DNSServiceFlags);
#else
#define DNSServiceRegisterPtr DNSServiceRegister
#define DNSServiceProcessResultPtr DNSServiceProcessResult
#define DNSServiceRefDeallocatePtr DNSServiceRefDeallocate
#define DNSServiceAddRecordPtr DNSServiceAddRecord
#define DNSServiceUpdateRecordPtr DNSServiceUpdateRecord
#define DNSServiceRemoveRecordPtr DNSServiceRemoveRecord
#endif

int Dns_sdInitialized = FALSE; 

#ifdef ZEROCONF_RUNTIME_LINKING

#ifdef _WINDOWS
HMODULE dllHandle = NULL;
#elif _LINUX
void *libHandle = NULL;
#endif

//DNS_SD functions
DNSServiceRegisterType DNSServiceRegisterPtr = NULL;
DNSServiceProcessResultType DNSServiceProcessResultPtr = NULL;
DNSServiceRefDeallocateType DNSServiceRefDeallocatePtr = NULL;
DNSServiceAddRecordType DNSServiceAddRecordPtr = NULL;
DNSServiceUpdateRecordType DNSServiceUpdateRecordPtr = NULL;
DNSServiceRemoveRecordType DNSServiceRemoveRecordPtr = NULL;
#endif

DNSServiceRef zeroconf_dict_ref  = NULL;
pthread_t dns_thread = NULL;

extern const char *serverName;

int InitializeZeroconf();
int UninitializeZeroconf();
void DNSSD_API dict_reg_reply(DNSServiceRef client, DNSServiceFlags flags, DNSServiceErrorType errorCode,
							  const char *name, const char *regtype, const char *domain, void *context);
void DNSSD_API phid_reg_reply(DNSServiceRef client, DNSServiceFlags flags, DNSServiceErrorType errorCode,
							  const char *name, const char *regtype, const char *domain, void *context);

int zeroconf_advertise_ws()
{
	DNSServiceErrorType ret;
	InitializeZeroconf();
	if(Dns_sdInitialized)
	{
		char txt[2048];
		
		snprintf(txt, sizeof(txt), "%ctxtvers=%s%cprotocolvers=%s%cauth=%s", 
				 (unsigned short)(strlen("txtvers=") + strlen(dnssd_phidget_ws_txt_ver)), dnssd_phidget_ws_txt_ver,
				 (unsigned short)(strlen("protocolvers=") + strlen(protocol_ver)), protocol_ver,
				 (unsigned short)(strlen("auth=") + 1), (password==NULL?"n":"y"));
		
		if((ret = DNSServiceRegisterPtr(&zeroconf_dict_ref, 0, 0, serverName, "_phidget_ws._tcp", "", "", htons(port), strlen(txt), txt, dict_reg_reply, NULL)) == kDNSServiceErr_NoError)
		{
			DPRINT("zeroconf_advertise_ws success");
			if((ret = DNSServiceProcessResultPtr(zeroconf_dict_ref)) != kDNSServiceErr_NoError)
			{
				DPRINT("DNSServiceProcessResultPtr returned error: %d",ret);
				return EPHIDGET_UNEXPECTED;
			}
			pthread_create(&dns_thread, NULL, (void *(*)(void *))dns_callback_thread,zeroconf_dict_ref);
		}
		else
		{
			DPRINT("DNSServiceRegisterPtr returned error: %d",ret);
			return EPHIDGET_UNEXPECTED;
		}
	}
	return EPHIDGET_OK;
}

int zeroconf_unadvertise_phidget(CPhidgetHandle phid)
{
	if(Dns_sdInitialized)
	{
		DNSServiceRefDeallocatePtr(phid->dnsServiceRef);
	}
	return EPHIDGET_OK;
}

int zeroconf_advertise_phidget_thread(void * ref)
{
	CPhidgetHandle phid = (CPhidgetHandle)ref;
	char mdns_name[1024];
	DNSServiceErrorType ret;
	const unsigned char txt[2048];
	int txt_len = sizeof(txt);
	const char *name;
	
	createDNSTXTRecord(phid, txt, &txt_len);
	CPhidget_getDeviceName(phid, &name);
	snprintf(mdns_name, sizeof(mdns_name), "%s (%d)", name, phid->serialNumber);
	
	if((ret = DNSServiceRegisterPtr((DNSServiceRef *)&phid->dnsServiceRef, 0, 0, mdns_name, "_phidget._tcp", "", "", htons(port), txt_len, txt, phid_reg_reply, NULL)) == kDNSServiceErr_NoError)
	{
		DPRINT("zeroconf_advertise_phidget success");
		if((ret = DNSServiceProcessResultPtr(phid->dnsServiceRef)) != kDNSServiceErr_NoError)
		{
			DPRINT("DNSServiceProcessResultPtr returned error: %d",ret);
			return -1;
		}
	}
	else
	{
		DPRINT("DNSServiceRegisterPtr returned error: %d",ret);
		return -1;
	}
	return 0;
}

//start a thread to do this
int zeroconf_advertise_phidget(CPhidgetHandle phid)
{
	pthread_t thread = NULL;
	if(Dns_sdInitialized)
	{
		pthread_create(&thread, NULL, (void *(*)(void *))zeroconf_advertise_phidget_thread,phid);
	}
	return EPHIDGET_OK;
}

int dns_callback_thread(void * ref)
{
	DNSServiceErrorType ret;
    while ((ret = DNSServiceProcessResultPtr(ref)) == kDNSServiceErr_NoError)
        continue;
	DPRINT("dns_callback_thread exiting: %d",ret);
	return EPHIDGET_OK;
}

//TODO: implement this better!
int createDNSTXTRecord(CPhidgetHandle phid, const unsigned char *txt_buf, int *len)
{
	int i;
	const char *name;
	char *txt[11];
	
	char txt_ver[20], txt_name[128], txt_type[50], txt_serial[20], txt_server_id[1024], txt_label[50];
	char txt_txtver[20], txt_protocolver[25], txt_auth[10], txt_id[10], txt_class[13];
	
	unsigned char *ptr = (unsigned char *)txt_buf;
	
	txt[0] = txt_txtver;
	txt[1] = txt_protocolver;
	txt[2] = txt_auth;
	txt[3] = txt_server_id;
	txt[4] = txt_name;
	txt[5] = txt_type;
	txt[6] = txt_serial;
	txt[7] = txt_ver;
	txt[8] = txt_label;
	txt[9] = txt_id;
	txt[10] = txt_class;
	
	CPhidget_getDeviceName(phid, &name);
	snprintf(txt_txtver, sizeof(txt_txtver), "txtvers=%s", dnssd_phidget_txt_ver);
	snprintf(txt_protocolver, sizeof(txt_protocolver), "protocolvers=%s", protocol_ver);
	snprintf(txt_auth, sizeof(txt_auth), "auth=%s", (password==NULL?"n":"y"));
	snprintf(txt_server_id, sizeof(txt_server_id), "server_id=%s", serverName);
	snprintf(txt_name, sizeof(txt_name), "name=%s", name);
	snprintf(txt_type, sizeof(txt_type), "type=%s", phid->deviceType);
	snprintf(txt_serial, sizeof(txt_serial), "serial=%d", phid->serialNumber);
	snprintf(txt_ver, sizeof(txt_ver), "version=%d", phid->deviceVersion);
	snprintf(txt_label, sizeof(txt_label), "label=%s", phid->label);
	snprintf(txt_id, sizeof(txt_id), "id=%d", phid->deviceIDSpec);
	snprintf(txt_class, sizeof(txt_class), "class=%d", phid->deviceID);
	
	for (i = 0; i < 11; i++)
	{
		unsigned char *len = ptr++;
		*len = ( unsigned char ) strlen(txt[i]);
		strcpy((char*)ptr, txt[i]);
		ptr += *len;
	}
	
	*len = (ptr-txt_buf);
	
	return EPHIDGET_OK;
}

int updateDNSTXTRecords()
{
	CPhidgetHandle phid = NULL;
	CPhidgetHandle *phidArray;
	int count, i;
	DNSServiceErrorType err;
	
	if(!phidm) return EPHIDGET_OK;
	
	CPhidgetManager_getAttachedDevices(phidm, &phidArray, &count);
	
	for (i=0;i<count;i++)
	{
		phid = phidArray[i];
		if(Dns_sdInitialized)
		{	
			const unsigned char txt[2048];
			int txt_len = sizeof(txt);
			createDNSTXTRecord(phid, txt, &txt_len);
			err = DNSServiceUpdateRecordPtr((DNSServiceRef)phid->dnsServiceRef, NULL, 0, (uint16_t) txt_len, txt, 0);
		}
	}
	
	CPhidgetManager_freeAttachedDevicesArray(phidArray);

	return EPHIDGET_OK;
}

void DNSSD_API dict_reg_reply(DNSServiceRef client, DNSServiceFlags flags, DNSServiceErrorType errorCode,
							  const char *name, const char *regtype, const char *domain, void *context)
{
	DPRINT("Got a reply for %s.%s%s: ", name, regtype, domain);
	switch (errorCode)
	{
		case kDNSServiceErr_NoError:      
			DPRINT("  Name now registered and active"); 
			break;
		case kDNSServiceErr_NameConflict: 
			DPRINT("  Name in use, please choose another"); 
			exit(-1);
		default:                          
			DPRINT("  Error %d", errorCode); 
			return;
	}
	strcpy((char *)serverName, name);
	
	updateDNSTXTRecords();
}

void DNSSD_API phid_reg_reply(DNSServiceRef client, DNSServiceFlags flags, DNSServiceErrorType errorCode,
							  const char *name, const char *regtype, const char *domain, void *context)
{
	DPRINT("Got a reply for %s.%s%s: ", name, regtype, domain);
	switch (errorCode)
	{
		case kDNSServiceErr_NoError:      
			DPRINT("  Name now registered and active"); 
			break;
		case kDNSServiceErr_NameConflict: 
			DPRINT("  Name in use, please choose another"); 
			exit(-1);
		default:                          
			DPRINT("  Error %d", errorCode); 
			return;
	}
}

int InitializeZeroconf()
{
#ifdef ZEROCONF_RUNTIME_LINKING
	
#ifdef _WINDOWS
	if(!(dllHandle = LoadLibrary(L"dnssd.dll")))
	{
		DWORD error = GetLastError();
		switch(error)
		{
			case ERROR_MOD_NOT_FOUND:
				DPRINT("LoadLibrary failed - module could not be found");
				break;
			default:
				DPRINT("LoadLibrary failed with error code: %d", error);
		}
		return EPHIDGET_UNEXPECTED;
	}
	
	// If the handle is valid, try to get the function address. 
	if (NULL != dllHandle) 
	{ 
		//Get pointers to our functions using GetProcAddress:
		DNSServiceRegisterPtr = (DNSServiceRegisterType)GetProcAddress(dllHandle, "DNSServiceRegister");
		DNSServiceProcessResultPtr = (DNSServiceProcessResultType)GetProcAddress(dllHandle, "DNSServiceProcessResult");
		DNSServiceRefDeallocatePtr = (DNSServiceRefDeallocateType)GetProcAddress(dllHandle, "DNSServiceRefDeallocate");
		DNSServiceAddRecordPtr = (DNSServiceAddRecordType)GetProcAddress(dllHandle, "DNSServiceAddRecord");
		DNSServiceUpdateRecordPtr = (DNSServiceUpdateRecordType)GetProcAddress(dllHandle, "DNSServiceUpdateRecord");
		DNSServiceRemoveRecordPtr = (DNSServiceRemoveRecordType)GetProcAddress(dllHandle, "DNSServiceRemoveRecord");
		
		Dns_sdInitialized = (
							 NULL != DNSServiceRegisterPtr && 
							 NULL != DNSServiceProcessResultPtr &&
							 NULL != DNSServiceRefDeallocatePtr &&
							 NULL != DNSServiceAddRecordPtr &&
							 NULL != DNSServiceUpdateRecordPtr &&
							 NULL != DNSServiceRemoveRecordPtr);
	}
	
	if(!Dns_sdInitialized)
	{
		DPRINT("InitializeZeroconf failed somehow...");
		return EPHIDGET_UNEXPECTED;
	}
	
#elif _LINUX
	libHandle = dlopen("libdns_sd.so",RTLD_LAZY);
	if(!libHandle)
	{
		DPRINT("dlopen failed with error: %s", dlerror());
		DPRINT("Assuming that zeroconf is not supported on this machine.");
		return EPHIDGET_UNSUPPORTED;
	}
	
	//Get pointers to our functions using dlsym:
	if(!(DNSServiceRegisterPtr = (DNSServiceRegisterType)dlsym(libHandle, "DNSServiceRegister"))) goto dlsym_err;
	if(!(DNSServiceProcessResultPtr = (DNSServiceProcessResultType)dlsym(libHandle, "DNSServiceProcessResult"))) goto dlsym_err;
	if(!(DNSServiceRefDeallocatePtr = (DNSServiceRefDeallocateType)dlsym(libHandle, "DNSServiceRefDeallocate"))) goto dlsym_err;
	if(!(DNSServiceAddRecordPtr = (DNSServiceAddRecordType)dlsym(libHandle, "DNSServiceAddRecord"))) goto dlsym_err;
	if(!(DNSServiceUpdateRecordPtr = (DNSServiceUpdateRecordType)dlsym(libHandle, "DNSServiceUpdateRecord"))) goto dlsym_err;
	if(!(DNSServiceRemoveRecordPtr = (DNSServiceRemoveRecordType)dlsym(libHandle, "DNSServiceRemoveRecord"))) goto dlsym_err;
	
	goto dlsym_good;
	
dlsym_err:
	DPRINT("dlsym failed with error: %s", dlerror());
	DPRINT("Assuming that zeroconf is not supported on this machine.");
	return EPHIDGET_UNSUPPORTED;
	
dlsym_good:
	Dns_sdInitialized = TRUE;
#endif
	
	
	DPRINT("InitializeZeroconf Seems good...");
#else
	Dns_sdInitialized = TRUE;
#endif
	
	return EPHIDGET_OK;
}

int UninitializeZeroconf()
{
	void *status;
	
	if(Dns_sdInitialized)
	{
		
		if(zeroconf_dict_ref)
		{
			DNSServiceRefDeallocatePtr(zeroconf_dict_ref);
			zeroconf_dict_ref = NULL;
		}
		
		if(dns_thread)
		{
			pthread_join(dns_thread, &status);
			dns_thread = NULL;
		}
		
#ifdef ZEROCONF_RUNTIME_LINKING
#ifdef _WINDOWS
		//Free the library:
		FreeLibrary(dllHandle); 
#elif _LINUX
		dlclose(libHandle);
#endif
#endif
		Dns_sdInitialized = FALSE;
	}
	return EPHIDGET_OK;
}

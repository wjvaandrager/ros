/*
 *  PhidgetWebservice21.h
 *  PhidgetWebservice21
 *
 *  Created by Allanon on 27/09/06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef __PHIDGETWEBSERVICE21
#define __PHIDGETWEBSERVICE21



extern pthread_mutex_t pd_mutex;
 void
pd_lock(void *pd_lock);
 void
pd_unlock(void *pd_lock);

int add_key(pds_session_t *pdss, const char *key, const char *val);
int remove_key(pds_session_t *pdss, const char *key);
int add_listener(pds_session_t *pdss, const char *kpat, pdl_notify_func_t notfiy, void *ptr);

extern int port;
extern const char *serverName;
extern const char *password;
extern const char *protocol_ver;

#endif

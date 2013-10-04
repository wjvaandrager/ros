#ifndef _PDICTSERVER_H_
#define _PDICTSERVER_H_

#include "pdict.h"
#include "utils.h"
#include "ptree.h"
#include "plist.h"

typedef struct pds_session pds_session_t;

typedef struct {
	plist_node_t *wa_l;
	regex_t wa_regex;
	pds_session_t *wa_pdss;
	const char *wa_cmdtag;
} wa_t;

typedef struct {
	const char *ipm_id;
	const char *ipm_key;
	const char *ipm_val;
	pdict_reason_t ipm_reason;
} id_pde_map_t;

struct pds_session {
	int pdss_id;
	char pdss_errdesc[2048];
	int pdss_rfd;
	int pdss_wfd;
	int (*pdss_read)(int, void *, unsigned int, char *errdesc, int edlen);
	int (*pdss_write)(int, const void *, unsigned int, char *errdesc,
	    int edlen);
	int (*pdss_close)(int, char *errdesc, int edlen);
	void (*pdss_log)(pu_log_level_t, int s, const char *entry);
	void (*pdss_cleanup)(void);
	ptree_node_t *pdss_pending; /* protected by pd_lock */
	ptree_node_t *pdss_notify_args;
	pthread_mutex_t pdss_lock;
	int pdss_report_period;
	char *pdss_report_cmdtag;
	pthread_t pdss_report_thread;
	char pdss_readbuf[2048];
	int pdss_bufcur;
	int pdss_buflen;
	unsigned int pdss_nreport;
	pthread_cond_t pdss_report_cv; /* report done */
	pdict_t *pdss_pd;
	/*
	 * caller's dict lock must be held around dictionary
	 * modifications.  It implicitly protects pdss_pending.
	 */
	void (*pdss_pd_lock)(void *);
	void (*pdss_pd_unlock)(void *);
	void *pdss_pd_lock_arg;
	int pdss_should_close;
	plist_node_t *pdss_expire; /* list of keys to expire at session end */
	int pdss_client_cr_null; /* client needs NULL in addition to CR */
};

int pds_init(void);
int pds_session_serve(const pds_session_t *pdss);
const pds_session_t *pds_session_alloc(pdict_t *pd, void (*pd_lock)(void *),
    void (*pd_unlock)(void *), void *pd_lock_arg, int readfd,
    int (*readfunc)(int, void *, unsigned int, char *errdesc, int edlen),
    int writefd, int (*writefunc)(int, const void *, unsigned int,
    char *errdesc, int edlen), int (*closefunc)(int, char *errdesc, int edlen),
    void (*logfunc)(pu_log_level_t, int s, const char *), 
	void(*cleanupfunc)(void), char *errdesc,
    int edlen);
void pds_session_free(pds_session_t *pdss);
int pds_session_id(const pds_session_t *pdss);
void pds_process_line(pds_session_t *pdss, char *line);
void pds_session_flush(pds_session_t *pdss);

#endif

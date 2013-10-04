// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#ifndef __STDAFX
#define __STDAFX

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <regex.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <regex.h>
#include <ctype.h>
#ifndef WINCE
#include <signal.h>
#endif
#include <time.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <regex.h>
#include <assert.h>
#if !defined(_WINDOWS) || defined(__CYGWIN__)
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#else
#include "wincompat.h"
#endif
#if defined(__CYGWIN__) || defined(__MINGW32_VERSION)
#include "getaddrinfo.h"
#endif
#ifndef _MSC_EXTENSIONS
#include <unistd.h>
#endif

#ifdef WINCE
int errno;
#define strerror(err) "strerror() Not Supported in Windows CE - error: " #err
#define abort() exit(1)
#define EAGAIN 35
#else
#include <sys/types.h>
#include <errno.h>
#endif

#ifndef _WINDOWS
#define SET_ERRDESC(errdesc, errdesclen) (errdesc ? snprintf(errdesc, \
    errdesclen, "%s", strerror(errno)) : 0);
#else
#define SET_ERRDESC(errdesc, errdesclen) wsa_set_errdesc(errdesc, errdesclen);
#endif

extern int print_debug_messages;
#ifdef DEBUG /*defined in the command line*/
#ifndef _MSC_EXTENSIONS
#define DPRINT(...) ( printf("%s(%d): ", __func__, __LINE__), \
    printf(__VA_ARGS__), printf("\n"), fflush(stdout), (void)0 )
#else
#define DPRINT(...) ( printf("%s+%d: ", __FILE__, __LINE__), \
    printf(__VA_ARGS__), printf("\n"), fflush(stdout), (void)0 )
#endif
#else
#ifndef _MSC_EXTENSIONS
#define DPRINT(...) \
if(print_debug_messages) \
{ \
	/*printf("%s(%d): ", __func__, __LINE__);*/ \
    printf(__VA_ARGS__); printf("\n"); fflush(stdout); \
}
#else
#define DPRINT(...) \
if(print_debug_messages) \
{ \
	/*printf("%s(%d): ", __FILE__, __LINE__);*/ \
    printf(__VA_ARGS__); printf("\n"); fflush(stdout); \
}
#endif
#endif



#define WARN(...) (fprintf(stderr, "PHIDGETWEBSERVICE: " __VA_ARGS__), fprintf(stderr, \
    "\n"), fflush(stderr), (void)0)

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#define MAX_KEY_SIZE 1024
#define MAX_VAL_SIZE 1024

#ifdef _WINDOWS
// Defines & Include for Windows only

	//use zeroconf
	#ifndef USE_ZEROCONF
	#define USE_ZEROCONF
	#endif
	//use runtime linking for zeroconf
	#ifndef ZEROCONF_RUNTIME_LINKING
	#define ZEROCONF_RUNTIME_LINKING
	#endif
	

	// Modify the following defines if you have to target a platform prior to the ones specified below.
	// Refer to MSDN for the latest info on corresponding values for different platforms.
	#ifndef WINVER				// Allow use of features specific to Windows XP or later.
	#define WINVER 0x0501		// Change this to the appropriate value to target other versions of Windows.
	#endif

	#ifndef _WIN32_WINNT		// Allow use of features specific to Windows XP or later.                   
	#define _WIN32_WINNT 0x0501	// Change this to the appropriate value to target other versions of Windows.
	#endif						

	#ifndef _WIN32_WINDOWS		// Allow use of features specific to Windows 98 or later.
	#define _WIN32_WINDOWS 0x0410 // Change this to the appropriate value to target Windows Me or later.
	#endif

	#ifndef _WIN32_IE			// Allow use of features specific to IE 6.0 or later.
	#define _WIN32_IE 0x0600	// Change this to the appropriate value to target other versions of IE.
	#endif

	#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
	// Windows Header Files:
	#include <winsock2.h>
	#include <ws2tcpip.h>
	#ifndef WINCE
		#include <Wspiapi.h>
		#include <process.h>
	#endif
	#include <windows.h>
	#include <winbase.h>
	#include <stdio.h>
	#include <stdlib.h>
	#include <time.h>
	#include <assert.h>
	#include "wincompat.h"
	#include <windows.h>
	#include <stdio.h>
	#include <stdlib.h>


	//#include <vld.h>
	/*#define _CRTDBG_MAP_ALLOC
	#include <stdlib.h>
	#include <crtdbg.h>*/

	#define strdup _strdup

	//#define CCONV __cdecl
	#define CCONV __stdcall
	#define SLEEP(dlay) Sleep(dlay);
	#define ZEROMEM(var,size) ZeroMemory(var, size);
	
	typedef SYSTEMTIME TIME;
	
	typedef	int	pid_t;
	#define	getpid	_getpid
	#define	strcasecmp	_stricmp
	//#define snprintf _snprintf
	
#else
// Defines & Include for both Mac and Linux
	#include <semaphore.h>
	#include <time.h>
	#include <stdio.h>
	#include <stdlib.h>
	#include <string.h>
	#include <pthread.h>
	#include <errno.h>
	#include <sys/types.h>
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <netdb.h>
	#include <unistd.h>
	#include <sys/time.h>
	#include <signal.h>
	
	typedef struct timeval TIME;
	typedef void *HANDLE;
	#define SLEEP(dlay) usleep(dlay*1000);
	#define ZEROMEM(var,size) memset(var, 0, size);
	#define CCONV
	typedef int SOCKET;
	#define INVALID_SOCKET -1;

	#ifndef USE_ZEROCONF
	#define USE_ZEROCONF
	#endif
	
	#ifdef _LINUX
		//use runtime linking for zeroconf
		#ifndef ZEROCONF_RUNTIME_LINKING
		#define ZEROCONF_RUNTIME_LINKING
		#include <dlfcn.h>
		#endif
	#endif
	
	#ifdef _MACOSX
		#include <CoreFoundation/CoreFoundation.h>
	#endif
	
#endif

#ifdef DEBUG /*defined in the command line*/

/*void * phi_malloc(size_t size, const char *file, int line);
void phi_free(void *ptr, const char *file, int line);
#define malloc(size) phi_malloc(size,__FILE__,__LINE__)
#define free(ptr) phi_free(ptr,__FILE__,__LINE__)*/
#endif
	

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

char * get_policy_file();

#ifndef _LINUX
#include "../../phidget21/phidget21/phidget21int.h"
#else
#include <phidget21internal.h>
#endif

#ifdef DMALLOC
#include "dmalloc.h"
#endif

#endif

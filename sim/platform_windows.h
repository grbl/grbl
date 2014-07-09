#ifdef platform_h
#error "platform implementation already defined"
#else

#include <windows.h>


typedef struct {
  HANDLE tid;
  int exit;
} plat_thread_t;

typedef DWORD WINAPI(*plat_threadfunc_t)(LPVOID);
#define PLAT_THREAD_FUNC(name,arg) DWORD WINAPI name(LPVOID arg)

#endif

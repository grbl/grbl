#ifdef platform_h
#error "platform implementation already defined"
#else

#include <pthread.h>


typedef struct {
  pthread_t tid;
  int exit;
} plat_thread_t;

typedef void*(*plat_threadfunc_t)(void*);
#define PLAT_THREAD_FUNC(name,arg) void* name(void* arg)

#endif

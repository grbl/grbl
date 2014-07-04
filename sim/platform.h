#ifndef platform_h

#ifdef  PLAT_LINUX
#include "platform_linux.h"
#else
#include "platform_windows.h"
#endif

#define platform_h

void platform_init();
void platform_terminate();

plat_thread_t* platform_start_thread(plat_threadfunc_t func);
void platform_stop_thread(plat_thread_t* thread);
void platform_kill_thread(plat_thread_t* thread);

uint32_t  platform_ns();  //monotonically increasing nanoseconds since program start.
void platform_sleep(long microsec); //sleep for suggested time in microsec.

uint8_t platform_poll_stdin(); //non-blocking stdin read - returns 0 if no char present, 0xFF for EOF

#endif

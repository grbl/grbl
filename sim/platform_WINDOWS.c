#include <stdio.h>
#include <stdlib.h>

#include <conio.h>
#include <windows.h>
#include "platform.h"

#define NS_PER_SEC 1000000000
#define MICRO_PER_MILLI 1000

double ns_per_perfcount;

//any platform-specific setup that must be done before sim starts here
void platform_init()
{
  __int64 counts_per_sec;
  QueryPerformanceFrequency((LARGE_INTEGER*)&counts_per_sec);
  ns_per_perfcount = (float)NS_PER_SEC / counts_per_sec;
}

//cleanup int here;
void platform_terminate()
{
}

//returns a free-running 32 bit nanosecond counter which rolls over
uint32_t platform_ns() 
{
  static uint32_t gTimeBase = 0;
  __int64 counts;
  uint32_t ns;
  QueryPerformanceCounter((LARGE_INTEGER*)&counts);
  ns = (counts*ns_per_perfcount);

  if (gTimeBase== 0){gTimeBase=ns;}
  return ns-gTimeBase;
}

//sleep in microseconds
void platform_sleep(long  microsec)
{
  Sleep(microsec/MICRO_PER_MILLI);
}

  
//create a thread
plat_thread_t* platform_start_thread(plat_threadfunc_t threadfunc) {
  plat_thread_t* th = malloc(sizeof(plat_thread_t));
  th->tid = CreateThread(NULL,0,threadfunc,&th->exit,0,NULL);
  if (!th->tid){
	 free(th);
	 return NULL;
  }
  return th;
}

//ask thread to exit nicely, wait
void platform_stop_thread(plat_thread_t* th){
  th->exit = 1;
  WaitForSingleObject(th->tid,INFINITE);
}

//force-kill thread
void platform_kill_thread(plat_thread_t* th){
  th->exit = 1;
  TerminateThread(th->tid, 0);
}

//return char if one available.
uint8_t platform_poll_stdin() {
  if (_kbhit()) {
     return getch();
  }
  return 0;
}

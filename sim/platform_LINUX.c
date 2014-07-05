#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include "platform.h"

#define MS_PER_SEC 1000000


//any platform-specific setup that must be done before sim starts here
void platform_init()
{
}

//cleanup int here;
void platform_terminate()
{
}

//returns a free-running 32 bit nanosecond counter which rolls over
uint32_t platform_ns() 
{
  static uint32_t gTimeBase = 0;
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC,&ts);
  if (gTimeBase== 0){gTimeBase=ts.tv_nsec;}
  return ts.tv_nsec-gTimeBase;
}

//sleep in microseconds
void platform_sleep(long  microsec)
{
  struct timespec ts={0};
  while (microsec >= MS_PER_SEC){
	 ts.tv_sec++;
	 microsec-=MS_PER_SEC;
  }
  ts.tv_nsec = microsec*1000;
  nanosleep(&ts,NULL);
}

#define SIM_ECHO_TERMINAL 1 //use this to make grbl_sim act like a serial terminal with local echo on.

//set terminal to allow kbhit detection
void enable_kbhit(int dir)
{
  static struct termios oldt, newt;
 
  if ( dir == 1 )
  {
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
	 newt.c_lflag &= ~( ICANON  ); 
	 if (!SIM_ECHO_TERMINAL) {
		newt.c_lflag &= ~( ECHO );
	 }
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  }
  else
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}
 
//detect key pressed
int kbhit (void)
{
  struct timeval tv={0};
  fd_set rdfs={{0}};
  int retval;

  /* tv.tv_sec = 0; */
  /* tv.tv_usec = 0; */
 
  /* FD_ZERO(&rdfs); */
  FD_SET (STDIN_FILENO, &rdfs);
 
  select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
  retval = FD_ISSET(STDIN_FILENO, &rdfs);

  return retval;
}



plat_thread_t* platform_start_thread(plat_threadfunc_t threadfunc) {
  plat_thread_t* th = malloc(sizeof(plat_thread_t));
  if (pthread_create(&th->tid, NULL, threadfunc, &th->exit)){
	 free(th);
	 return NULL;
  }
  return th;
}

//ask thread to exit nicely, wait
void platform_stop_thread(plat_thread_t* th){
  th->exit = 1;
  pthread_join(th->tid,NULL);  
}

//force-kill thread
void platform_kill_thread(plat_thread_t* th){
  th->exit = 1;
  pthread_cancel(th->tid); 
}

//return char if one available.
uint8_t platform_poll_stdin() {
  uint8_t char_in=0;
  enable_kbhit(1);
  if ( kbhit()) {
	 char_in = getchar();
  }
  enable_kbhit(0);
  return char_in;
}

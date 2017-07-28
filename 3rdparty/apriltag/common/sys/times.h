#ifndef _TIMES_H
#define _TIMES_H

#ifdef _WIN32
#include <sys/timeb.h>
#include <sys/types.h>
#include <winsock2.h>
#include <Windows.h>
#include <stdint.h> // portable: uint64_t   MSVC: __int64 

int gettimeofday(struct timeval* t, void* timezone);

// from linux's sys/times.h

//#include <features.h>

#define __need_clock_t
#include <time.h>


/* Structure describing CPU time used by a process and its children.  */
struct tms
  {
    clock_t tms_utime;          /* User CPU time.  */
    clock_t tms_stime;          /* System CPU time.  */

    clock_t tms_cutime;         /* User CPU time of dead children.  */
    clock_t tms_cstime;         /* System CPU time of dead children.  */
  };

/* Store the CPU time used by this process and all its
   dead children (and their dead children) in BUFFER.
   Return the elapsed real time, or (clock_t) -1 for errors.
   All times are in CLK_TCKths of a second.  */
clock_t times (struct tms *__buffer);

typedef long long suseconds_t ;


////https://stackoverflow.com/a/26085827/6055233
//// MSVC defines this in winsock2.h!?
////typedef struct timeval {
////  long tv_sec;
////  long tv_usec;
////} timeval;
//
//int gettimeofday(struct timeval * tp, struct timezone * tzp)
//{
//  // Note: some broken versions only have 8 trailing zero's, the correct epoch has 9 trailing zero's
//  // This magic number is the number of 100 nanosecond intervals since January 1, 1601 (UTC)
//  // until 00:00:00 January 1, 1970 
//  static const uint64_t EPOCH = ((uint64_t)116444736000000000ULL);
//
//  SYSTEMTIME  system_time;
//  FILETIME    file_time;
//  uint64_t    time;
//
//  GetSystemTime(&system_time);
//  SystemTimeToFileTime(&system_time, &file_time);
//  time = ((uint64_t)file_time.dwLowDateTime);
//  time += ((uint64_t)file_time.dwHighDateTime) << 32;
//
//  tp->tv_sec = (long)((time - EPOCH) / 10000000L);
//  tp->tv_usec = (long)(system_time.wMilliseconds * 1000);
//  return 0;
//}

#endif
#endif
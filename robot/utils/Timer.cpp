//
// Created by jayen on 19/03/19.
//

#include "Timer.hpp"
#include <cerrno>
#include <cstring>
#include <iostream>
#include "Logger.hpp"

using namespace std;

Timer::Timer() {
   // _start_time = clock();
   gettimeofday(&timeStamp, NULL);
}  // postcondition: elapsed()==0
void Timer::restart() {
   // _start_time = clock();
   gettimeofday(&timeStamp, NULL);
}  // post: elapsed()==0

double Timer::sec(const timeval &t) {
   return t.tv_sec + t.tv_usec / 1000000.;
}

double Timer::msec(const timeval &t) {
   return t.tv_sec * 1000ull + t.tv_usec / 1000.;
}

// need to return double or unsigned long long for accuracy
double Timer::usec(const timeval &t) {
   return t.tv_sec * 1000000ull + t.tv_usec;
}

/* return elapsed time in seconds */
double Timer::elapsed() {
   timeval tmp;
   gettimeofday(&tmp, NULL);
   return sec(tmp) -
          sec(timeStamp);
}

uint32_t Timer::elapsed_ms() {
   timeval tmp;
   gettimeofday(&tmp, NULL);
   return msec(tmp) - msec(timeStamp);
}

uint32_t Timer::elapsed_us() {
   timeval tmp;
   int     success = gettimeofday(&tmp, NULL);
   // on v6, tmp=-1 sometimes
   // https://trello.com/c/amYdAjpH/189-freeze-then-limp-issue
   if (success == 0) {
      double elapsed_us = usec(tmp) - usec(timeStamp);
      if (elapsed_us < 0) {
         llog(ERROR) << "elapsed_us is negative: " << elapsed_us << endl;
         // arbitrary number so we know this has happened
         return 5000000;
      }
      return elapsed_us;
   } else {
      llog(ERROR) << "elapsed_us-gettimeofday: " << strerror(errno) << endl;
      // arbitrary number so we know this has happened
      return 5000000;
   }
}

/* return estimated maximum value for elapsed() */
double Timer::elapsed_max() {
   /* Portability warning: elapsed_max() may return too high a value on systems
    * where clock_t overflows or resets at surprising values.
    */
   return elapsed();
}

/* return minimum value for elapsed() */
double Timer::elapsed_min() {
   return elapsed();
}

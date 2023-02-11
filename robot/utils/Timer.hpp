/* A timer class
 **/

#pragma once

#include <sys/time.h>
#include <stdint.h>
#include <time.h>


//  timer  -------------------------------------------------------------------//

//  A timer object measures elapsed time.

//  It is recommended that implementations measure wall clock rather than CPU
//  time since the intended use is performance measurement on systems where
//  total elapsed time is more important than just process or CPU time.

//  Warnings: The maximum measurable elapsed time may well be only 596.5+ hours
//  due to implementation limitations.  The accuracy of timings depends on the
//  accuracy of timing information provided by the underlying platform, and
//  this varies a great deal from platform to platform.

class Timer {
   public:
      Timer();

      void restart();

      double sec(const timeval &t);

      double msec(const timeval &t);

      double usec(const timeval &t);

      /* return elapsed time in seconds */
      double elapsed();

      uint32_t elapsed_ms();

      uint32_t elapsed_us();

      /* return estimated maximum value for elapsed() */
      double elapsed_max();

      /* return minimum value for elapsed() */
      double elapsed_min();

   private:
      timeval timeStamp;
};


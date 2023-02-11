//
// Created by jayen on 19/03/19.
//

// https://trello.com/c/amYdAjpH/189-freeze-then-limp-issue
// code from Timer.cpp:
//
//float Timer::usec(const timeval &t) {
//   return t.tv_sec * 1000000 + t.tv_usec;
//}
//
//uint32_t Timer::elapsed_us() {
//   timeval tmp;
//   int     success = gettimeofday(&tmp, NULL);
//   // on v6, tmp=-1 sometimes
//   // https://trello.com/c/amYdAjpH/189-freeze-then-limp-issue
//   if (success == 0) {
//      float elapsed_us = usec(tmp) - usec(timeStamp);
//      if (elapsed_us < 0) {
//         cerr << __FILE__ << "\t" << __LINE__ << "\telapsed_us:\t" << elapsed_us << "\n";
//         cerr << __FILE__ << "\t" << __LINE__ << "\ttmp.tv_sec:\t" << tmp.tv_sec << "\n";
//         cerr << __FILE__ << "\t" << __LINE__ << "\ttmp.tv_usec:\t" << tmp.tv_usec << "\n";
//         cerr << __FILE__ << "\t" << __LINE__ << "\ttimeStamp.tv_sec:\t" << timeStamp.tv_sec << "\n";
//         cerr << __FILE__ << "\t" << __LINE__ << "\ttimeStamp.tv_usec:\t" << timeStamp.tv_usec << "\n";
//         cerr << __FILE__ << "\t" << __LINE__ << "\tstrerror(errno):\t" << strerror(errno) << "\n";
//         // arbitrary number so we know this has happened
//         return 5000000;
//      }
//      return elapsed_us;
//   } else {
//      cerr << __FILE__ << "\t" << __LINE__ << "\tstrerror(errno):\t" << strerror(errno) << "\n";
//      // arbitrary number so we know this has happened
//      return 5000000;
//   }
//}
//
// this printed:
///home/jayen/docs/school/UNSW/robocup/2010/robot/utils/Timer.cpp 55      elapsed_us:     -4.29497e+09
///home/jayen/docs/school/UNSW/robocup/2010/robot/utils/Timer.cpp 56      tmp.tv_sec:     1552946357
///home/jayen/docs/school/UNSW/robocup/2010/robot/utils/Timer.cpp 57      tmp.tv_usec:    600925
///home/jayen/docs/school/UNSW/robocup/2010/robot/utils/Timer.cpp 58      timeStamp.tv_sec:       1552946357
///home/jayen/docs/school/UNSW/robocup/2010/robot/utils/Timer.cpp 59      timeStamp.tv_usec:      598717
///home/jayen/docs/school/UNSW/robocup/2010/robot/utils/Timer.cpp 60      strerror(errno):        Success

#include <cassert>
#include <ctime>
#include <iostream>

using namespace std;

double usec(const timeval &t) {
   cerr << __FILE__ << "\t" << __LINE__ << "\tt.tv_sec:\t" << t.tv_sec << "\n";
   cerr << __FILE__ << "\t" << __LINE__ << "\tt.tv_usec:\t" << t.tv_usec << "\n";
   cerr << __FILE__ << "\t" << __LINE__ << "\tt.tv_sec * 1000000ull:\t" << t.tv_sec * 1000000ull << "\n";
   cerr << __FILE__ << "\t" << __LINE__ << "\tt.tv_sec * 1000000ull + t.tv_usec:\t" << t.tv_sec * 1000000ull + t.tv_usec << "\n";
   return t.tv_sec * 1000000ull + t.tv_usec;
}

int main() {
   timeval timeStamp  = {1552946357, 598717};
   timeval tmp        = {1552946357, 600925};
   double   elapsed_us = usec(tmp) - usec(timeStamp);
   cerr << __FILE__ << "\t" << __LINE__ << "\tusec(tmp):\t" << usec(tmp) << "\n";
   cerr << __FILE__ << "\t" << __LINE__ << "\tusec(timeStamp):\t" << usec(timeStamp) << "\n";
   cerr << __FILE__ << "\t" << __LINE__ << "\telapsed_us:\t" << elapsed_us << "\n";
   assert(elapsed_us == 2208);
}

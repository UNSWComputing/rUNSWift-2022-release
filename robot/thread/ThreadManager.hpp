#pragma once

#include <pthread.h>
#include <signal.h>
#include <setjmp.h>
#include <sys/time.h>
#include <string>
#include <thread/Thread.hpp>
#include <utils/speech.hpp>
#include <utils/ConcurrentMap.hpp>
#include <utils/Timer.hpp>
#include <blackboard/Blackboard.hpp>

#define ALL_SIGNALS -1  // for indicating that we should register
                        // all signal handlers
#define TICK_AS_FAST_AS_POSSIBLE 0
#define TICK_NEVER -1


class ThreadManager;

extern bool attemptingShutdown;
extern ConcurrentMap<pthread_t, jmp_buf*>     jumpPoints;

struct SafelyRunArgs {
   ThreadManager *threadManager;
   Blackboard *blackboard;
};

template<class T, void(T::*mem_fn) (Blackboard *)> void* thunk(void* args);

void overtimeAlert(int);
void handleSignals(int sigNumber, siginfo_t* info, void*);
void registerSignalHandlers(int signal = ALL_SIGNALS);

class ThreadManager {
   public:
      std::string name;
      int padMicroseconds;
      pthread_t pthread;
      bool running;

      ThreadManager(const char *name, int padMicroseconds);

      template <class T> void run(Blackboard *bb);

      void join();

      ~ThreadManager();

   private:

      template <class T>
      void safelyRun(Blackboard *bb);
      SafelyRunArgs args;
};

#include "ThreadManager.tcc"

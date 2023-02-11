#include <malloc.h>
#include <limits.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cassert>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <iostream>
#include <stdexcept>
#include "perception/vision/camera/NaoCameraV4.hpp"
#include "utils/Logger.hpp"
#include "utils/speech.hpp"
#include "utils/Timer.hpp"


using namespace std;

/**
 * reads the system error and writes it to the log, then throws an exception
 * @param s an additional string, decribing the failed action
 */
static inline void errno_throw(const char *s) {
   llog(ERROR) << s << " error "<< errno << ", " << strerror(errno) << endl;
   throw runtime_error(strerror(errno));
}

/**
 * sets all bytes of x to 0
 * @param x the variable to clear
 */
#define CLEAR(x) memset(&(x), 0, sizeof(x))

/**
 * Set the initial settings for the camera(format, etc.)
 */
NaoCameraV4::NaoCameraV4(Blackboard *blackboard,
                         const char *filename,
                         const std::string cameraChoice,
                         const IOMethod method,
                         const int format)
:
   /* 0 is a dummy value that causes the protected constructor to be called */
   NaoCamera(blackboard, filename, method, format, 0, cameraChoice)
{
   // open your camera
   open_device();
   // decide if its a file or a device
   struct stat buf;
   if (fstat(fd, &buf) < 0) {
      llog(ERROR) << "failed to stat " << filename << endl;
      throw runtime_error("inside NaoCamera.cpp, failed to stat camera");
   }
   v4lDeviceP = S_ISCHR(buf.st_mode);

  #ifdef CTC_2_1
   // fix green camera bug
   setControl(V4L2_CID_CAM_INIT, 0);
  #endif

   // reopen your camera (SET_DEFAULT_PARAMETERS causes hang on DQBUF)
   close_device();
   open_device();
   
   init_buffers();
   
   if (!init_camera()) {
      llog(FATAL) << "Error initializing camera!\n";
      throw runtime_error("Error initializing camera");
   }

   start_capturing();

   // close everything
   stop_capturing();
   uninit_buffers();
   close_device();

   // re open it again
   open_device();
   init_buffers();

   if (!init_camera()) {
      llog(FATAL) << "Error initializing camera!\n";
      throw runtime_error("Error initializing camera");
   }

   start_capturing();

   dumpFile = NULL;
}

NaoCameraV4::~NaoCameraV4()
{
}

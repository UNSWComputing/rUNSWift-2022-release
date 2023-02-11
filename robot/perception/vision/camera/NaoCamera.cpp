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
#include "perception/vision/camera/NaoCamera.hpp"
#include "utils/Logger.hpp"
#include "utils/speech.hpp"
#include "utils/Timer.hpp"
#include "blackboard/Blackboard.hpp"

#ifndef CTC_2_1

#include <cstdarg>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#endif // CTC_2_1

#define EXTENSION_UNIT_ID 3 // On the V6, this is needed

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


#define CAMERA_FRAMERATE 30


bool NaoCamera::setControl(const uint32_t controlId,
                           const int32_t controlValue) {
   if (v4lDeviceP) {
      struct v4l2_control control;
      CLEAR(control);
      if (controlId < V4L2_CID_BASE)
         control.id = V4L2_CID_BASE + controlId;
      else
         control.id    = controlId;
      control.value = controlValue;

      struct v4l2_control current;
      CLEAR(current);
      current.id = control.id;
      ioctl (fd, VIDIOC_G_CTRL, &current);
      std::cerr << fd << " - Setting id " << controlId <<
         " \033[36m" << CAMERA_CONTROL_VALUE_MAP[controlId] <<
         ": " << controlValue << "\033[0m...";
      int i = 0;
      while (current.value != controlValue) {
//         CLEAR(current);
//         current.id = control.id;
         ioctl(fd, VIDIOC_S_CTRL, &control);
         ioctl (fd, VIDIOC_G_CTRL, &current);
         //cout << "setting id " << control.id << "to " << controlValue << std::endl;
         if (++i == 1000000) {
            SAY("Nao camera set control is stuck!  This head may be dead!");
            std::cerr << "\tstuck!\n"
                         "\n"
                         "Even a reboot doesn't fix this...  Not sure what to do...\n"
                         "\n";
         }
      }
      std::cerr << "\tOK!\n";
/*
      if (-1 == ioctl(fd, VIDIOC_S_CTRL, &control) && errno != ERANGE) {
         llog(ERROR) << "VIDIOC_S_CTRL error " << errno << ": " <<
               strerror(errno) << endl << "control.id: " << control.id << endl
               << "control.value: " << control.value << endl;
         return false;
      }
*/
      return true;
   } else {
      llog(WARNING) << "tried to change settings on a file, not a v4l device";
      return false;
   }
}

#ifndef CTC_2_1
int set_uvc_xu(
   int device_fd,
   uint8_t extension_unit_id,
   uint8_t control_selector,
   uint16_t size,
   void *data)
{
   struct uvc_xu_control_query query = {
      extension_unit_id,
      control_selector,
      UVC_SET_CUR,
      size,
      (__u8 *) data
   };
   return ioctl(device_fd, UVCIOC_CTRL_QUERY, &query);
}

int get_uvc_xu(
   int device_fd,
   uint8_t extension_unit_id,
   uint8_t control_selector,
   uint16_t size,
   void *data)
{
   struct uvc_xu_control_query query = {
      extension_unit_id,
      control_selector,
      UVC_GET_CUR,
      size,
      (__u8 *) data
   };
   return ioctl(device_fd, UVCIOC_CTRL_QUERY, &query);
}

// from https://github.com/aldebaran/linux-aldebaran/blob/sbr/v4.4.86-rt99-baytrail/drivers/media/i2c/soc_camera/ov5640.c

#define AEC_STABLE_RANGE_HIGH           0x3A0F
#define AEC_STABLE_RANGE_LOW            0x3A10
#define AEC_HYSTERESIS_RANGE_HIGH       0x3A1B
#define AEC_HYSTERESIS_RANGE_LOW        0x3A1E

// The maximum gain and exposure values for a v6 camera.
#define V6_MAX_GAIN 1023
#define V6_MAX_EXPOSURE_ABSOLUTE 1048575
#define V6_MAX_BRIGHTNESS 255
#define V6_MAX_CONTRAST 255
#define V6_MAX_SATURATION 255

/*
WARNING: There are issues with reading and writing to the camera. We think data
3 is always ignored. It also appears that whatever you do, a read will just read
the value of the last register written to.
*/

int writeRegister(int fd, uint16_t addr, uint8_t val) {
   uint8_t data[5];
   data[0] = 1;
   data[1] = static_cast<uint8_t>(addr >> 8u);
   data[2] = static_cast<uint8_t>(addr & 0xffu);
   data[3] = 0x00;
   data[4] = val;
   int output = set_uvc_xu(fd, EXTENSION_UNIT_ID, 14, 5, data);
   if (output < 0)
      llog(ERROR) << "writeRegister-set_uvc_xu failed:\t" << strerror(errno) << "\n";
   return output;
}

void NaoCamera::setAutoExposureTarget(int fd, uint8_t high) {
   const uint8_t low = static_cast<const uint8_t>(high * 0x30 / 0x48); // scale based on defaults in kernel source
   writeRegister(fd, AEC_STABLE_RANGE_HIGH, high);
   writeRegister(fd, AEC_STABLE_RANGE_LOW, low);
   writeRegister(fd, AEC_HYSTERESIS_RANGE_HIGH, high);
   writeRegister(fd, AEC_HYSTERESIS_RANGE_LOW, low);
   // uvc_xu seems to read from the last address we wrote, so write to this one again so we can read it later
   writeRegister(fd, AEC_STABLE_RANGE_HIGH, high);
}
#endif

/**
 * Set the initial settings for the camera(format, etc.)
 */
NaoCamera::NaoCamera(Blackboard *blackboard, const char *filename, const IOMethod method,
                     const int format):
      filename(filename), io(method), format(format) {

   readCameraSettings(blackboard);
   // open your camera
   open_device();
   // decide if its a file or a device
   struct stat buf;
   if (fstat(fd, &buf) < 0) {
      llog(ERROR) << "failed to stat " << filename << endl;
      throw runtime_error("inside NaoCamera.cpp, failed to stat camera");
   }
   v4lDeviceP = S_ISCHR(buf.st_mode);

   // fix green camera bug
  #ifdef CTC_2_1
   setControl(V4L2_CID_CAM_INIT, 0);
  #endif

   // reopen your camera (SET_DEFAULT_PARAMETERS causes hang on DQBUF)
   close_device();
   open_device();

   init_buffers();

   // init() the first camera
   if (!init_camera()) {
      llog(FATAL) << "Error initializing camera!\n";
      throw runtime_error("Error initializing camera");
   }

   // because set camera stops capturing
   start_capturing();

   // because set camera starts capturing, and because we don't want to stream
   // while init'ing (although alvideo input inits while streaming)
   stop_capturing();

   if (!init_camera()) {
      llog(FATAL) << "Error initializing second camera!\n";
      throw runtime_error("Error initializing second camera");
   }
   start_capturing();
   dumpFile = NULL;
}


/**
 * Initialise parameters without initialising the camera.
 */
NaoCamera::NaoCamera(Blackboard *blackboard, const char *filename, const IOMethod method,
                     const int format,
                     int dummy, const std::string cameraChoice):
      filename(filename), io(method), format(format),
               cameraChoice(cameraChoice)
{
    readCameraSettings(blackboard);
}


void NaoCamera::readCameraSettings(Blackboard *blackboard) {
    readCameraSettings(blackboard, cameraSettings, cameraChoice);
}

void NaoCamera::readCameraSettings(Blackboard *blackboard,
        CameraSettings &settings, std::string cameraName) {
    settings.hflip = blackboard->config[(cameraName + ".hflip").c_str()].as<int>();
    settings.vflip = blackboard->config[(cameraName + ".vflip").c_str()].as<int>();
    settings.brightness = blackboard->config[(cameraName + ".brightness").c_str()].as<int>();
    settings.contrast = blackboard->config[(cameraName + ".contrast").c_str()].as<int>();
    settings.saturation = blackboard->config[(cameraName + ".saturation").c_str()].as<int>();
    settings.hue = blackboard->config[(cameraName + ".hue").c_str()].as<int>();
    settings.sharpness = blackboard->config[(cameraName + ".sharpness").c_str()].as<int>();
    settings.backlightCompensation = blackboard->config[(cameraName + ".backlightcompensation").c_str()].as<int>();
    settings.exposure = blackboard->config[(cameraName + ".exposure").c_str()].as<int>();
    settings.gain = blackboard->config[(cameraName + ".gain").c_str()].as<int>();
    settings.whiteBalance = blackboard->config[(cameraName + ".whitebalance").c_str()].as<int>();
    settings.exposureAuto = blackboard->config[(cameraName + ".exposureauto").c_str()].as<int>();
    settings.autoWhiteBalance = blackboard->config[(cameraName + ".autowhitebalance").c_str()].as<int>();
    settings.autoFocus = blackboard->config[(cameraName + ".autofocus").c_str()].as<int>();
    settings.focusAbsolute = blackboard->config[(cameraName + ".focusabsolute").c_str()].as<int>();
    settings.exposureAlgorithm = blackboard->config[(cameraName + ".exposurealgorithm").c_str()].as<int>();
    settings.aeTargetAvgLuma = blackboard->config[(cameraName + ".aetargetavgluma").c_str()].as<int>();
    settings.aeTargetAvgLumaDark = blackboard->config[(cameraName + ".aetargetavglumadark").c_str()].as<int>();
    settings.aeTargetGain = blackboard->config[(cameraName + ".aetargetgain").c_str()].as<int>();
    settings.aeMinVirtGain = blackboard->config[(cameraName + ".aeminvirtgain").c_str()].as<int>();
    settings.aeMaxVirtGain = blackboard->config[(cameraName + ".aemaxvirtgain").c_str()].as<int>();
    settings.aeMinVirtAGain = blackboard->config[(cameraName + ".aeminvirtagain").c_str()].as<int>();
    settings.aeMaxVirtAGain = blackboard->config[(cameraName + ".aemaxvirtagain").c_str()].as<int>();
    settings.aeTargetExposure = blackboard->config[(cameraName + ".aetargetexposure").c_str()].as<int>();
    settings.aeUseWeightTable = blackboard->config[(cameraName + ".aeuseweighttable").c_str()].as<bool>();
    settings.aeWeightTableX1 = blackboard->config[(cameraName + ".aeweighttablex1").c_str()].as<float>();
    settings.aeWeightTableX2 = blackboard->config[(cameraName + ".aeweighttablex2").c_str()].as<float>();
    settings.aeWeightTableY1 = blackboard->config[(cameraName + ".aeweighttabley1").c_str()].as<float>();
    settings.aeWeightTableY2 = blackboard->config[(cameraName + ".aeweighttabley2").c_str()].as<float>();

}

void NaoCamera::setCameraSettings(const CameraSettings settings, bool top) {
//   use `v4l2-ctl --list-ctrls-menus --device=/dev/video0` to see what options are available
//   use `v4l2-ctl --set-ctrl=<ctrl>=<val>[,<ctrl>=<val>...] --device=/dev/video0` to set ctrls, even while runswift & offnao are running
#ifdef CTC_2_1
    setControl(V4L2_CID_HFLIP, settings.hflip);
    setControl(V4L2_CID_VFLIP, settings.vflip);
    setControl(V4L2_CID_EXPOSURE_AUTO, settings.exposureAuto);
    setControl(V4L2_CID_AUTO_WHITE_BALANCE, settings.autoWhiteBalance);
    setControl(V4L2_CID_CONTRAST, settings.contrast);
    setControl(V4L2_CID_SATURATION, settings.saturation);
    setControl(V4L2_CID_HUE, settings.hue); // documentation on aldebaran says "disabled"?, enabled on rUNSWift driver
    setControl(V4L2_CID_SHARPNESS, settings.sharpness);

    if (settings.exposureAuto){
        setControl(V4L2_CID_BRIGHTNESS, settings.brightness);
        setControl(V4L2_CID_EXPOSURE_ALGORITHM, settings.exposureAlgorithm);
        setControl(V4L2_CID_BACKLIGHT_COMPENSATION, settings.backlightCompensation);
        setControl(V4L2_MT9M114_AE_TARGET_AVERAGE_LUMA, settings.aeTargetAvgLuma);
        setControl(V4L2_MT9M114_AE_TARGET_AVERAGE_LUMA_DARK, settings.aeTargetAvgLumaDark);
        setControl(V4L2_MT9M114_AE_TARGET_GAIN, settings.aeTargetGain);
        setControl(V4L2_MT9M114_AE_MIN_VIRT_DGAIN, settings.aeMinVirtGain);
        setControl(V4L2_MT9M114_AE_MAX_VIRT_DGAIN, settings.aeMaxVirtGain);
        setControl(V4L2_MT9M114_AE_MIN_VIRT_AGAIN, settings.aeMinVirtAGain);
        setControl(V4L2_MT9M114_AE_MAX_VIRT_AGAIN, settings.aeMaxVirtAGain);
        setControl(V4L2_MT9M114_FADE_TO_BLACK, 0);          // Fade to black under very low light conditions: 0 = disabled, 1 = enabled
        setControl(V4L2_CID_POWER_LINE_FREQUENCY, 1);       // AE avoids flicker caused by artifical lighting: 0 = disabled, 1 = 50Hz, 2 = 60Hz
    } else {
        setControl(V4L2_CID_EXPOSURE, settings.exposure);
        setControl(V4L2_CID_GAIN, settings.gain);
    }

    if (!settings.autoWhiteBalance){
        setControl(V4L2_CID_WHITE_BALANCE_TEMPERATURE, settings.whiteBalance);
    }
#else // not CTC_2_1
      // have to do it in this order when setting focus to be near
      setControl(V4L2_CID_FOCUS_ABSOLUTE, settings.focusAbsolute);
      setControl(V4L2_CID_FOCUS_AUTO, 1);
      setControl(V4L2_CID_FOCUS_AUTO, settings.autoFocus);
      // 0 is auto, 1 is manual. Need to toggle or it won't work sometimes.
      setControl(V4L2_CID_EXPOSURE_AUTO, settings.exposureAuto);
      setControl(V4L2_CID_EXPOSURE_AUTO, !settings.exposureAuto);
      if(settings.exposureAuto)
          setAutoExposureTarget(fd, settings.aeTargetExposure);

      setControl(V4L2_CID_BRIGHTNESS, (settings.brightness+1));// %
                                                             //V6_MAX_BRIGHTNESS);
      setControl(V4L2_CID_BRIGHTNESS, settings.brightness);
      setControl(V4L2_CID_CONTRAST, (settings.contrast+1));// % V6_MAX_CONTRAST);
      setControl(V4L2_CID_CONTRAST, settings.contrast);
      setControl(V4L2_CID_SATURATION, (settings.saturation+1));// %
                                                             //V6_MAX_SATURATION);
      setControl(V4L2_CID_SATURATION, settings.saturation);

      if (!settings.exposureAuto)
      {
          // Manual exposure does not work correctly unless the gain and
          // exposure are set to different values when activated.
          setControl(V4L2_CID_GAIN, (settings.gain+1) % V6_MAX_GAIN);
          setControl(V4L2_CID_GAIN, settings.gain);
          setControl(V4L2_CID_EXPOSURE_ABSOLUTE, (settings.exposure+1) %
                                                      V6_MAX_EXPOSURE_ABSOLUTE);
          setControl(V4L2_CID_EXPOSURE_ABSOLUTE, settings.exposure);
      }


      // Flip Camera Horizontally
   uint16_t valHFlip = static_cast<uint16_t>(settings.hflip);
   uint16_t curHFlip = ~valHFlip; // so it's initialized
      do
      {
        set_uvc_xu(fd, EXTENSION_UNIT_ID, 12, 2, &valHFlip);
        get_uvc_xu(fd, EXTENSION_UNIT_ID, 12, 2, &curHFlip);
      } while (curHFlip != valHFlip);

      // Flip Camera Vertically
   uint16_t valVFlip = static_cast<uint16_t>(settings.vflip);
   uint16_t curVFlip = ~valVFlip; // so it's initialized
      do
      {
        set_uvc_xu(fd, EXTENSION_UNIT_ID, 13, 2, &valVFlip);
        get_uvc_xu(fd, EXTENSION_UNIT_ID, 13, 2, &curVFlip);
      } while (curVFlip != valVFlip);

// For now, this aeUseWeightTable only works for top camera (dont' use for bottom)
    if (settings.aeUseWeightTable){
        unsigned width = top ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
        unsigned height = top ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;
        // Refer to https://cdn.sparkfun.com/datasheets/Sensors/LightImaging/OV5640_datasheet.pdf for details of following array
        uint8_t windowVals[17] = {
              1, // enable
              0, // upper four bits of x coordinate auto exposure window start
              0, // lower eight bits of x coordinate auto exposure window start
              0, // upper three bits of y coordinate auto exposure window start
              0, // lower eight bits of y coordinate auto exposure window start
              (uint8_t) (width >> 8u), // upper four bits of x coordinate auto exposure window end
              (uint8_t) width,  // lower eight bits of x coordinate auto exposure window end
              (uint8_t) (height >> 8u), // upper three bits of y coordinate auto exposure window end
              (uint8_t) height,  // upper eight bits of y coordinate auto exposure window end
              0u << 4u | 0u,
              0u << 4u | 0u,
              0u << 4u | 0u,
              0u << 4u | 0u,
              10u << 4u | 10u,
              10u << 4u | 10u,
              10u << 4u | 10u,
              10u << 4u | 10u};

        uint8_t  curWindowVals[17];
        for (int wv = 0; wv < 17; ++wv) {
           curWindowVals[wv] = ~windowVals[wv];
        }
        do
        {
            set_uvc_xu(fd, EXTENSION_UNIT_ID, 9, 17, windowVals);
            get_uvc_xu(fd, EXTENSION_UNIT_ID, 9, 17, curWindowVals);
        } while (memcmp(curWindowVals, windowVals, 17));
    }
#endif


}

NaoCamera::~NaoCamera() {
   stop_capturing();
   uninit_buffers();
   close_device();
}

const uint8_t *NaoCamera::read_frame(void) {
   unsigned int i;
   const uint8_t *image = NULL;
   switch (io) {
      /* reading from file and video device are exactly the same */
      case IO_METHOD_READ:
         if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
            switch (errno) {
               case EAGAIN:
                  return NULL;

               case EIO:
                  /* Could ignore EIO, see spec. */

                  /* fall through */

               default:
                  errno_throw("read");
            }
         }

         image = buffers[0].start;

         break;

      case IO_METHOD_MMAP:
         if (lastDequeued.index != UINT_MAX) {
            if (-1 == ioctl(fd, VIDIOC_QBUF, &lastDequeued)) {
               errno_throw("VIDEOC_BUF");
            }
         }

         CLEAR(lastDequeued);

         lastDequeued.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
         lastDequeued.memory = V4L2_MEMORY_MMAP;

         if (-1 == ioctl(fd, VIDIOC_DQBUF, &lastDequeued)) {
            switch (errno) {
               case EAGAIN:
                  return NULL;

               case EIO:
                  /* Could ignore EIO, see spec. */
                  std::cout << "eio" << std::endl;

                  /* fall through */
               case EFAULT:
                  std::cout << "efault" << std::endl;

               case EINVAL:
                  std::cout << "einval" << std::endl;

               default:
                  std::cout << "lol mmap" << std::endl;
                  errno_throw("VIDIOC_DQBUF");
            }
         }

         assert(lastDequeued.index < n_buffers);

         image = buffers[lastDequeued.index].start;

         break;

      case IO_METHOD_USERPTR:
         // TODO(jayen): verify that lastDequeued fix works for user pointers
         if (lastDequeued.index != UINT_MAX) {
            if (-1 == ioctl(fd, VIDIOC_QBUF, &lastDequeued)) {
               errno_throw("VIDEOC_BUF");
            }
         }

         CLEAR(lastDequeued);

         lastDequeued.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
         lastDequeued.memory = V4L2_MEMORY_USERPTR;

         if (-1 == ioctl(fd, VIDIOC_DQBUF, &lastDequeued)) {
            switch (errno) {
               case EAGAIN:
                  return NULL;

               case EIO:
                  /* Could ignore EIO, see spec. */

                  /* fall through */

               default:
                  std::cout << "lol user?" << std::endl;
                  errno_throw("VIDIOC_DQBUF");
            }
         }

         for (i = 0; i < n_buffers; ++i)
            if (lastDequeued.m.userptr == (unsigned long) buffers[i].start
                && lastDequeued.length == buffers[i].length)
               break;

         assert(i < n_buffers);

         image =(const uint8_t*) lastDequeued.m.userptr;

         break;

      default:
         throw runtime_error("what kind of IO method is that?!?!?");
   }

   return image;
}

const uint8_t *NaoCamera::get(const __u32 colourSpace) {
   if (colourSpace != V4L2_PIX_FMT_YUYV)
      throw runtime_error("only yuv422 is supported!");
   fd_set fds;
   struct timeval tv;
   int r;
   FD_ZERO(&fds);
   FD_SET(fd, &fds);

   /* Timeout. */
   tv.tv_sec = 0;
   tv.tv_usec = 100000;

   if (io == IO_METHOD_MMAP) {
      select:
      r = select(fd + 1, &fds, NULL, NULL, &tv);

      if (-1 == r) {
         if (EINTR == errno)
            goto select;  // Got interrupted (by profiler?)

         errno_throw("select");
      }

#ifndef CTC_2_1
      // Don't say this on V5 because we get this too often. If we hear this on
      // a V6, that's an issue.
      if (0 == r) {
         SAY("Camera took more than 100 milliseconds to return an image");
      }
#endif // CTC_2_1
   }

   const uint8_t *image = read_frame();
   //writeFrame(image);
   llog(DEBUG2) << "image returning from NaoCamera: " << (void *)image << endl;
   return image;
}

bool NaoCamera::init_camera() {
   if (v4lDeviceP) {
  #ifdef CTC_2_1
      // GET video device information
      v4l2_std_id esid0;
      CLEAR (esid0);
      int test;
      test = ioctl(fd, VIDIOC_G_STD, &esid0);
      if (test != 0) {
         perror("ioctl 1");
         llog(ERROR) << "failed ioctl with error code " << test << endl;
         throw runtime_error("inside NaoCamera.cpp, failed ioctl");
      }

      // set video device standard
      switch (format) {
         // aldebaran is invalidly using this field
         // http:// v4l2spec.bytesex.org/spec-single/v4l2.html#V4L2-STD-ID
         // The 32 most significant bits are reserved for custom(driver defined)
         // video standards.
         // There isn't supposed to be a way to specify QQVGA with this field
         // REVIEW: can these not be defined earlier?
         case k960p:
            esid0 =(v4l2_std_id)0x08000000; /*VGA*/
            break;
         case kVGA:
            esid0 =(v4l2_std_id)0x08000000; /*VGA*/
            break;
         case kQVGA:
            esid0 =(v4l2_std_id)0x04000000; /*QVGA*/
            break;
         default:
            throw runtime_error("inside NaoCamera.cpp, unknown format");
      }
      test = ioctl(fd, VIDIOC_S_STD, &esid0);
      if (test != 0) {
         perror("ioctl 2");
         llog(ERROR) << "failed ioctl 2 error " << test << endl;
      }
      // set video device format
      struct v4l2_format fmt0;
      memset(&fmt0, 0, sizeof(fmt0));
      fmt0.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      fmt0.fmt.pix.field = V4L2_FIELD_NONE;

      switch (format) {
         case k960p:
            fmt0.fmt.pix.width       = 1280;
            fmt0.fmt.pix.height      = 960;
            break;
         case kVGA:
            fmt0.fmt.pix.width       = 640;
            fmt0.fmt.pix.height      = 480;
            break;
         case kQVGA:
            fmt0.fmt.pix.width       = 320;
            fmt0.fmt.pix.height      = 240;
            break;
         default:
            throw runtime_error("inside NaoCamera.cpp, unknown format");
      }

      // fmt0.fmt.pix.pixelformat = 0x56595559;

      fmt0.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      test = ioctl(fd, VIDIOC_S_FMT, &fmt0);
      if (test != 0) {
         perror("ioctl 3");
         llog(ERROR) << "failed ioctl 3 error" << test << endl;
         throw runtime_error("NaoCamera.cpp, failed ioctl 3");
      }

      // set fps
      struct v4l2_streamparm parm;
      CLEAR(parm);

      parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      test = ioctl(fd, VIDIOC_G_PARM, &parm);
      if (test != 0) {
         perror("ioctl 4");
         llog(ERROR) << "failed ioctl 4" << endl;
         throw runtime_error("NaoCamera.cpp, failed ioctl 4");
      }
      parm.parm.capture.timeperframe.numerator = 1;
      parm.parm.capture.timeperframe.denominator = CAMERA_FRAMERATE;
      parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
      ioctl(fd, VIDIOC_S_PARM, &parm);
  #else
      int test;

      // set fps
      struct v4l2_streamparm parm;
      CLEAR(parm);

      parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      test = ioctl(fd, VIDIOC_G_PARM, &parm);
      if (test != 0) {
         errno_throw("VIDIOC_G_PARM");
      }
      // cout << "Numerator: " << parm.parm.capture.timeperframe.numerator << "\n";
      // cout << "Denominator: " << parm.parm.capture.timeperframe.denominator << "\n";
      // cout << "Capability: " << parm.parm.capture.capability << "\n";
      // cout << "Capture Mode: " << parm.parm.capture.capturemode << "\n";
      // cout << "Extened mode: " << parm.parm.capture.extendedmode << "\n";
      // cout << "Buffers: " << parm.parm.capture.readbuffers << "\n";

      parm.parm.capture.timeperframe.numerator = 1;
      parm.parm.capture.timeperframe.denominator = CAMERA_FRAMERATE;
      parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
      test = ioctl(fd, VIDIOC_S_PARM, &parm);
      if (test != 0) {
         errno_throw("VIDIOC_S_PARM");
      }

      // Print availble resolutions for requested pixel_format
	// int ret = 0;
	// int fsizeind = 0; /*index for supported sizes*/
	// struct v4l2_frmsizeenum fsize;
      // printf("V4L2 pixel sizes:\n");
	// CLEAR(fsize);
	// fsize.index = 0;
	// fsize.pixel_format = V4L2_PIX_FMT_YUYV;
	// while ((ret = ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fsize)) == 0)	{
	// 	fsize.index++;
	// 	if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
	// 		printf("( %u x %u ) Pixels\n", fsize.discrete.width, fsize.discrete.height);
	// 		fsizeind++;
	// 	}
	// }


      // set video device format
      struct v4l2_format fmt0;
      CLEAR(fmt0);
      memset(&fmt0, 0, sizeof(fmt0));
      fmt0.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      test = ioctl(fd, VIDIOC_G_FMT, &fmt0);
      if (test != 0) {
         errno_throw("ioctl 3.a");
      }
      else {
         cout << "Width: " << fmt0.fmt.pix.width << "\n";
         cout << "Height: " << fmt0.fmt.pix.height << "\n";
         cout << "Pix format: " << fmt0.fmt.pix.pixelformat << "\n";
         cout << "Field: " << fmt0.fmt.pix.field << "\n";
         cout << "Colour Space: " << fmt0.fmt.pix.colorspace << "\n";
      }

      fmt0.fmt.pix.field = V4L2_FIELD_NONE;

      switch (format) {
         case k960p:
            fmt0.fmt.pix.width       = 1280;
            fmt0.fmt.pix.height      = 960;
            break;
         case kVGA:
            fmt0.fmt.pix.width       = 640;
            fmt0.fmt.pix.height      = 480;
            break;
         case kQVGA:
            fmt0.fmt.pix.width       = 320;
            fmt0.fmt.pix.height      = 240;
            break;
         default:
            throw runtime_error("inside NaoCamera.cpp, unknown format");
      }
      // fmt0.fmt.pix.pixelformat = 0x56595559;

      fmt0.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

      if (ioctl(fd, VIDIOC_TRY_FMT, &fmt0) == -1) {
            errno_throw("VIDIOC_TRY_FMT");
      }

      test = ioctl(fd, VIDIOC_G_FMT, &fmt0);
      if (test != 0) {
         errno_throw("ioctl 3");
         cout << "failed ioctl 3 error" << test << endl;
         throw runtime_error("NaoCamera.cpp, failed ioctl 3");
      }
  #endif
      imageSize = fmt0.fmt.pix.sizeimage;

      setCameraSettings(cameraSettings, cameraChoice == "camera.top");
   }

   return true;
}

void NaoCamera::stop_capturing(void) {
   enum v4l2_buf_type type;

   switch (io) {
      case IO_METHOD_READ:
         /* Nothing to do. */
         break;

      case IO_METHOD_MMAP:
      case IO_METHOD_USERPTR:
         type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

         if (-1 == ioctl(fd, VIDIOC_STREAMOFF, &type))
            errno_throw("VIDIOC_STREAMOFF");

         break;
      default:
         throw runtime_error("unknown io method");
         break;
   }
}

void NaoCamera::uninit_buffers(void) {
   unsigned int i;

   switch (io) {
      case IO_METHOD_READ:
         free(buffers[0].start);
         break;

      case IO_METHOD_MMAP:
         for (i = 0; i < n_buffers; ++i)
            if (-1 == munmap(buffers[i].start, buffers[i].length))
               errno_throw("munmap");
         break;

      case IO_METHOD_USERPTR:
         for (i = 0; i < n_buffers; ++i)
            free(buffers[i].start);
         break;
      default:
         throw runtime_error("unknown io method");
         break;
   }
}

void NaoCamera::open_device(void) {
   fd = open(filename.c_str(), O_CLOEXEC | O_RDWR);

   if (fd < 0) {
      llog(ERROR) << "failed to open " << filename << endl;
      SAY(filename + strerror(errno));
      sleep(5);
      throw runtime_error("inside NaoCamera.cpp, failed to open camera");
   }
}

void NaoCamera::close_device(void) {
   if (-1 == close(fd))
      errno_throw("close");

   fd = -1;
}

void NaoCamera::init_mmap(void) {
   struct v4l2_requestbuffers req;

   CLEAR(req);

   req.count  = NUM_FRAME_BUFFERS;
   req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   req.memory = V4L2_MEMORY_MMAP;

   if (-1 == ioctl(fd, VIDIOC_REQBUFS, &req)) {
      if (EINVAL == errno) {
         llog(ERROR) << "memory mapping not supported on this device!" << endl;
         exit(EXIT_FAILURE);
      } else {
         errno_throw("VIDIOC_REQBUFS");
      }
   }

   if (req.count < 1) {
      llog(ERROR) << "Insufficient buffer memory" << endl;
      throw runtime_error("Insufficient buffer memory");
   }

   CLEAR(buffers);

   if (!buffers) {
      llog(ERROR) << "Out of memory\n";
      exit(EXIT_FAILURE);
   }

   for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
      struct v4l2_buffer buf;

      CLEAR(buf);

      buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index  = n_buffers;

      if (-1 == ioctl(fd, VIDIOC_QUERYBUF, &buf))
         errno_throw("VIDIOC_QUERYBUF");

      buffers[n_buffers].length = buf.length;
      buffers[n_buffers].start = reinterpret_cast<uint8_t*>
            (mmap(NULL /* start anywhere */,
                  buf.length,
                  PROT_READ | PROT_WRITE /* required */,
                  MAP_SHARED /* recommended */,
                  fd, buf.m.offset));

      if (MAP_FAILED == buffers[n_buffers].start)
         errno_throw("mmap");
   }
}

void NaoCamera::init_buffers(void) {
   if (v4lDeviceP) {
  #ifdef CTC_2_1
      struct v4l2_capability cap;
      struct v4l2_cropcap cropcap;
      struct v4l2_crop crop;
      struct v4l2_format fmt;
      v4l2_std_id esid0;
      unsigned int min;

      CLEAR(cap);
      CLEAR(cropcap);
      CLEAR(crop);
      CLEAR(fmt);
      if (-1 == ioctl(fd, VIDIOC_QUERYCAP, &cap)) {
         if (EINVAL == errno) {
            llog(ERROR) << "not an V4L2 device" << endl;
            throw runtime_error("not a v4l2 device");
         } else {
            errno_throw("VIDIOC_QUERYCAP");
         }
      }

      if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
         llog(ERROR) << "not a video capture device" << endl;
         throw runtime_error("not a video capture device");
      }

      switch (io) {
         case IO_METHOD_READ:
            if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
               llog(ERROR) << "does not support read i/o" << endl;
               throw runtime_error("does not support read i/o");
            }

            break;

         case IO_METHOD_MMAP:
         case IO_METHOD_USERPTR:
            if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
               llog(ERROR) << "does not support streaming i/o" << endl;
               throw runtime_error("does not support streaming i/o");
            }

            break;

         default:
            throw runtime_error("what kind of IO method is that?!?!?");
      }


      /* Select video input, video standard and tune here. */


      CLEAR(cropcap);

      cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (0 == ioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
         crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
         crop.c = cropcap.defrect; /* reset to default */

         if (-1 == ioctl(fd, VIDIOC_S_CROP, &crop)) {
            switch (errno) {
               case EINVAL:
                  /* Cropping not supported. */
                  break;
               default:
                  /* Errors ignored. */
                  break;
            }
         }
      } else {
         /* Errors ignored. */
      }

      switch (format) {
         // REVIEW: this is done somewhere else - maybe we can refactor it out
         // aldebaran is invalidly using this field
         // http:// v4l2spec.bytesex.org/spec-single/v4l2.html#V4L2-STD-ID
         // The 32 most significant bits are reserved for custom(driver defined)
         // video standards.
         // There isn't supposed to be a way to specify QQVGA with this field
         case k960p:
            esid0 = 0x08000000UL; /*VGA*/
            break;
         case kVGA:
            esid0 = 0x08000000UL; /*VGA*/
            break;
         case kQVGA:
            esid0 = 0x04000000UL; /*QVGA*/
            break;
      }
      if (-1 == ioctl(fd, VIDIOC_S_STD, &esid0))
         errno_throw("VIDIOC_S_STD");

      CLEAR(fmt);

      fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      switch (format) {
         case k960p:
            fmt.fmt.pix.width     = 1280;
            fmt.fmt.pix.height    = 960;
            break;
         case kVGA:
            fmt.fmt.pix.width     = 640;
            fmt.fmt.pix.height    = 480;
            break;
         case kQVGA:
            fmt.fmt.pix.width     = 320;
            fmt.fmt.pix.height    = 240;
            break;
      }
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      fmt.fmt.pix.field       = V4L2_FIELD_NONE;

      if (-1 == ioctl(fd, VIDIOC_G_FMT, &fmt))
         errno_throw("VIDIOC_G_FMT");
  #else
      struct v4l2_capability cap;
      struct v4l2_cropcap cropcap;
      struct v4l2_crop crop;
      struct v4l2_format fmt;
      struct v4l2_input input;
      unsigned int min;

      CLEAR(cap);
      CLEAR(cropcap);
      CLEAR(crop);
      CLEAR(fmt);
      CLEAR(input);

      if (-1 == ioctl(fd, VIDIOC_QUERYCAP, &cap)) {
         if (EINVAL == errno) {
            cout << "not an V4L2 device" << endl;
            throw runtime_error("not a v4l2 device");
         } else {
            errno_throw("VIDIOC_QUERYCAP");
         }
      }

      // input.index = 0;
      // if (ioctl(fd, VIDIOC_ENUMINPUT, &input) == -1) {
      //     errno_throw("VIDIOC_ENUMINPUT");
      // }

      // if (ioctl(fd, VIDIOC_S_INPUT, &input.index) == -1) {
      //       errno_throw("VIDIOC_S_INPUT");
      // }

      // printf("Driver: \"%s\"\n", cap.driver);
      // printf("Card: \"%s\"\n", cap.card);
      // printf("Bus: \"%s\"\n", cap.bus_info);
      // printf("Version: %d.%d\n", (cap.version >> 16) && 0xff, (cap.version >> 24) && 0xff);
      // printf("Capabilities: %08x\n", cap.capabilities);

      // printf("Index: \"%d\"\n", input.index);
      // printf("Name: \"%s\"\n", input.name);
      // printf("Type: \"%d\"\n", input.type);
      // printf("Audio Set: \"%d\"\n", input.audioset);
      // printf("Video Stds: \"%lld\"\n", input.std);
      // printf("Status: \"%d\"\n", input.status);
      // printf("Caps: \"%d\"\n", input.capabilities);

      if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
         cout << "not a video capture device" << endl;
         throw runtime_error("not a video capture device");
      }

      switch (io) {
         case IO_METHOD_READ:
            if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
               cout << "does not support read i/o" << endl;
               throw runtime_error("does not support read i/o");
            }

            break;

         case IO_METHOD_MMAP:
         case IO_METHOD_USERPTR:
            if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
               cout << "does not support streaming i/o" << endl;
               throw runtime_error("does not support streaming i/o");
            }

            break;

         default:
            throw runtime_error("what kind of IO method is that?!?!?");
      }


      /* Select video input, video standard and tune here. */


      CLEAR(cropcap);

      cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (0 == ioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
         crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
         crop.c = cropcap.defrect; /* reset to default */

      // V6 replies inapproprite ioctl for device
      //    if (-1 == ioctl(fd, VIDIOC_S_CROP, &crop)) {
      //       switch (errno) {
      //          case EINVAL:
      //             /* Cropping not supported. */
      //             break;
      //          default:
      //             /* Errors ignored. */
      //             break;
      //       }
      //    }

      } else {
         /* Errors ignored. */
      }

      CLEAR(fmt);

      fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      switch (format) {
         case k960p:
            fmt.fmt.pix.width     = 1280;
            fmt.fmt.pix.height    = 960;
            break;
         case kVGA:
            fmt.fmt.pix.width     = 640;
            fmt.fmt.pix.height    = 480;
            break;
         case kQVGA:
            fmt.fmt.pix.width     = 320;
            fmt.fmt.pix.height    = 240;
            break;
      }
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      fmt.fmt.pix.field       = V4L2_FIELD_NONE;

      // int test;
      // if (-1 == ioctl(fd, VIDIOC_G_FMT, &fmt))
      //    errno_throw("VIDIOC_G_FMT");
      // test = ioctl(fd, VIDIOC_G_FMT, &fmt);
      // if (test != 0) {
      //    errno_throw("ioctl 3.a");
      // }
      // else {
      //    cout << "Width: " << fmt.fmt.pix.width << "\n";
      //    cout << "Height: " << fmt.fmt.pix.height << "\n";
      //    cout << "Pix format: " << fmt.fmt.pix.pixelformat << "\n";
      //    cout << "Field: " << fmt.fmt.pix.field << "\n";
      //    cout << "Colour Space: " << fmt.fmt.pix.colorspace << "\n";
      // }
  #endif
      if (-1 == ioctl(fd, VIDIOC_S_FMT, &fmt))
         errno_throw("VIDIOC_S_FMT");

      /* Note VIDIOC_S_FMT may change width and height. */

      /* Buggy driver paranoia. */
      min = fmt.fmt.pix.width * 2;
      if (fmt.fmt.pix.bytesperline < min)
         fmt.fmt.pix.bytesperline = min;
      min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
      if (fmt.fmt.pix.sizeimage < min)
         fmt.fmt.pix.sizeimage = min;

      imageSize = fmt.fmt.pix.sizeimage;
   } else {
      llog(INFO) << "no need to init buffers for files" << endl;
   }
   switch (io) {
      case IO_METHOD_READ:
         init_read();
         break;

      case IO_METHOD_MMAP:
         init_mmap();
         break;

      case IO_METHOD_USERPTR:
         init_userp();
         break;

      default:
         throw runtime_error("what kind of IO method is that?!?!?");
   }
}

void NaoCamera::init_userp() {
   struct v4l2_requestbuffers req;
   unsigned int page_size;

   page_size = getpagesize();
   imageSize =(imageSize + page_size - 1) & ~(page_size - 1);

   CLEAR(req);

   req.count  = NUM_FRAME_BUFFERS;
   req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   req.memory = V4L2_MEMORY_USERPTR;

   if (-1 == ioctl(fd, VIDIOC_REQBUFS, &req)) {
      if (EINVAL == errno) {
         llog(ERROR) << "does not support user pointer i/o" << endl;
         throw runtime_error("does not support user pointer i/o");
      } else {
         errno_throw("VIDIOC_REQBUFS");
      }
   }

   CLEAR(buffers);

   if (!buffers) {
      llog(ERROR) << "Out of memory\n";
      exit(EXIT_FAILURE);
   }

   for (n_buffers = 0; n_buffers < NUM_FRAME_BUFFERS; ++n_buffers) {
      buffers[n_buffers].length = imageSize;
      buffers[n_buffers].start =
            reinterpret_cast<uint8_t*>(memalign(/* boundary */ page_size,
                                                imageSize));

      if (!buffers[n_buffers].start) {
         llog(ERROR) << "Out of memory\n";
         exit(EXIT_FAILURE);
      }
   }
}

void NaoCamera::start_capturing(void) {
   // assign an impossible index to lastDequeued so we know not to
   // re-enqueue if there have been no previous buffers
   lastDequeued.index = UINT_MAX;

   unsigned int i;
   enum v4l2_buf_type type;

   switch (io) {
      case IO_METHOD_READ:
         /* Nothing to do. */
         break;

      case IO_METHOD_MMAP:
         for (i = 0; i < n_buffers; ++i) {
            struct v4l2_buffer buf;

            CLEAR(buf);

            buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory      = V4L2_MEMORY_MMAP;
            buf.index       = i;

            if (-1 == ioctl(fd, VIDIOC_QBUF, &buf))
               errno_throw("VIDIOC_QBUF");
         }

         type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

         if (-1 == ioctl(fd, VIDIOC_STREAMON, &type))
            errno_throw("VIDIOC_STREAMON");

         break;

      case IO_METHOD_USERPTR:
         for (i = 0; i < n_buffers; ++i) {
            struct v4l2_buffer buf;

            CLEAR(buf);

            buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory      = V4L2_MEMORY_USERPTR;
            buf.index       = i;
            buf.m.userptr   = (unsigned long) buffers[i].start;
            buf.length      = buffers[i].length;

            if (-1 == ioctl(fd, VIDIOC_QBUF, &buf))
               errno_throw("VIDIOC_QBUF");
         }

         type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

         if (-1 == ioctl(fd, VIDIOC_STREAMON, &type))
            errno_throw("VIDIOC_STREAMON");

         break;

      default:
         throw runtime_error("what kind of IO method is that?!?!?");
   }
}

void NaoCamera::init_read(void) {
   CLEAR(buffers);

   if (!buffers) {
      llog(ERROR) << "Out of memory\n";
      exit(EXIT_FAILURE);
   }

   buffers[0].length = imageSize;
   buffers[0].start  = reinterpret_cast<uint8_t*>(malloc(imageSize));

   if (!buffers[0].start) {
      llog(ERROR) << "Out of memory\n";
      exit(EXIT_FAILURE);
   }
}

//
// Created by jayen on 16/03/19.
//
#include "Camera.hpp"
#include "NaoCamera.hpp"
#include <iostream>

static const int  EXPOSURE_HIGH_LEN                    = 16;
static const char EXPOSURE_HIGH[EXPOSURE_HIGH_LEN + 1] = "exposure_target=";
using namespace std;

void Camera::terminalCalibration() {
   static string device = VIDEO_TOP;
   string        input;
   string        command;

   do {
      cout << "\n\n";
      command = "v4l2-ctl --device=" + device + " --list-ctrls-menus";
      system(command.c_str());
#ifndef CTC_2_1
      int fd = open(device.c_str(), O_CLOEXEC | O_RDWR);
      if (fd >= 0) {
//         upcast to int for printing because printing uint8_t is printing char
         cout << "                exposure_target (int)    : min=0 max=255 default=120 value=unreadable\n";
         close(fd);
      } else {
         cerr << "failed to open " << device << endl;
      }
#endif

      cout << "\n"
              "Menu:\n"
              "- /dev/video<foo>                switch to another device\n"
              #ifndef CTC_2_1
              "- exposure_target=<n>            set auto exposure target range (low=high*2/3)\n"
              #endif
              "- <ctrl>=<val>[,<ctrl>=<val>...] set control(s) to value(s)\n"
              "- q                              quit calibration and let runswift run swift\n"
              "> "
           << flush;
      if (cin >> input) {
         if (input.find("/dev/") == 0) {
            device = input;
#ifndef CTC_2_1
         } else if (input.find(EXPOSURE_HIGH) == 0) {
            fd = open(device.c_str(), O_CLOEXEC | O_RDWR);
            if (fd >= 0) {
               string high = input.substr(EXPOSURE_HIGH_LEN);
               NaoCamera::setAutoExposureTarget(fd, static_cast<uint8_t>(strtoul(high.c_str(), NULL, 0)));
               close(fd);
            } else {
               cerr << "failed to open " << device << endl;
            }
#endif
         } else if ((int) input.find('=') > 0) { // for some strange reason, string::find returns an unsigned int...
            command = "v4l2-ctl --device=" + device + " --set-ctrl=" + input;
            system(command.c_str());
         }
      }
   } while (cin && input != "q");

   cout << "Now press ctrl-c to exit runswift" << endl;
}

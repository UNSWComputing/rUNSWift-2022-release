//
// Created by jayen on 30/06/19.
//
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <string>

using namespace std;

FILE *pipe = NULL;

FILE *openSay() {
   const char *command = "/home/nao/bin/say.py";
   FILE       *pipe    = popen(command, "w");
   if (!pipe) {
      string command_string = getenv("RUNSWIFT_CHECKOUT_DIR");
      command_string += "/image";
      command_string += command;

      pipe = popen(command_string.c_str(), "w");
      if (!pipe) {
         cerr << "error starting " << command << "\n";
      }
   }
   return pipe;
}

FILE *closeSay(FILE *pipe) {
   // say.py doesn't always die, even if i explicitly send an EOF
   // may already be fixed by the len(s) check but we keep it anyway
   system("/usr/bin/pkill -9 say.py");
   if (pipe) {
      // this waits for say.py to exit, so be sure to kill it first
      pclose(pipe);
   }
   return NULL;
}

void say(const char *text) {
   if (pipe && (feof(pipe) || ferror(pipe))) {
      // not sure why say.py is going defunct - only started happening recently, and we recently fell back to flite
      pipe = closeSay(pipe);
      pipe = openSay();
   }
   if (pipe) {
      int value = fputs(text, pipe);
      if (value >= 0) {
         value = fputc('\n', pipe);
         if (value >= 0) {
            value = fflush(pipe);
            if (value >= 0) {
               return;
            }
         }
      }
      char *errorString = strerror(errno);
      cerr << errorString << endl;
   } else {
      pipe = openSay();
   }
   cerr << "SAY:\t\t" << text << endl;
}

int main() {
   for (int i = 0; i < 999; ++i) {
      std::cerr << __FILE__ << "\t" << __LINE__ << "\t's':\t" << 's' << "\n";
      say("sonar right");
      struct timespec timespec = {0, 1000000};
      nanosleep(&timespec, NULL);
   }
   struct timespec timespec = {99, 0};
   nanosleep(&timespec, NULL);
   return 0;
}

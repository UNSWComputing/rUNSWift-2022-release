//
// Created by jayen on 6/03/19.
//

#include "speech.hpp"

#include "utils/home_nao.hpp"

using namespace std;

Speech &Speech::instance() {
   static Speech instance;
   return instance;
}

FILE *openSay() {
   const char *command = "/home/nao/2.8/bin/flite";
   FILE       *pipe    = popen(command, "w");
   if (!pipe) {
      llog(ERROR) << "error starting " << command << "\n";
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

void Speech::say(const char *text) {
   if (pipe && (feof(pipe) || ferror(pipe))) {
      // not sure why say.py is going defunct - only started happening recently, and we recently fell back to flite
      pipe = closeSay(pipe);
      pipe = openSay();
   }
   if (pipe) {
      int value = fputs(text, pipe);
      if (value >= 0) {
         value = fputc('\n', pipe);
      }
      char *errorString = strerror(errno);
      llog(ERROR) << errorString << endl;
   } else {
      pipe = openSay();
   }
   cerr << "SAY:\t\t" << text << endl;
}

Speech::Speech() {
   pipe = openSay();
}

Speech::~Speech() {
   pipe = closeSay(pipe);
}

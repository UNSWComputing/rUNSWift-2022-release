#pragma once

#include <cstdio>
#include <iostream>
#include "utils/Logger.hpp"
#include "utils/Timer.hpp"
#include <errno.h>

class Speech {
public:
   static Speech &instance();

   void say(const char *text);

private:
   Speech();

   ~Speech();

   // Declare copy constructors privately and don't implement them
   // This is to ensure singleton class
   Speech(Speech const &copy);

   Speech &operator=(Speech const &copy);

   FILE *pipe;
};

inline void SAY(const std::string text) {
   (Speech::instance()).say(text.c_str());
}

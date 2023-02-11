
#include "ButtonPresses.hpp"
#include <cassert>

ButtonPresses::ButtonPresses() : value(NONE) {};

bool ButtonPresses::remove(Value test) {
   assert(test != NONE);
   if (value != test) return false;
   value = NONE;
   return true;
}

void ButtonPresses::operator=(int n) {
   assert(n >= NONE);
   assert(n < BUTTONS_TOTAL);
   value = static_cast<Value>(n);
}

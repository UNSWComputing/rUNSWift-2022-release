#pragma once

#include <stdint.h>
#include <ostream>

class ButtonPresses
{
public:
   enum Value //: uint8_t
   {
      NONE = 0,
      SINGLE_CHEST_TAP = 1,
      DOUBLE_CHEST_TAP = 2,
      TRIPLE_CHEST_TAP = 3,
      QUADRUPLE_CHEST_TAP = 4,
      BUTTONS_TOTAL // Must be last
   };

   ButtonPresses();

   operator Value() const { return value; };

   void operator=(int v);

   // If ButtonPresses equals v, returns true and sets ButtonPresses to NONE
   // Otherwise returns false
   bool remove(Value v);

private:
   Value value;
};

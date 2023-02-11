#pragma once

#include <iostream>

class ProtobufSerialisable {
  public:
    virtual void serialise(std::ostream& os) const = 0;
    virtual void deserialise(std::istream& is) = 0;
};

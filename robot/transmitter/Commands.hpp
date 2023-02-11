//
// Created by jayen on 11/11/19.
//

#ifndef RUNSWIFT_COMMANDS_HPP
#define RUNSWIFT_COMMANDS_HPP

#include <vector>

#include "transmitter/TransmitterDefs.hpp"
#include "utils/ProtobufSerialisable.hpp"

namespace offnao {
   class Commands;
}

class Commands : ProtobufSerialisable {
   public:
      // only for deserialisation
      Commands();

      explicit Commands(std::vector<std::string> argv);

      explicit Commands(OffNaoMask_t sendingMask);

      ~Commands();

      void serialise(std::ostream &os) const;

      void deserialise(std::istream &is);

      bool hasArgV() const;

      std::vector<std::string> getArgV() const;

      bool hasSendingMask() const;

      OffNaoMask_t getSendingMask() const;

   private:
      offnao::Commands *commands;
};

std::ostream &operator<<(std::ostream &out, const Commands &commands);

#endif //RUNSWIFT_COMMANDS_HPP

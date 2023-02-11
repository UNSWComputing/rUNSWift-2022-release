//
// Created by jayen on 12/11/19.
//

#include "transmitter/Commands.hpp"

#include <iterator>

#include "Commands.pb.h"

// only for deserialisation
Commands::Commands() : commands(new offnao::Commands()) {}

Commands::Commands(std::vector<std::string> argv) : commands(new offnao::Commands()) {
   commands->mutable_argv()->Reserve(argv.size());
   for (std::vector<std::string>::iterator iterator = argv.begin(); iterator != argv.end(); ++iterator) {
      commands->add_argv(*iterator);
   }
}

Commands::Commands(OffNaoMask_t sendingMask) : commands(new offnao::Commands()) {
   commands->set_sendingmask(sendingMask);
}

Commands::~Commands() {
   delete commands;
}

void Commands::serialise(std::ostream &os) const {
   // https://developers.google.com/protocol-buffers/docs/techniques#streaming
   unsigned int size = commands->ByteSize();
   os.write(reinterpret_cast<char *>(&size), sizeof(size));
   commands->SerializeToOstream(&os);
}

void Commands::deserialise(std::istream &is) {
   // https://developers.google.com/protocol-buffers/docs/techniques#streaming
   unsigned int size;
   is.read(reinterpret_cast<char *>(&size), sizeof(size));
   char *array = new char[size];
   is.read(array, size);
   commands->ParseFromArray(array, size);
}

bool Commands::hasArgV() const {
   return commands->argv_size();
}

std::vector<std::string> Commands::getArgV() const {
   std::vector<std::string>                              cpp;
   const google::protobuf::RepeatedPtrField<std::string> &pb  = commands->argv();
   int                                                   size = commands->argv_size();
   cpp.resize(static_cast<unsigned int>(size));
   for (int i = 0; i < size; ++i) {
      cpp[i] = pb.Get(i);
   }
   return cpp;
}

bool Commands::hasSendingMask() const {
   return commands->has_sendingmask();
}

OffNaoMask_t Commands::getSendingMask() const {
   return commands->sendingmask();
}

std::ostream &operator<<(std::ostream &out, const Commands &commands) {
   if (commands.hasSendingMask()) {
      out << commands.getSendingMask() << "\n";
   }
   if (commands.hasArgV()) {
      const std::vector<std::string> &argv = commands.getArgV();
      copy(argv.begin(), argv.end(), std::ostream_iterator<std::string>(out, "\n"));
   }
   return out;
}

//
// Created by jayen on 24/03/19.
//

#include "naoData.hpp"
#include "../../robot/naoData.pb.h"
#include "progopts.hpp"

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

using namespace std;
using namespace google::protobuf;

// not sure why i need to forward declare these, since they exist when the templated callers are instantiated
static void serialise(const Frame &, offnao::Frame &);

template<typename T, typename U>
static void serialise(const T *array, ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField, int size) {
   repeatedPtrField.Reserve(size);
   for (int i = 0; i < size; ++i) {
      U *u = repeatedPtrField.Add();
      serialise(array[i], *u);
   }
}

template<typename T, typename U>
static void serialise(const vector<T> &vector, ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField) {
   serialise(vector.data(), repeatedPtrField, vector.size());
}

static void serialise(const Frame &cpp, offnao::Frame &pb) {
   Blackboard::serialise(*cpp.blackboard, *pb.mutable_blackboard());
   pb.set_timestamp(cpp.timestamp);
}

// making this static creates a warning in the friend declaration.  there are some GCC pragmas to squelch the warning, but they don't work in g++...
void serialise(const NaoData &cpp, offnao::NaoData &pb) {
   serialise(cpp.frames, *pb.mutable_frames());
   pb.set_currentframe(cpp.currentFrame);
   pb.set_timerecorded(cpp.timeRecorded);
   pb.set_ispaused(cpp.isPaused);
}

void NaoData::serialise(ostream &os) const {
   offnao::NaoData naoData;
   ::serialise(*this, naoData);
   naoData.SerializeToOstream(&os);
}

// not sure why i need to forward declare these, since they exist when the templated callers are instantiated
static void deserialise(Frame &, const offnao::Frame &);

//
template<typename T>
static void deserialise(T &cpp, const T &pb) {
   cpp = pb;
}

static void deserialise(unsigned int &cpp, const int32 &pb) {
   cpp = (unsigned int) pb;
}

static void deserialise(time_t &cpp, const int32 &pb) {
   cpp = (time_t) pb;
}

template<typename T, typename U>
static void deserialise(T *array, const ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField, int size) {
   for (int i = 0; i < size; ++i) {
      deserialise(array[i], repeatedPtrField.Get(i));
   }
}

template<typename T, typename U>
static void deserialise(T *array, const ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField) {
   deserialise(array, repeatedPtrField, repeatedPtrField.size());
}

template<typename T, typename U>
static void deserialise(vector<T> &vector, const ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField) {
   vector.resize(static_cast<unsigned int>(repeatedPtrField.size()));
   deserialise(vector.data(), repeatedPtrField);
}

static void deserialise(Frame &cpp, const offnao::Frame &pb) {
   cpp.blackboard = new Blackboard(config);
   Blackboard::deserialise(*cpp.blackboard, pb.blackboard());
   deserialise(cpp.timestamp, pb.timestamp());
}

// making this static creates a warning in the friend declaration.  there are some GCC pragmas to squelch the warning, but they don't work in g++...
void deserialise(NaoData &cpp, const offnao::NaoData &pb) {
   deserialise(cpp.frames, pb.frames());
   deserialise(cpp.currentFrame, pb.currentframe());
   deserialise(cpp.timeRecorded, pb.timerecorded());
   deserialise(cpp.isPaused, pb.ispaused());
}

void NaoData::deserialise(istream &is) {
   offnao::NaoData naoData;

   // can't use ParseFromIstream because we need to set total bytes limit for large .ofn2's
   google::protobuf::io::IstreamInputStream zcis(&is);
   google::protobuf::io::CodedInputStream cis(&zcis);
   cis.SetTotalBytesLimit(INT_MAX, INT_MAX);
   naoData.ParseFromCodedStream(&cis);

   ::deserialise(*this, naoData);
}

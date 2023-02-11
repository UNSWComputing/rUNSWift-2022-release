//
// Created by jayen on 24/03/19.
//
#include "naoData.hpp"

#include <boost/serialization/vector.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

template<class Archive>
void NaoData::serialize(Archive &ar, const unsigned int file_version) {
   ar & frames;
   ar & currentFrame;
   ar & timeRecorded;
   ar & isPaused;
}

template
void NaoData::serialize(boost::archive::binary_iarchive &, const unsigned int);

template
void NaoData::serialize(boost::archive::binary_oarchive &, const unsigned int);

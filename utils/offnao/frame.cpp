//
// Created by jayen on 24/03/19.
//

#include "frame.hpp"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include "qt_serialisation/boostSerializationQPoint.hpp"
#include "progopts.hpp"
#include "blackboard/Blackboard.hpp"


template<class Archive>
void Frame::shallowSerialize(Archive &ar, const unsigned int file_version) {
      ar & timestamp;
}

template<class Archive>
void Frame::save(Archive & ar, const unsigned int version) const {
   ar & *blackboard;
   ((Frame*)this)->shallowSerialize(ar, version);
}

template
void Frame::save(boost::archive::binary_oarchive &, const unsigned int) const;

template<class Archive>
void Frame::load(Archive & ar, const unsigned int version) {
   blackboard = new Blackboard(config);
   ar & *blackboard;
   shallowSerialize(ar, version);

}

template
void Frame::load(boost::archive::binary_iarchive &, const unsigned int);

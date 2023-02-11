//
// Created by jayen on 12/04/19.
//
// So Vision.cpp doesn't have to include tiny-dnn
// _fwd_decl convention comes from msgpack

#ifndef RUNSWIFT_ROBOTDETECTOR_FWD_DECL_HPP
#define RUNSWIFT_ROBOTDETECTOR_FWD_DECL_HPP

#include "DetectorInterface.hpp"

Detector *newRobotDetector();
Detector *newSSRobotDetector();

#endif //RUNSWIFT_ROBOTDETECTOR_FWD_DECL_HPP

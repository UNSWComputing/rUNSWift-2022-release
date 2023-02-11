#pragma once

#include "utils/angles.hpp"
#include "types/Point.hpp"
#include "types/RRCoord.hpp"
#include "utils/basic_maths.hpp"
#include "utils/SPLDefs.hpp"
#include "types/boostSerializationEigenTypes.hpp"
#include <Eigen/Eigen>
#include <cmath>

struct AbsCoord {
   AbsCoord(float x, float y, float theta) : vec(x, y, theta) {
      var.setZero();
      var(0, 0) = SQUARE(FULL_FIELD_LENGTH);
      var(1, 1) = SQUARE(FULL_FIELD_WIDTH);
      var(2, 2) = SQUARE(M_PI);
      weight = 1.0;
   }

   AbsCoord() : vec(0, 0, 0) {
      var.setZero();
      var(0, 0) = SQUARE(FULL_FIELD_LENGTH);
      var(1, 1) = SQUARE(FULL_FIELD_WIDTH);
      var(2, 2) = SQUARE(M_PI);
      weight = 1.0;
   }

   Eigen::Vector3f vec;
   Eigen::Matrix<float, 3, 3> var;
   float weight;

   float x() const {
      return vec[0];
   }

   float y() const {
      return vec[1];
   }

   float theta() const {
      return vec[2];
   }

   float getVar(int m, int n) const {
      return var(m,n);
   }

   float distance() const {
      return hypotf(vec[0], vec[1]);
   }

   bool operator== (const AbsCoord &other) const {
      return vec == other.vec;
   }

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version) {
      ar & vec;
      ar & var;
      ar & weight;
   }

   /**
    * This assumes that the AbsCoord is already in robot relative coordinates, this will simply
    * convert it to polar coords.
    */
   RRCoord convertToRobotRelative(void) const {
      return RRCoord(sqrtf(vec.x()*vec.x() + vec.y()*vec.y()), atan2f(vec.y(), vec.x()), 0.0f);
   }

   RRCoord convertToRobotRelative(const AbsCoord &robotPose) const {
      float xdiff = x() - robotPose.x();
      float ydiff = y() - robotPose.y();
      float distance = sqrtf(xdiff*xdiff + ydiff*ydiff);

      float angle = atan2(ydiff, xdiff);
      float rrHeading = normaliseTheta(angle - robotPose.theta());

      return RRCoord(distance, rrHeading, 0.0f);
   }

   AbsCoord convertToRobotRelativeCartesian(const AbsCoord &robotPose) const {
      RRCoord rrCoord = convertToRobotRelative(robotPose);
      return AbsCoord(
            rrCoord.distance() * cos(rrCoord.heading()),
            rrCoord.distance() * sin(rrCoord.heading()),
            0.0f);
   }
};

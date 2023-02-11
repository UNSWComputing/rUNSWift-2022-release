#pragma once

#include <Eigen/Eigen>
#include <iostream>

typedef Eigen::Vector2i Point;
typedef Eigen::Vector2f PointF;

namespace boost {
   namespace serialization {

      template <class Archive>
      void serialize(Archive & ar, Point& p, const unsigned int version)
      {
         ar & p[0];
         ar & p[1];
      }

      template <class Archive>
      void serialize(Archive & ar, PointF& p, const unsigned int version)
      {
         ar & p[0];
         ar & p[1];
      }

   }
}

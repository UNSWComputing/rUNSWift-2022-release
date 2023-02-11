#ifndef BALL_INFO_HPP
#define BALL_INFO_HPP

#include "types/Point.hpp"
#include "types/RRCoord.hpp"
#include "types/XYZ_Coord.hpp"

#include <iostream>
#include <cmath>


struct BallInfo {
   BallInfo () {}
   BallInfo (RRCoord rr, int radius, Point imageCoords) :
      rr(rr),
      radius(radius),
      imageCoords(imageCoords){}

   virtual ~BallInfo () {}

   RRCoord rr;
   int radius;
   Point imageCoords;
   bool topCamera;

   bool operator== (const BallInfo &other) const
   {
      return rr           == other.rr
          && radius       == other.radius
          && imageCoords  == other.imageCoords
          && topCamera    == other.topCamera;
   }

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      ar & rr;
      ar & radius;
      ar & imageCoords;
      ar & topCamera;

      if (file_version >= 1 && file_version < 2) {
         float visionVar = 0;
         ar & visionVar;
      }

      if (file_version < 2)
      {
          int tmpLastSeen = 0;
          ar & tmpLastSeen;
          int tmpLifetime = 0;
          ar & tmpLifetime;
      }
   }
};

inline std::ostream &operator<<(std::ostream &os, const BallInfo &info) {
    os << "rr: " << info.rr << std::endl;
    os << "imageCoord: " << info.imageCoords << std::endl;
    os << "radius: " << info.radius << std::endl;
    return os;
}


struct BallHint
{

   enum Type
   {
      bLeft           = 0x00,
      bRight          = 0x01,
      bHidden         = 0x02,
      bNone           = 0x03
   };

   /* Names of corresponding enums from above */
   static const char *const TypeName[];

   BallHint () :
      type(bNone) {
   }

   BallHint (Type type) :
      type(type) {
   }

   virtual ~BallHint () {
   }

   Type type;

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version) {
      ar & type;
   }

};

#ifndef SWIG
BOOST_CLASS_VERSION(BallInfo, 2);
#endif

#endif

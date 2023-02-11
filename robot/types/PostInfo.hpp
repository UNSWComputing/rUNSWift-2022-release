#ifndef POST_INFO_HPP
#define POST_INFO_HPP

#include "types/Point.hpp"
#include "types/RRCoord.hpp"
#include "types/BBox.hpp"

#include <iostream>

struct PostInfo {

   enum Type
   {
      pNone        = 0x00,
      pLeft        = 0x01,
      pRight       = 0x02,
      pHome        = 0x04,
      pAway        = 0x08,
      pHomeLeft    = pLeft  | pHome,
      pHomeRight   = pRight | pHome,
      pAwayLeft    = pLeft  | pAway,
      pAwayRight   = pRight | pAway,
   };

   // If you are "to the left of" the post, the right post is further away
   // If you are "to the right of" the post, the left post is further away
   enum Direction
   {
      pToLeftOf,
      pToRightOf,
      pUnknown
   };

   /* Names of corresponding enums from above */
   static const char *const TypeName[];
   static const char *const DirName[];

   PostInfo () {}
   PostInfo (RRCoord rr, Type type, BBox imageCoords,
             float wDistance, float kDistance, bool trustDistance,
             Direction dir) :
      rr(rr),
      type(type),
      imageCoords(imageCoords),
      wDistance(wDistance),
      kDistance(kDistance),
      trustDistance(trustDistance),
      dir(dir) {}

   virtual ~PostInfo () {}

   RRCoord rr;
   Type type;
   BBox imageCoords;
   float wDistance;
   float kDistance;
   bool trustDistance;
   Direction dir;

   bool operator== (const PostInfo &other) const
   {
      return rr            == other.rr
          && type          == other.type
          && imageCoords   == other.imageCoords
          && wDistance     == other.wDistance
          && kDistance     == other.kDistance
          && trustDistance == other.trustDistance
          && dir           == other.dir;
   }


   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version) {
      ar & rr;
      ar & type;
      ar & imageCoords;
      ar & wDistance;
      ar & kDistance;
      ar & trustDistance;
      ar & dir;
   }
};

#endif

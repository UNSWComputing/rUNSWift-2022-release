#ifndef FIELD_FEATURE_INFO_HPP
#define FIELD_FEATURE_INFO_HPP

#ifndef Q_MOC_RUN
#include <boost/serialization/version.hpp>
#endif
#include <ostream>

#include "types/Point.hpp"
#include "types/RRCoord.hpp"
#include "utils/basic_maths.hpp"
#include "FieldFeatureInfoLegacy.hpp"

struct FieldFeatureInfo {

   enum Type
   {
      fNone           = 0x00,
      fLine           = 0x01,
      fCorner         = 0x02,
      fTJunction      = 0x03,
      fPenaltySpot    = 0x04,
      fCentreCircle   = 0x05,
      fFieldLinePoint = 0x06,
      fXJunction      = 0x07,
      fParallelLines  = 0x08,
      fGoalBoxCorner  = 0x09,
      NUMBER_OF_TYPES
   };

   /* Names of corresponding enums from above */
   static const char *const TypeName[NUMBER_OF_TYPES];

   FieldFeatureInfo (RRCoord rr, Type type) :
      rr(rr),
      type(type) {

      p1.setZero();
      p2.setZero();
   }

   FieldFeatureInfo () {
   }

   FieldFeatureInfo(const FieldFeatureInfo &other) {
      this->rr = other.rr;
      this->type = other.type;
      this->p1 = other.p1;
      this->p2 = other.p2;
   }

   virtual ~FieldFeatureInfo () {
   }

   RRCoord rr;
   Type type;
   /**
    * start and end points of a line, so we can see lines in offnao
    */
   Point p1;
   Point p2;

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version) {
      ar & rr;
      ar & type;

      if (file_version < 1)
      {
         if (type == fLine) {
            LineInfo tmpLine;
            bool tmpLineUsed = false;
            ar & tmpLine;
            ar & tmpLineUsed;
         } else if (type == fCorner) {
            CornerInfo tmpCorner;
            ar & tmpCorner;
         } else if (type == fTJunction) {
            TJunctionInfo tmpTJunction;
            ar & tmpTJunction;
         } else if (type == fPenaltySpot) {
            PenaltySpotInfo tmpPenaltyspot;
            ar & tmpPenaltyspot;
         } else if (type == fCentreCircle) {
            CentreCircleInfo tmpCentrecircle;
            ar & tmpCentrecircle;
         } else if (type == fFieldLinePoint) {
            FieldLinePointInfo tmpFieldlinepoints;
            ar & tmpFieldlinepoints;
         } else if (type == fXJunction) {
            XJunctionInfo tmpxjunction;
            ar & tmpxjunction;
         } else if (type == fParallelLines) {
            ParallelLinesInfo tmpparallellines;
            ar & tmpparallellines;
         } else if (type == fGoalBoxCorner){
            GoalBoxCornerInfo tmpgoal_box_corner;
            ar & tmpgoal_box_corner;
         }
      }
   }


};

inline std::ostream &operator<<(std::ostream &os, const FieldFeatureInfo &info)
{
    os << " type: " << FieldFeatureInfo::TypeName[info.type] << "\n rr: \n"
       << info.rr;
    return os;
}

BOOST_CLASS_VERSION(FieldFeatureInfo, 1);

#endif

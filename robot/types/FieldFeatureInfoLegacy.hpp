#ifndef FIELD_FEATURE_INFO_LEGACY_HPP
#define FIELD_FEATURE_INFO_LEGACY_HPP

#include <boost/serialization/version.hpp>

#include "types/Point.hpp"
#include "types/RRCoord.hpp"

struct FieldLinePointInfo {
   Point p, rrp;

   FieldLinePointInfo () {
   }
   FieldLinePointInfo (Point p, Point rrp)
      : p (p), rrp (rrp) {
   }


   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      if (file_version >= 4) {
         ar & p & rrp;
      }
   }
};

#ifndef SWIG
BOOST_CLASS_VERSION(FieldLinePointInfo, 4);
#endif

/* Line structure, given in terms of parameters
 * t1, t2 and t3, that satisfy the equation:
 * t1x + t2y + t3 = 0, each of quich are integral.
 **/
struct LineInfo {
   Point p1, p2;
   int t1, t2, t3;
   RRCoord rr;

   LineInfo () {
      p1 = Point(0,0);
      p2 = Point(0,0);
      t1 = 0;
      t2 = 0;
      t3 = 0;
      rr = RRCoord();
   }
   LineInfo (Point p1, Point p2, RRCoord rr = RRCoord(0,0))
      : p1(p1), p2(p2), rr(rr) {
      t1 = p1.y() - p2.y();
      t2 = p2.x() - p1.x();
      t3 = p1.x() * p2.y() - p2.x() * p1.y();
   }


   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      ar & p1 & p2;
      if (file_version >= 1) {
         ar & t1 & t2 & t3;
      }
      if (file_version >= 4) {
         ar & rr;
      }
   }
};

#ifndef SWIG
BOOST_CLASS_VERSION(LineInfo, 4);
#endif

struct CornerInfo
{
   // The main corner (kink) point.
   Point p;

   // The points that form the ends of the corner.
   // e1
   // |
   // |
   // p --- e2
   Point e1;
   Point e2;

   CornerInfo () {}
   CornerInfo (Point p, Point e1, Point e2) : p (p), e1 (e1), e2 (e2) {}

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      ar & p;
      ar & e1;
      ar & e2;
   }
};

struct TJunctionInfo
{
   Point p;

   TJunctionInfo() {}
   TJunctionInfo(Point p) : p (p) {}

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      if (file_version >= 3) {
         ar & p;
      }
   }
};

#ifndef SWIG
BOOST_CLASS_VERSION(TJunctionInfo, 3);
#endif

struct GoalBoxCornerInfo
{
   Point p;
   /* Left is from the orientation of the STRIKER */
   bool left_corner;

   GoalBoxCornerInfo() {}
   GoalBoxCornerInfo(Point p, bool left_corner) : p (p), left_corner (left_corner) {}

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      if (file_version >= 5) {
         ar & p;
         ar & left_corner;
      }
   }
};

#ifndef SWIG
BOOST_CLASS_VERSION(GoalBoxCornerInfo, 5);
#endif

struct PenaltySpotInfo
{
   Point p;
   int w, h;

   PenaltySpotInfo() {}
   PenaltySpotInfo(Point p) : p (p) {}
   PenaltySpotInfo(Point p, int w_, int h_) : p (p), w(w_), h(h_) {}

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      if (file_version >= 3) {
         ar & p;
      }
   }
};

#ifndef SWIG
BOOST_CLASS_VERSION(PenaltySpotInfo, 3);
#endif

struct XJunctionInfo
{
   Point p;

   XJunctionInfo() {}
   XJunctionInfo(Point p) : p (p) {}

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      if (file_version >= 3) {
         ar & p;
      }
   }
};

#ifndef SWIG
BOOST_CLASS_VERSION(XJunctionInfo, 3);
#endif
struct CentreCircleInfo
{
   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
   }
};

struct ParallelLinesInfo
{
   LineInfo l1, l2;

   ParallelLinesInfo() {}
   ParallelLinesInfo(LineInfo l1, LineInfo l2) : l1 (l1), l2 (l2) {}

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      if (file_version >= 3) {
         ar & l1 & l2;
      }
   }
};


#ifndef SWIG
BOOST_CLASS_VERSION(ParallelLinesInfo, 3);
#endif

#endif // FIELD_FEATURE_INFO_LEGACY_HPP
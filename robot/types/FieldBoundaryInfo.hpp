#pragma once

#include "types/RansacTypes.hpp"

/* 2010 code required two field edges to be written, one robot relative,
 * and one image relative for debugging. This is just a wrapper. Feel
 * free to change
 */
struct FieldBoundaryInfo
{
   FieldBoundaryInfo () {}
   FieldBoundaryInfo (RANSACLine rrBoundary, RANSACLine imageBoundary) :
      rrBoundary (rrBoundary),
      imageBoundary (imageBoundary) {}

   virtual ~FieldBoundaryInfo () {}

   RANSACLine rrBoundary;
   RANSACLine imageBoundary;

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version) {
      ar & rrBoundary;
      ar & imageBoundary;
   }
};

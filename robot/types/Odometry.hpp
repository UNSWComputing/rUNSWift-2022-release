#pragma once

#include <iostream>

struct Odometry {
   float forward;
   float left;
   float turn;

   Odometry(float f = 0.0f, float l = 0.0f, float t = 0.0f)
      : forward(f), left(l), turn(t) {}

   inline void clear() {
      forward = left = turn = 0.0f;
   }

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      ar & forward & left & turn;
   }
};

#ifndef SWIG
   static inline Odometry operator+(const Odometry& a, const Odometry& b) {
      Odometry c;
      c.forward = a.forward + b.forward;
      c.left = a.left + b.left;
      c.turn = a.turn + b.turn;
      return c;
   }

   static inline Odometry operator-(const Odometry& a, const Odometry& b) {
      Odometry c;
      c.forward = a.forward - b.forward;
      c.left = a.left - b.left;
      c.turn = a.turn - b.turn;
      return c;
   }
#endif

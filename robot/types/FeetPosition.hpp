#ifndef FEET_POSITION_HPP
#define FEET_POSITION_HPP

#include <iostream>

class FootPosition
{
  public:
    FootPosition(float x, float y, float theta): x(x), y(y), theta(theta){}
    FootPosition() : x(0), y(0), theta(0){}

    // Foot position, relative to torso frame
    float x;
    float y;
    float theta;

    friend std::ostream & operator << (std::ostream &out, const FootPosition &fp)
    {
        out << fp.x << ", " << fp.y << ", " << fp.theta;
        return out;
    }
};


class FeetPosition
{
  public:
    FeetPosition(float lx, float ly, float ltheta, float rx, float ry, float rtheta)
      : left(FootPosition(lx, ly, ltheta)), right(FootPosition(rx, ry, rtheta)){}
    FeetPosition(){}

    // Foot position for left and right foot
    FootPosition left;
    FootPosition right;

    friend std::ostream & operator << (std::ostream &out, const FeetPosition &fp)
    {
        out << "L: (" << fp.left << "), R: (" << fp.right << ")";
        return out;
    }
};

#endif // FEET_POSITION_HPP

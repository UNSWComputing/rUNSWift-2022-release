#ifndef FIELD_FEATURE_HPP
#define FIELD_FEATURE_HPP

class FieldFeature
{
  public:
    FieldFeature(double x, double y, double orientation)
        : x(x), y(y), orientation(orientation){};
    double x;
    double y;
    double orientation;
};

#endif // FIELD_FEATURE_HPP
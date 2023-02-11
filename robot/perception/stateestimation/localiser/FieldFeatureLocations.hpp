#ifndef FIELD_FEATURE_LOCATIONS_HPP
#define FIELD_FEATURE_LOCATIONS_HPP

#include <vector>
#include "types/RRCoord.hpp"
#include "types/AbsCoord.hpp"
#include "FieldFeature.hpp"

class FieldFeatureLocations
{
  public:

    FieldFeatureLocations();

    std::vector<FieldFeature> corners;
    std::vector<FieldFeature> t_junctions;
    std::vector<FieldFeature> centre_circles;
    std::vector<FieldFeature> penalty_spots;
    std::vector<float> constantXLines;
    std::vector<float> constantYLines;
};
#endif // FIELD_FEATURE_LOCATIONS_HPP

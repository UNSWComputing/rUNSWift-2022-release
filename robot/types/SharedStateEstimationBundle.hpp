#ifndef SHARED_STATE_ESTIMATION_BUNDLE_HPP
#define SHARED_STATE_ESTIMATION_BUNDLE_HPP

#include <Eigen/Eigen>
#include <vector>
#include "types/AbsCoord.hpp"

class SharedStateEstimationBundle
{
  public:
   SharedStateEstimationBundle() : haveBallUpdate(false) {}

    bool sanityCheck();

    template <class Archive>
    void serialize(Archive &ar, const unsigned int file_version)
    {
        ar & robotPos;
        ar & ballPosRRC;
        ar & ballVelRRC;
        ar & haveBallUpdate;
    }

    AbsCoord robotPos;
    AbsCoord ballPosRRC;
    AbsCoord ballVelRRC;
    bool haveBallUpdate;
};

#endif // SHARED_STATE_ESTIMATION_BUNDLE_HPP

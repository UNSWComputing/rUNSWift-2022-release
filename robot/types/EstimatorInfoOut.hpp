#ifndef ESTIMATOR_INFO_OUT_HPP
#define ESTIMATOR_INFO_OUT_HPP

#include "types/RobotObstacle.hpp"

class EstimatorInfoOut
{
public:
  EstimatorInfoOut(){};

  AbsCoord robotPos;
  float robotPosUncertainty;
  float robotHeadingUncertainty;
  std::vector<AbsCoord> allRobotPos;
  RRCoord ballPosRR;
  AbsCoord ballPosRRC;
  AbsCoord ballVelRRC;
  AbsCoord ballPos;
  AbsCoord ballVel;
  AbsCoord teamBallPos;
  AbsCoord teamBallVel;
  float teamBallPosUncertainty;
  SharedStateEstimationBundle sharedStateEstimationBundle;
  std::vector<RobotObstacle> robotObstacles;
  bool hadTeamBallUpdate;
};

#endif // ESTIMATOR_INFO_OUT_HPP

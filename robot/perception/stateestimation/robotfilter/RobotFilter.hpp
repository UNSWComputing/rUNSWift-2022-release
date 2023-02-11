#ifndef ROBOT_FILTER_HPP
#define ROBOT_FILTER_HPP

#include "perception/stateestimation/Estimator.hpp"
#include "types/RobotObstacle.hpp"
#include "types/GroupedRobots.hpp"

class EstimatorInfoInit;
class EstimatorInfoIn;
class EstimatorInfoMiddle;
class EstimatorInfoOut;

class RobotFilter : public Estimator
{
  public:
    RobotFilter(const EstimatorInfoInit &estimatorInfoInit);
    void tick(
        const EstimatorInfoIn &estimatorInfoIn,
        EstimatorInfoMiddle &estimatorInfoMiddle,
        EstimatorInfoOut &estimatorInfoOut);

  private:
    void update(
        const EstimatorInfoIn &estimatorInfoIn,
        const EstimatorInfoOut &estimatorInfoOut);
    std::vector<RobotObstacle> generateRobotObstacles() const;
    std::vector<GroupedRobots> groupedRobots;
};


#endif // ROBOT_FILTER_HPP
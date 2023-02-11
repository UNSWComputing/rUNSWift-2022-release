#include "RobotFilter.hpp"

#include "types/EstimatorInfoIn.hpp"
#include "types/EstimatorInfoOut.hpp"

#include <string>
#include <sstream>

const int NO_CLOSE_OBSERVATION = -1;

//Make my life easier for iterating over vectors
#define FOR_EACH(index, vector) for (unsigned int index = 0; index < vector.size(); ++index)


RobotFilter::RobotFilter(const EstimatorInfoInit &estimatorInfoInit)
    : Estimator(estimatorInfoInit)
{
}

void RobotFilter::tick(
    const EstimatorInfoIn &estimatorInfoIn,
    EstimatorInfoMiddle &estimatorInfoMiddle,
    EstimatorInfoOut &estimatorInfoOut)
{
    update(estimatorInfoIn, estimatorInfoOut);
    estimatorInfoOut.robotObstacles = generateRobotObstacles();
}

void RobotFilter::update(const EstimatorInfoIn &estimatorInfoIn, const EstimatorInfoOut &estimatorInfoOut)
{
    //Only update visual robots if not incapacitated
    if (!estimatorInfoIn.isIncapacitated) {

        std::vector<GroupedRobots>::iterator it = groupedRobots.begin();

        while (it != groupedRobots.end()) {
            GroupedRobots &group = (*it);
            group.tick(estimatorInfoIn.odometryDiff, estimatorInfoIn.headYaw, estimatorInfoOut.robotPos);
            if (group.isEmpty()) {
                it = groupedRobots.erase(it);
            } else {
                ++it;
            }
        }


        //Greedy algorithm to determine what observation goes into what group.
        //Current assumptions: multiple observations cannot go into the same group,
        //even if they are close together. The closest observation gets merge
        //into the group so there is many cases where this is suboptimal but as
        //this is a NP Complete problem it is good enough.
        std::vector<bool> observationMerged(estimatorInfoIn.visualRobots.size(), false);
        FOR_EACH(groupIndex, groupedRobots) {
            GroupedRobots &group = groupedRobots[groupIndex];

            int smallestIndex = NO_CLOSE_OBSERVATION;
            double smallestDistance = 0;
            FOR_EACH(visualIndex, estimatorInfoIn.visualRobots) {

                if (!observationMerged[visualIndex]) {

                    const RobotVisionInfo &visualRobot = estimatorInfoIn.visualRobots[visualIndex];
                    if (group.canMergeRobot(visualRobot)) {

                        double distance = group.distanceToRobot(visualRobot);
                        if (smallestIndex == -1 || smallestDistance > distance) {
                            smallestIndex = visualIndex;
                            smallestDistance = distance;
                        }
                    }
                }
            }

            if (smallestIndex != -1) {
                const RobotVisionInfo &observation = estimatorInfoIn.visualRobots[smallestIndex];
                group.mergeRobot(observation);
                observationMerged[smallestIndex] = true;
            }
        }

        FOR_EACH(visualIndex, estimatorInfoIn.visualRobots) {
            if (!observationMerged[visualIndex]) {
                GroupedRobots group(estimatorInfoIn.visualRobots[visualIndex]);
                groupedRobots.push_back(group);
            }
        }
    }
}

std::vector<RobotObstacle> RobotFilter::generateRobotObstacles() const
{
    std::vector<RobotObstacle> obstacles;

    FOR_EACH(groupedIndex, groupedRobots)
    {
        const GroupedRobots &group = groupedRobots[groupedIndex];
        if (group.isOnField() && group.isImportantObstacle())
        {
            std::string type;
            if (group.getType() == RobotVisionInfo::rUnknown)
            {
                type = "unknown";
            }
            else if (group.getType() == RobotVisionInfo::rBlue)
            {
                type = "blue";
            }
            else
            {
                type = "red";
            }
            obstacles.push_back(group.generateRobotObstacle());
        }
    }

    return obstacles;
}

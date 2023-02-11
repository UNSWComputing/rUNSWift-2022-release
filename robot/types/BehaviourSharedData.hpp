/*
* BehaviourSharedData.hpp
*
*  Created on: 15/05/2014
*      Author: osushkov
*/

#ifndef BEHAVIOUR_SHARED_DATA_HPP
#define BEHAVIOUR_SHARED_DATA_HPP

#include "types/boostSerializationEigenTypes.hpp"
#include "perception/behaviour/ReadySkillPositionAllocation.hpp"
#include <iostream>
#ifndef Q_MOC_RUN
#include <boost/serialization/version.hpp>
#endif

class BehaviourSharedData {
public:
    BehaviourSharedData();

    // Number of seconds since the robot last kicked the ball
    int secondsSinceLastKick;

    // The encoded enum value for the robots role
    int role;

    // Whether the robot is playing the ball or not
    bool playingBall;

    // Whether the robot needs assistance
    bool needAssistance;

    // Whether the robot is assisting
    bool isAssisting;

    bool isKickedOff;

    float walkingToX; // x of where a robot is walking to
    float walkingToY; // y of where a robot is walking to
    float walkingToH; // theta of where a robot is walking to

    // Notification to other robots that robot is about to kick the ball.
    bool kickNotification;

    bool sanityCheck()
    {
        // Check for secondsSinceLastKick nan
        if (std::isnan(secondsSinceLastKick)){
            std::cout << "received nan for secondsSinceLastKick" << std::endl;
            return false;
        }

        return true;
    }

    template<class Archive>
    void serialize(Archive &ar, const unsigned int file_version) {
        if (file_version > 5)
        {
          bool tmp = false;
          ar & tmp;
          ar & tmp;
        }
        if (file_version < 5)
        {
            float tmp;
            ar & tmp;
            ar & tmp;
            ar & tmp;
        }

        if (file_version < 5)
        {
            int tmp;
            ar & tmp;
        }

        if (file_version >= 1)
        {
            ar & role;
            ar & playingBall;
            ar & needAssistance;
            ar & isAssisting;
            ar & secondsSinceLastKick;
        }

        if (file_version >= 3)
        {
            ar & isKickedOff;
        }

        if (file_version < 5)
        {
           bool tmp = false;
           ar & tmp;
        }

        if (file_version < 5)
        {
           bool tmp = false;
           ar & tmp;
        }

        if (file_version < 5)
        {
            ReadySkillPositionAllocation tmp;
            ar & tmp;
        }

        if (file_version >= 4 && file_version < 5)
        {
            int tmp;
            ar & tmp;
        }
    }
};

BOOST_CLASS_VERSION(BehaviourSharedData, 5);

#endif // BEHAVIOUR_SHARED_DATA_HPP

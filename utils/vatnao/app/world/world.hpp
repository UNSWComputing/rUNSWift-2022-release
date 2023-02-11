#pragma once

#include "blackboard/Blackboard.hpp"
#include "record2Reader.hpp"

class World{
    public:
        World(Record2Reader *dump);

        // Move forward or backward the given number of frames
        // Return the actual number of frames moved foward of
        //  back, may be less than numFrames if there are not
        //  enough frames remaining or more than numFrames if
        //  several need to be skipped because of blank frames.
        int forward(int numFrames = 1);
        int back(int numFrames = 1);

        Blackboard *blackboard;

    private:
        Record2Reader *record2Reader;

        bool hasValidFrame;
};

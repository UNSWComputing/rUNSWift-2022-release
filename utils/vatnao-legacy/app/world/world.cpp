#include "world.hpp"
#include <iostream>

#include "../exceptions.hpp"

World::World(Record2Reader *dump){
    record2Reader = dump;
    blackboard = new Blackboard();
    hasValidFrame = false;
}

int World::forward(int numFrames){
    int movedFrames = 0;
    while(movedFrames < numFrames && record2Reader->currFrameIndex() < record2Reader->numFrames()) {
        // we need to make sure both our frames are not null, otherwise it'll crash when we
        //  try processing. Make sure the number of frames moved is recorded.
        do {
            record2Reader->writeNextFrame(blackboard);
            movedFrames++;
        } while((readFrom(vision, topFrame) == NULL || readFrom(vision, botFrame) == NULL) && record2Reader->isFinalFrame() == false);
    }
    if (record2Reader->isFinalFrame() && hasValidFrame == false) {
        throw NoRawImagesInDumpError();
    } else {
        hasValidFrame = true;
    }
    return movedFrames;
}

int World::back(int numFrames){
    int movedFrames = 0;
    while(movedFrames < numFrames && record2Reader->currFrameIndex() >= 0) {
        // we need to make sure both our frames are not null, otherwise it'll crash when we
        //  try processing. Make sure the number of frames moved is recorded.
        do {
            record2Reader->writePrevFrame(blackboard);
            movedFrames++;
        } while(readFrom(vision, topFrame) == NULL || readFrom(vision, botFrame) == NULL);
    }
    return movedFrames;
}


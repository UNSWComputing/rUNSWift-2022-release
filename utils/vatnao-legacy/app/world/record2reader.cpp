#include "record2reader.hpp"
#include <fstream>

using namespace std;

Record2Reader::Record2Reader(const std::string fileName): naoData() {
    // Treat this like offnao
    offNao = true;

    std::ifstream ifs;
    // catch if file not found
    ifs.exceptions(ifstream::badbit);
    ifs.open(fileName.c_str(), ios::in | ios::binary);

    std::cout << "Write Archive" << std::endl;
    naoData.deserialise(ifs);
    std::cout << "Archive fully Loaded" << std::endl;
    // NaoData will sometimes load input with the index on the last frame, set it to
    // 0 just to be useful.
    naoData.setCurrentFrame(0);
    std::cout << "CurrFrame: " << currFrameIndex() << "/" << numFrames() << std::endl;
}

void Record2Reader::writePrevFrame(Blackboard *blackboard){
    naoData.prevFrame();
    writeCurrFrame(blackboard);
}

void Record2Reader::writeNextFrame(Blackboard *blackboard){
    naoData.nextFrame();
    writeCurrFrame(blackboard);
}

void Record2Reader::writeCurrFrame(Blackboard *blackboard){
    Frame frame = naoData.getCurrentFrame();
    *blackboard = *frame.blackboard;
    std::cout << "Wrote Frame: " << currFrameIndex() << "/" << numFrames() << " to blackboard" << std::endl;
}

int Record2Reader::numFrames(){
    return naoData.getFramesTotal();
}

int Record2Reader::currFrameIndex(){
    return naoData.getCurrentFrameIndex();
}

bool Record2Reader::isFinalFrame() {
    return currFrameIndex() + 1 == numFrames();
}

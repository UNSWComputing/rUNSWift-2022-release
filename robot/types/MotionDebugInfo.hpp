#ifndef MOTION_DEBUG_INFO_HPP
#define MOTION_DEBUG_INFO_HPP

#include "types/FeetPosition.hpp"

class MotionDebugInfo
{
public:

    FeetPosition feetPosition;
    float x;
    float y;

    bool kickAborted;
    bool kickCompleted;

    // If overwriteKickLean is true kick lean will be set to kickLeanOverwrite.
    bool overwriteKickLean;
    float kickLeanOverwrite;

    MotionDebugInfo() : kickAborted(false), kickCompleted(false), overwriteKickLean(false) {};
};

#endif // MOTION_DEBUG_INFO_HPP

#pragma once

#include "types/Experiment.hpp"
#include "types/XYZ_Coord.hpp"
#include "types/AbsCoord.hpp"

class CleverNaoInfoMotion;

/*
The data CleverNao needs passed to other parts of the system.
*/
class CleverNaoInfoBehaviour
{

public:

    // The experiment currently being performed.
    Experiment* theExperiment;

    // Whether the nao should just behave normally (false) or take some
    // special CleverNao action.
    bool cleverNaoBehaviour;

    // Whether additional CleverNao related data should be recorded for later analysis.
    bool cleverNaoRecording;

    // Whether the nao should wait and do nothing while CleverNao thinks.
    bool wait;

    // Whether the nao should move to a specific location and facing, and
    // the position values of that location and facing. Units are mm.
    bool moveTo;
    Point naoPos;
    Point naoFacing;

    // Whether the nao should kick.
    bool kick;

    // Whether the nao should kick with its left foot. False means right foot.
    bool kickLeft;

    // Whether to overwrite the kick lean, and the kick lean to use.
    bool overwriteKickLean;
    float kickLeanOverwrite;

    // Whether gyro readings should be recorded for Clever Nao.
    bool recordGyroX;
    bool requestClearGyroXReadings;

    // Whether CoM calculations should be recorded for Clever Nao.
    bool recordCoM;
    bool requestClearCoMReadings;

    // The robot relative position of the ball.
    bool recordRRBallPos;
    std::vector<AbsCoord> rrBallPos;

    /*
    Creates an "empty" instance of CleverNaoInfoBehaviour.
    */
    CleverNaoInfoBehaviour() : cleverNaoBehaviour(false), wait(true), moveTo(false), kick(false), kickLeft(true),
        overwriteKickLean(false), recordGyroX(false), requestClearGyroXReadings(false), recordCoM(false),
                                                                  requestClearCoMReadings(false), recordRRBallPos(false)
    {
        rrBallPos.reserve(1000);
    }

    /*
    Set the nao to behave normally.
    */
    void setBehaviourNormal()
    {
        clearAllFlags();
    }

    /*
    Set the nao to wait for CleverNao.
    */
    void setBehaviourWait()
    {
        clearAllFlags();
        cleverNaoBehaviour = true;
        wait = true;
    }

    /*
    Set the nao to move to a specific position and facing (by the point it
    should face).
    */
    void setBehaviourMove(const Point &target, const Point &facingTarget)
    {
        clearAllFlags();
        cleverNaoBehaviour = true;
        moveTo = true;
        naoPos = target;
        naoFacing = facingTarget;
    }

    /*
    Set the nao to perform a single kick.
    */
    void setBehaviourKick()
    {
        clearAllFlags();
        cleverNaoBehaviour = true;
        kick = true;
    }

    /*
    Clears all flags so a new setting can be created.
    */
    void clearAllFlags()
    {
        cleverNaoBehaviour = false;
        wait = false;
        moveTo = false;
        kick = false;
    }

    /*
    Save the data recorded in CleverNaoInfoBehaviour.
    */
    void saveData(const std::string& bodyName, const CleverNaoInfoMotion& cleverNaoInfoMotion);

    /*
    Creates a directory to save the next set of data in and returns the path to that directory.
    */
    std::string getNextDataDirectory(const std::string& bodyName);

    /*
    Save the data recorded in CleverNaoInfoBehaviour to the directory provided.
    */
    void saveDataToDirectory(const CleverNaoInfoMotion& cleverNaoInfoMotion, const std::string& saveDir);
};

/*
The data CleverNao needs passed to other parts of the system.
*/
class CleverNaoInfoMotion
{
public:

    /*
    Creates an "empty" instance of CleverNaoInfoMotion.
    */
    CleverNaoInfoMotion() : kickCompleted(false), kickAborted(false)
    {
        gyroReadingsX.reserve(1000);
        CoM.reserve(1000);
    }

    // Whether the nao has just completed a kick, and whether that kick was aborted at some point during the motion.
    bool kickCompleted;
    bool kickAborted;

    // The gyro x readings recorded for clevernao.
    std::vector<float> gyroReadingsX;

    // The CoM calculations, recorded for Clever Nao.
    std::vector<XYZ_Coord> CoM;

    /*
    Clears all flags so a new setting can be created.
    */
    void clearAllFlags()
    {
        kickCompleted = false;
        kickAborted = false;
    }
};

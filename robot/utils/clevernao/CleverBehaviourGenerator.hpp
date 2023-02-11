#pragma once

#include "blackboard/Blackboard.hpp"

#include "utils/Timer.hpp"

#include "types/Experiment.hpp"

#include "utils/clevernao/CleverNaoDefines.hpp"

class CleverBehaviourGenerator
{

public:

    /*
    Initialise CleverBehaviourGenerator to the not standing state.
    */
    CleverBehaviourGenerator() : startedStanding(false),
                                                  experimentInProgress(false) {}

    /*
    Creates a behaviour corresponding to the next action the nao should take
    given the state of the experiment being performed. Returns true if the
    experiment is complete, false otherwise.
    */
    bool createBehaviourForExperiment(Blackboard &bb, Experiment*& waitingForExperiment,
                                                    const CleverNaoExperimentType experimentType, std::string& saveDir);

private:

    // Whether the nao has started standing.
    bool startedStanding;

    // Whether there is an experiment in progress.
    bool experimentInProgress;

    // Time since the nao started standing.
    Timer startedStandingTime;

    // Time since the last experiment finished.
    Timer timeSinceLastExperimentFinished;

    // Time since the last experiment started.
    Timer timeSinceLastExperimentStarted;
};

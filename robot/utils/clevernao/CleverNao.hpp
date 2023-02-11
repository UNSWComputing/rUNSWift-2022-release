#pragma once

#include <vector>
#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#endif

#include "blackboard/Blackboard.hpp"
#include "blackboard/Adapter.hpp"

#include "types/BehaviourRequest.hpp"
#include "types/Experiment.hpp"

#include "utils/clevernao/ResourceManager.hpp"
#include "utils/clevernao/CleverBehaviourGenerator.hpp"
#include "utils/clevernao/gaussianprocess/GPFunc/FlatFunc.hpp"

//#define CLEVER_NAO_VERIFY_RESULTS
#ifdef CLEVER_NAO_VERIFY_RESULTS
#define CLEVER_NAO_NUM_VERIFICATION_TESTS 10
#endif // CLEVER_NAO_VERIFY_RESULTS

/*
 * CleverNao is an active learning system. It attempts to learn to better use
 * the algorithms available to the Nao by strategically choosing experiments to
 * evaluate thier performance in various configurations.
 */
class CleverNao : Adapter {

public:
    /*
     * Constructor
     * The experiment and heuristic manager implementations define the behaviour
     * of CleverNao.
     */
    CleverNao(Blackboard *bb) : Adapter(bb),
                resourceManager(new FlatFunc<0>()),
                nextExperiment((Experiment*)new ExperimentKickLean()),
                currentPoint(KICK_LEAN_EXPERIMENT_EXPERIMENT_DIMS, 0.0f),
                worthwhileExperiments(true), curNumExperimentTypes(1), experimentType(EMPTY_EXPERIMENT),
                initialExperiments(false), notSaved(true)
#ifdef CLEVER_NAO_VERIFY_RESULTS
                , beganTesting(false), curVerification(1),
                verificationComplete(false)
#endif // CLEVER_NAO_VERIFY_RESULTS
    {
        nextExperiment->experimentType = EMPTY_EXPERIMENT;
        if(readFrom(behaviour, cleverNaoInfo.kickLeft))
            experimentType = KICK_LEAN_LEFT;
        else
            experimentType = KICK_LEAN_RIGHT;
    }

    /* Destructor */
    ~CleverNao();

    // Called per frame to run CleverNao and receive task instructions.
    void execute();

private:

    // The system used to generate behaviours used to control rUNSWift.
    CleverBehaviourGenerator behaviourGenerator;

    // The system used to manage resources by selecting optimal experiments to
    // perform.
    ResourceManager<KICK_LEAN_EXPERIMENT_EXPERIMENT_DIMS, KICK_LEAN_EXPERIMENT_SITUATION_DIMS,
                                                                KICK_LEAN_EXPERIMENT_HEURISTIC_DIMS, 1> resourceManager;

    // The experiment currently in progress.
    Experiment* nextExperiment;

    // The current position of the nao in terms of the experiment parameters.
    std::vector<float> currentPoint;

    // The current estimate for heuristic generation opportunity value.
    float heurGenValue;

    // The current estimate for experiment generation opportunity value.
    float expGenValue;

    // Whether we're done, and there are no more worthwhile experiments.
    bool worthwhileExperiments;

    // The type of experiment being performed.
    int curNumExperimentTypes;
    CleverNaoExperimentType experimentType;

    // Whether we're still performing the initial set of experiments.
    bool initialExperiments;

    // Whether we have saved.
    bool notSaved;

#ifdef CLEVER_NAO_VERIFY_RESULTS
    // Whether testing has not yet begun.
    bool beganTesting;

    // The ID of this verification test.
    int curVerification;

    // Whether all verification tests are complete.
    bool verificationComplete;
#endif // CLEVER_NAO_VERIFY_RESULTS
};

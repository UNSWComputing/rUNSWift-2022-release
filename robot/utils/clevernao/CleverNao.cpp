#include <iterator>
#include <algorithm>

#include <iostream>

#include "utils/clevernao/CleverNao.hpp"

/* Destructor */
CleverNao::~CleverNao()
{
    delete nextExperiment;
}

// Called per frame to run CleverNao and receive task instructions.
void CleverNao::execute()
{
    // If all worthwhile have not already been performed, continue.
    if(worthwhileExperiments)
    {
        // Whether the current experiment is finished and a new one is needed.
        bool experimentComplete;

        // The directory in which data is being saved.
        std::string saveDir;

        // Get the behaviour appropriate to the current experiment situation.
        experimentComplete = behaviourGenerator.createBehaviourForExperiment(*blackboard, nextExperiment,
                                                                                               experimentType, saveDir);

        // If the returned behaviour is NULL the experiment is complete.
        if(experimentComplete)
        {
            // The data that describes the new experiment.
            std::vector<std::vector<float> > newExperiments;
            std::vector<float> newExperiment;

            // Train the model with the experiment results.
            resourceManager.addTraining(nextExperiment->getCleverNaoFormatValues(), nextExperiment->reward, 0);

            // Choose a new experiment.
            if(!initialExperiments)
            {
                // The next situation to consider.
                std::vector<float> situation;

                // The value of the last experiment found.
                float lastOpportunityValue = 0;

                // If there are no target situations run the default version.
                worthwhileExperiments = false;
                if(nextExperiment->valueSituations.size() == 0)
                {
                    lastOpportunityValue = 0;
                    worthwhileExperiments |= resourceManager.selectExperiment(*blackboard, newExperiment, situation,
                                                                        lastOpportunityValue, currentPoint, saveDir, 0);
                    newExperiments.push_back(newExperiment);
                }

                // Otherwise examine each target situation separately.
                else
                {
                    for(unsigned int situationID=0; situationID<nextExperiment->valueSituations.size(); ++situationID)
                    {
                        std::cout << std::endl << "Situation " << situationID << std::endl;
                        situation = nextExperiment->valueSituations[situationID];
                        lastOpportunityValue = 0;
                        worthwhileExperiments |= resourceManager.selectExperiment(*blackboard, newExperiment, situation,
                                                                        lastOpportunityValue, currentPoint, saveDir, 0);
                        newExperiments.push_back(newExperiment);
                    }
                }
            }

            // If a worthwhile experiment was found, perform it.
            if(worthwhileExperiments)
            {
                nextExperiment->setExperimentFromCleverNaoFormatValues(newExperiment);
                behaviourGenerator.createBehaviourForExperiment(*blackboard, nextExperiment, experimentType, saveDir);
            }

            // Otherwise, training is complete.
            else
            {
                std::cout << "CleverNao is done training." << std::endl;
                nextExperiment->setLengthscales(resourceManager.getLengthscales());
                // TODO shutdown.
            }
        }
    }

#ifdef CLEVER_NAO_VERIFY_RESULTS
    // When done with experiments, measure the accuracy of the best point
    // chosen.
    else if(!verificationComplete)
    {
        // Whether the current verification experiment is finished and a new one
        // should be started.
        bool experimentComplete;

        // If testing has not been started grab the appropriate experiment.
        if(!beganTesting)
        {
            float bestPointValueEstimate;
            float temp = 0;
            std::vector<float> situation;
            std::vector<float> newExperiment;
            std::cout << "Begin CleverNao verification." << std::endl;

            // Set the experiment type.
            experimentType = KICK_LEAN_LEFT;

            // Get the best point.
            bestPointValueEstimate = resourceManager.getBest(*blackboard,
                                                newExperiment, situation, temp);

            std::cout << "Expected verification value: " <<
                                            bestPointValueEstimate << std::endl;

            // Set the system to use the best point.
            nextExperiment->setExperimentFromCleverNaoFormatValues(newExperiment);

            // Testing has begun.
            beganTesting = true;
        }

        // Run the test experiment.
        experimentComplete = behaviourGenerator.createBehaviourForExperiment(*blackboard, nextExperiment,
                                                                                                        experimentType);

        // When a verification experiment is completed the result should be
        // recorded.
        if(experimentComplete)
        {
            // Display the verification reward.
            std::cout << "Verification reward " << curVerification << ": " <<
                                      nextExperiment->reward << std::endl;

            // Set the experiment type.
            experimentType = KICK_LEAN_LEFT;

            // Get the follow up behaviour.
            behaviourGenerator.createBehaviourForExperiment(*blackboard, nextExperiment, experimentType);

            // Increment the number of verification tests completed.
            ++curVerification;
        }

        // Terminate when the desired number of tests are complete.
        if(curVerification == CLEVER_NAO_NUM_VERIFICATION_TESTS)
        {
            std::cout << "Clever Nao verification complete." << std::endl;
            verificationComplete = true;
        }
    }
#endif // VERIFY_RESULTS
    // All steps complete, save the results.
    else if(notSaved)
    {
        // Last opportunity value is set to default.
        float temp = 0;

        // Determine the best experiment.
        std::vector<float> situation;
        std::vector<float> bestPoint;
        std::vector<std::vector<float> > experimentResults;

        // Find the best points.
        for(unsigned int situationID=0; situationID<nextExperiment->valueSituations.size(); ++situationID)
        {
            situation = nextExperiment->valueSituations[situationID];
            resourceManager.getBest(*blackboard, bestPoint, situation, temp);
            experimentResults.push_back(bestPoint);
        }

        // Set the system to use the best point.
        nextExperiment->setExperimentFromCleverNaoFormatValues(experimentResults);

        // Save the best point.
        nextExperiment->save(blackboard->kinematics.bodyName);

        // Config saved.
        notSaved = false;
    }
}

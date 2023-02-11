#pragma once

#include "types/Point.hpp"

#include "utils/clevernao/CleverNaoDefines.hpp"
#include "utils/body.hpp"
#include "utils/ConfigUtils.hpp"
#include "utils/home_nao.hpp"

// Whether debug should be printed.
#define CLEVER_NAO_EXPERIMENT_DEBUG

/*
 * An experiment used by CleverNao. Defines an experiment used to assess the set
 * of heuristics evaluated by CleverNao.
 */
class Experiment
{
public:
    /*
    Creates an experiment with experiment variables uninitialised.
    */
    Experiment() : experimentType(KICK_LEAN_LEFT), state(0), reward(0.0f) {};

    /*
    Creates an experiment with experiment variables initialised from a resource manager experiment vector.
    */
    Experiment(CleverNaoExperimentType type, const std::vector<float> &newValues) : experimentType(type), state(0),
                                                                                                            reward(0.0f)
    {
        setExperimentFromCleverNaoFormatValues(newValues);
    }

    /*
    Returns the values stored in the experiment in the default Clever Nao format (values between 0 and 1).
    */
    inline const std::vector<float> getCleverNaoFormatValues() const
    {
        return(values[0]);
    }

    /*
    Gets the values stored in the experiment in the format required for use by rUNSWift.
    */
    std::vector<float> getRUNSWiftFormatValues() const
    {
        // The converted values.
        std::vector<float> convertedValues = values[0];

        // Make the conversion.
        convertToRUNSWiftFormatValues(convertedValues);

        // Return the values after conversion to rUNSWift format.
        return(convertedValues);
    }

    /*
    Gets the values stored in the experiment in the format required for use by runswift relevent to the given situation.
    */
    std::vector<float> getRUNSWiftFormatValues(std::vector<float> situation, bool rUNSWiftFormatSituation) const
    {
        if(rUNSWiftFormatSituation)
            convertToCleverNaoFormatValues(situation);
        return(getRUNSWiftFormatValues(situation));
    }

    /*
    Gets the values stored in the experiment in the format required for use by runswift relevent to the given situation.
    */
    std::vector<float> getRUNSWiftFormatValues(std::vector<float> situation) const
    {
        // The converted values relevant to the given situation.
        std::vector<float> convertedValues;

        // The closest situation's ID.
        int closestSituationID = 0;

        // The distance to the closest situation.
        float closestSituationDistance = std::numeric_limits<float>::max();

        // If there is only one relevant situation, just use it.
        if(values.size() == 1)
            return(getRUNSWiftFormatValues());

        // Otherwise find the closest situation.
        for(unsigned int situationID=0; situationID<values.size(); ++situationID)
        {
            float distance = getDistance(valueSituations[situationID], situation);
            if(distance < closestSituationDistance)
            {
                closestSituationID = situationID;
                closestSituationDistance = distance;
            }
        }

        // Return the values associated with the given situation in rUNSWift format.
        convertedValues = values[closestSituationID];

        convertToRUNSWiftFormatValues(convertedValues);
        return(convertedValues);
    }

    /*
    Sets the experiment's stored values from a set of values in Clever Nao format.
    */
    inline void setExperimentFromCleverNaoFormatValues(const std::vector<float>& newValues)
    {
        values.clear();
        values.push_back(std::vector<float>());
        values[0].reserve(newValues.size());
        for(unsigned int value=0; value<newValues.size(); ++value)
            values[0].push_back(newValues[value]);
    }

    /*
    Sets the experiment's stored values from a set of values in Clever Nao format.
    */
    inline void setExperimentFromCleverNaoFormatValues(const std::vector<std::vector<float> >& newValues)
    {
        values = newValues;
    }

    /*
    Sets the experiment's stored values from a set of values in rUNSWift format.
    */
    void setExperimentFromRUNSWiftFormatValues(const std::vector<float>& newValues)
    {
        // Make space for the new values.
        values.clear();
        values.push_back(std::vector<float>());
        values[0] = newValues;

        // Perform the conversion.
        convertToCleverNaoFormatValues(values[0]);
    }

    /*
    Calculates the squared distance between two situations, scaled by the lengthscales.
    */
    float getDistance(const std::vector<float>& a, const std::vector<float>& b) const
    {
        // The total squared distance.
        float squaredDistance = 0;

        // Sum the squared differences.
        for(unsigned int dim=0; dim<a.size(); ++dim)
            squaredDistance += pow(lengthscales[dim]*(a[dim]-b[dim]), 2);

        // Return the squared distance calculated.
        return(squaredDistance);
    }

    /*
    Sets the lengthscales to be used in situation comparisons.
    */
    void setLengthscales(Eigen::VectorXf newLengthscales)
    {
        // Populate lengthscales with the lengthscale values converted to a better format for this system.
        lengthscales.reserve(newLengthscales.size());
        for(int lengthscale=0; lengthscale<newLengthscales.size(); ++lengthscale)
            lengthscales[lengthscale] = pow(newLengthscales[lengthscale], 2);
    }

    /*
    Converts from Clever Nao format values to rUNSWift format values.
    */
    virtual void convertToRUNSWiftFormatValues(std::vector<float>& valuesToConvert) const=0;

    /*
    Converts from rUNSWift format values to Clever Nao format values.
    */
    virtual void convertToCleverNaoFormatValues(std::vector<float>& valuesToConvert) const=0;

    /*
    Saves the results of this experiment to the relevant configuration file.
    */
    virtual void save(std::string bodyName)=0;

    /*
    A virtual destructor is required for an interface to function correctly.
    */
    virtual ~Experiment() {};

    // The kind of experiment this is.
    CleverNaoExperimentType experimentType;

    // The state the experiment is currently in.
    int state;

    // The reward this experiment has accumulated.
    float reward;

    // The values being managed by Clever Nao.
    std::vector<std::vector<float> > values;

    // The set of situations an action should be selected for.
    std::vector<std::vector<float> > valueSituations;

    // The lengthscales to use when comparing situations.
    std::vector<float> lengthscales;
};

inline std::ostream& operator<<(std::ostream& os, const Experiment* experiment)
{
    std::vector<float> rUNSWiftFormatValues;
    os << "Clever Nao Format Values: ";
    for(unsigned int value=0; value<experiment->values.size(); ++value)
        os << experiment->values[0][value] << ", ";
    os << std::endl << std::endl;
    os << "rUNSWift Format Values: ";
    rUNSWiftFormatValues = experiment->getRUNSWiftFormatValues();
    for(unsigned int value=0; value<experiment->values.size(); ++value)
        os << rUNSWiftFormatValues[value] << ", ";
    os << std::endl << std::endl;
    os << "State: " << experiment->state << std::endl;
    os << "Reward: " << experiment->reward << std::endl;
    return os;
}

#define KICK_LEAN_EXPERIMENT_BALL_OFFSET_DIVISOR 100.0f
#define MAX_KICK_LEAN_EXPERIMENT 5.0f
#define KICK_LEAN_CONVERSION_EXPERIMENT (2.0f*MAX_KICK_LEAN_EXPERIMENT)

class ExperimentKickLean : public Experiment
{
public:

    /*
    Creates an experiment with experiment variables uninitialised.
    */
    ExperimentKickLean()
    {
        Experiment::experimentType = KICK_LEAN_LEFT;
        Experiment::state = 0;
        Experiment::reward = 0.0f;
    }

    /*
    Creates an experiment with experiment variables initialised from a resource manager experiment vector.
    */
    ExperimentKickLean(CleverNaoExperimentType type, const std::vector<float> &newValues)
    {
        Experiment::experimentType = type;
        Experiment::state = 0;
        Experiment::reward = 0.0f;
        setExperimentFromCleverNaoFormatValues(newValues);
    }

    /*
    Converts from Clever Nao format values to rUNSWift format values.
    */
    void convertToRUNSWiftFormatValues(std::vector<float>& valuesToConvert) const
    {
        // Convert situation dimensions.
        valuesToConvert[0] = KICK_LEAN_EXPERIMENT_BALL_OFFSET_DIVISOR * valuesToConvert[0];

        // Convert heuristic dimensions if they exist.
        if(valuesToConvert.size() > 1)
            valuesToConvert[1] = KICK_LEAN_CONVERSION_EXPERIMENT * (valuesToConvert[1]-0.5f);
    }

    /*
    Converts from rUNSWift format values to Clever Nao format values.
    */
    void convertToCleverNaoFormatValues(std::vector<float>& valuesToConvert) const
    {
        // Convert situation dimenstions.
        valuesToConvert[0] = valuesToConvert[0]/KICK_LEAN_EXPERIMENT_BALL_OFFSET_DIVISOR;

        // Convert heuristic dimensions if they exist.
        if(valuesToConvert.size() > 1)
            valuesToConvert[1] = (valuesToConvert[1]/KICK_LEAN_CONVERSION_EXPERIMENT)+0.5f;
    }

    /*
    Saves the results of this experiment to the relevant configuration file.
    */
    void save(std::string bodyName)
    {
        // The name of the target section.
        std::string targetSection = "kick";

        // The name of the target entry
        std::string targetEntry;

        // The values in runswift format.
        std::vector<float> rUNSWiftFormatValues = getRUNSWiftFormatValues();

        // Set the name of the target entry.
        if(Experiment::experimentType == KICK_LEAN_LEFT)
            targetEntry = "leanOffsetL=";
        else
            targetEntry = "leanOffsetR=";

        // String stream for converting float to string.        
        std::ostringstream convert;

        // Somewhat lazily just do the full search for every entry.
        for(int entry=-1; entry<(int)Experiment::valueSituations.size(); ++entry)
        {
            // Set the name and value of the target entry.
            switch(entry)
            {
            case -1:
                if(Experiment::experimentType == KICK_LEAN_LEFT)
                    targetEntry = "leanOffsetLVaried";
                else
                    targetEntry = "leanOffsetRVaried";
                break;
            case 0:
                if(Experiment::experimentType == KICK_LEAN_LEFT)
                    targetEntry = "leanOffsetLInner";
                else
                    targetEntry = "leanOffsetRInner";
                break;
            case 1:
                if(Experiment::experimentType == KICK_LEAN_LEFT)
                    targetEntry = "leanOffsetLMid";
                else
                    targetEntry = "leanOffsetRMid";
                break;
            case 2:
                if(Experiment::experimentType == KICK_LEAN_LEFT)
                    targetEntry = "leanOffsetLOuter";
                else
                    targetEntry = "leanOffsetROuter";
                break;
            }
            if(entry > -1)
                rUNSWiftFormatValues = getRUNSWiftFormatValues(Experiment::valueSituations[entry]);
            else
                rUNSWiftFormatValues[1] = 1;

            // Overwrite the old value.
            convert << rUNSWiftFormatValues[1];
            overwriteEntry(targetSection, targetEntry, convert.str(), true, bodyName);
            convert.str(std::string());
        }
    }
};

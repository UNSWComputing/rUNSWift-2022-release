#ifndef SIMULATION_FIELD_FEATURE_H_
#define SIMULATION_FIELD_FEATURE_H_

#include <cstring>

namespace Simulation
{
    class FieldFeature
    {
    public:
        enum Type
        {
            NO_TYPE,
            CORNER,
            T_JUNCTION,
            CENTRE_CIRCLE
        };

        Type type;
        float ffs[4];


        FieldFeature(Type t, float pos[4])
            : type(t)
        {
            memcpy(ffs, pos, 4*sizeof(float));
        }

        FieldFeature()
            : type(NO_TYPE)
        { 
            for (unsigned i = 0; i < 4; ++i) ffs[i] = 0.;
        }

    };
}

#endif   // SIMULATION_FIELD_FEATURE_H_

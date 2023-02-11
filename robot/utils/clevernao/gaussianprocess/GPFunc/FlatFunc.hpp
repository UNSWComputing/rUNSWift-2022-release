#pragma once

#include "GPFunc.hpp"

/*
A GPFunc that simply returns a constant value.
*/
template<int value>
class FlatFunc : public GPFunc
{

public:

    /*
    Returns the constant value associated with this function.
    */
    inline float getValue(const std::vector<float> &point) const
                                               {return(((float)value)/1000000);}

};

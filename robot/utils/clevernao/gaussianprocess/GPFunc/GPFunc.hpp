#pragma once

/*
Interface for classes that provide functions that act as the mean and covariance
for the Guassian Process.
*/
class GPFunc
{

public:

    /*
    Grabs the actual value of the function. Note that because this is a virtual
    function it can't be inlined and in general will be quite expensive.
    */
    virtual float getValue(const std::vector<float> &point) const = 0;

    virtual ~GPFunc() {};
};

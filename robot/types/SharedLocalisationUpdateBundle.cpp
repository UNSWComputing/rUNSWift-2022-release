#include "SharedLocalisationUpdateBundle.hpp"

bool SharedLocalisationUpdateBundle::sanityCheck()
{
    // Check for ballSeenFraction nan
    if (std::isnan(ballSeenFraction)){
        std::cout << "received nan for ballSeenFraction" << std::endl;
        return false;
    }

    // Check for nans in mean
    for (int i = 0; i < SHARED_DIM; i++) {
        if (std::isnan(sharedUpdateMean(i, 0))){
            std::cout << "received nan in sharedUpdateMean" << std::endl;
            return false;
        }
    }

    // Check for nans in covariance
    for (int row = 0; row < SHARED_DIM; row++) {
        for (int col = 0; col < SHARED_DIM; col++) {
            if (std::isnan(sharedUpdateCovariance(row, col))){
                std::cout << "received nan in sharedUpdateCovariance" << std::endl;
                return false;
            }
        }
    }

    // Check for sharedDx nan
    if (std::isnan(sharedDx)){
        std::cout << "received nan for sharedDx" << std::endl;
        return false;
    }

    // Check for sharedDy nan
    if (std::isnan(sharedDy)){
        std::cout << "received nan for sharedDy" << std::endl;
        return false;
    }

    // Check for sharedDh nan
    if (std::isnan(sharedDh)){
        std::cout << "received nan for sharedDh" << std::endl;
        return false;
    }

    // Check for sharedCovarianceDx nan
    if (std::isnan(sharedCovarianceDx)){
        std::cout << "received nan for sharedCovarianceDx" << std::endl;
        return false;
    }

    // Check for sharedCovarianceDy nan
    if (std::isnan(sharedCovarianceDy)){
        std::cout << "received nan for sharedCovarianceDy" << std::endl;
        return false;
    }

    // Check for sharedCovarianceDh nan
    if (std::isnan(sharedCovarianceDh)){
        std::cout << "received nan for sharedCovarianceDh" << std::endl;
        return false;
    }
    
    return true;
}

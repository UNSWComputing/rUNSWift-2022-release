#ifndef EIGEN_HELPERS
#define EIGEN_HELPERS

#include <Eigen/Core>
#include <iostream>

template<typename Derived>
bool check_finite(const Eigen::MatrixBase<Derived>& matrix, std::string location) {
    if(!matrix.allFinite()) {
        std::cerr << "Not all finite matrix in " << location << "\n" << matrix << std::endl;
        return false;
    }

    return true;
}

#endif // EIGEN_HELPERS

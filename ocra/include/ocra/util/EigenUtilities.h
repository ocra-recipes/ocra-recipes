#ifndef OCRA_UTIL_EIGEN_UTILITIES_H
#define OCRA_UTIL_EIGEN_UTILITIES_H

#include <Eigen/Dense>
#include <Eigen/Lgsm>
#include <vector>

namespace ocra {
    namespace util {


inline Eigen::Displacementd eigenVectorToDisplacementd(const Eigen::VectorXd& eigenVector)
{
    Eigen::VectorXd tmpVector = Eigen::VectorXd::Zero(7);
    tmpVector(3) = 1.0;

    if (eigenVector.rows()==3) {
        tmpVector.head(3) = eigenVector;
    }
    else if (eigenVector.rows()==7) {
        tmpVector = eigenVector;
    }

    return Eigen::Displacementd(tmpVector(0), tmpVector(1), tmpVector(2), tmpVector(3), tmpVector(4), tmpVector(5), tmpVector(6));
}

inline std::vector<Eigen::Displacementd> eigenVectorToDisplacementd(const std::vector<Eigen::VectorXd>& eigenVector)
{
    std::vector<Eigen::Displacementd> dispVec;
    for(int i=0; i<eigenVector.size(); i++)
    {
        dispVec.push_back(eigenVectorToDisplacementd(eigenVector[i]));
    }
    return dispVec;
}




    } // namespace util
} // namespace ocra
#endif // OCRA_UTIL_EIGEN_UTILITIES_H

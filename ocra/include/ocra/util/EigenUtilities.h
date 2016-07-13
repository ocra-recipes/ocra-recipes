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

    if (eigenVector.rows()<=3) {
        for (auto i = 0; i < eigenVector.rows(); ++i) {
            tmpVector(i) = eigenVector(i);
        }
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

inline Eigen::Twistd eigenVectorToTwistd(const Eigen::VectorXd& eigenVector, bool linearOnly=true, bool angularOnly=false)
{
    if (eigenVector.size()==6) {
        return Eigen::Twistd(eigenVector);
    } else if (eigenVector.size()==3) {
        if (linearOnly) {
            return Eigen::Twistd(0,0,0,eigenVector(0),eigenVector(1),eigenVector(2));
        }
        if (angularOnly) {
            return Eigen::Twistd(eigenVector(0),eigenVector(1),eigenVector(2),0,0,0);
        }
    }
    return Eigen::Twistd::Zero();
}

inline Eigen::Wrenchd eigenVectorToWrenchd(const Eigen::VectorXd& eigenVector, bool linearOnly=true, bool angularOnly=false)
{
    if (eigenVector.size()==6) {
        return Eigen::Wrenchd(eigenVector);
    } else if (eigenVector.size()==3) {
        if (linearOnly) {
            return Eigen::Wrenchd(0,0,0,eigenVector(0),eigenVector(1),eigenVector(2));
        }
        if (angularOnly) {
            return Eigen::Wrenchd(eigenVector(0),eigenVector(1),eigenVector(2),0,0,0);
        }
    }
    return Eigen::Wrenchd::Zero();
}




    } // namespace util
} // namespace ocra
#endif // OCRA_UTIL_EIGEN_UTILITIES_H

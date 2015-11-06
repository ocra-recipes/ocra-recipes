#ifndef wOcraLINEARINTERPOLATIONTRAJECTORY_H
#define wOcraLINEARINTERPOLATIONTRAJECTORY_H

#include "wocra/Trajectory/wOcraTrajectory.h"



namespace wocra
{

class wOcraLinearInterpolationTrajectory : public wocra::wOcraTrajectory {
    public:
        // wOcraLinearInterpolationTrajectory(Eigen::MatrixXd& waypoints, bool endsWithQuaternion = false);
        // wOcraLinearInterpolationTrajectory(const Eigen::VectorXd& startingVector, const Eigen::VectorXd& endingVector, bool endsWithQuaternion = false);
        // wOcraLinearInterpolationTrajectory(Eigen::Displacementd& startingDisplacement, Eigen::Displacementd& endingDisplacement, bool endsWithQuaternion = true);
        // wOcraLinearInterpolationTrajectory(Eigen::Rotation3d& startingOrientation, Eigen::Rotation3d& endingOrientation, bool endsWithQuaternion = true);



        // void getDesiredValues();
        //Eigen::VectorXd getDesiredValues(double time);
        Eigen::MatrixXd getDesiredValues(double time);

        // void getDesiredValues(double time, Eigen::Displacementd& desiredDisplacement);
        // void getDesiredValues(double time, Eigen::Rotation3d& desiredOrientation);

    protected:

        double t0;


};






} // end of namespace wocra
#endif // wOcraLINEARINTERPOLATIONTRAJECTORY_H

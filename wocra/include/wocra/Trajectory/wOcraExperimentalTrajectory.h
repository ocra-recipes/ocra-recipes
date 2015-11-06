#ifndef wOcraEXPERIMENTALTRAJECTORY_H
#define wOcraEXPERIMENTALTRAJECTORY_H

#include "wocra/Trajectory/wOcraTrajectory.h"



namespace wocra
{

class wOcraExperimentalTrajectory : public wocra::wOcraTrajectory {
    public:
        // wOcraExperimentalTrajectory(Eigen::MatrixXd& waypoints, bool endsWithQuaternion = false);
        // wOcraExperimentalTrajectory(const Eigen::VectorXd& startingVector, const Eigen::VectorXd& endingVector, bool endsWithQuaternion = false);
        // wOcraExperimentalTrajectory(Eigen::Displacementd& startingDisplacement, Eigen::Displacementd& endingDisplacement, bool endsWithQuaternion = true);
        // wOcraExperimentalTrajectory(Eigen::Rotation3d& startingOrientation, Eigen::Rotation3d& endingOrientation, bool endsWithQuaternion = true);




        Eigen::MatrixXd getDesiredValues(double time);
        void getDesiredValues(double time, Eigen::Displacementd& position, Eigen::Twistd& velocity, Eigen::Twistd& acceleration);




    protected:

        double t0;



};






} // end of namespace wocra
#endif // wOcraEXPERIMENTALTRAJECTORY_H

#ifndef wOcraMINIMUMJERKTRAJECTORY_H
#define wOcraMINIMUMJERKTRAJECTORY_H

#include "wocra/Trajectory/wOcraTrajectory.h"



namespace wocra
{

class wOcraMinimumJerkTrajectory : public wocra::wOcraTrajectory {
    public:
        // wOcraMinimumJerkTrajectory(Eigen::MatrixXd& waypoints, bool endsWithQuaternion = false);
        // wOcraMinimumJerkTrajectory(const Eigen::VectorXd& startingVector, const Eigen::VectorXd& endingVector, bool endsWithQuaternion = false);
        // wOcraMinimumJerkTrajectory(Eigen::Displacementd& startingDisplacement, Eigen::Displacementd& endingDisplacement, bool endsWithQuaternion = true);
        // wOcraMinimumJerkTrajectory(Eigen::Rotation3d& startingOrientation, Eigen::Rotation3d& endingOrientation, bool endsWithQuaternion = true);



        Eigen::MatrixXd getDesiredValues(double time);
        // void getDesiredValues(double time, Eigen::Displacementd& position, Eigen::Twistd& velocity, Eigen::Twistd& acceleration);




    protected:

        double t0;



};






} // end of namespace wocra
#endif // wOcraMINIMUMJERKTRAJECTORY_H

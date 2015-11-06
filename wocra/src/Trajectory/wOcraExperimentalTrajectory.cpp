
#include "wocra/Trajectory/wOcraExperimentalTrajectory.h"
#include <math.h>



namespace wocra
{

// wOcraExperimentalTrajectory::wOcraExperimentalTrajectory(Eigen::MatrixXd& _waypoints, bool _endsWithQuaternion):
//     wOcraTrajectory(_waypoints, _endsWithQuaternion){};
//
// wOcraExperimentalTrajectory::wOcraExperimentalTrajectory(const Eigen::VectorXd& _startingVector, const Eigen::VectorXd& _endingVector, bool _endsWithQuaternion):
//     wOcraTrajectory(_startingVector, _endingVector, _endsWithQuaternion){};
//
// wOcraExperimentalTrajectory::wOcraExperimentalTrajectory(Eigen::Displacementd& _startingDisplacement, Eigen::Displacementd& _endingDisplacement, bool _endsWithQuaternion):
//     wOcraTrajectory(_startingDisplacement, _endingDisplacement, _endsWithQuaternion){};
//
// wOcraExperimentalTrajectory::wOcraExperimentalTrajectory(Eigen::Rotation3d& _startingOrientation, Eigen::Rotation3d& _endingOrientation, bool _endsWithQuaternion):
//     wOcraTrajectory(_startingOrientation, _endingOrientation, _endsWithQuaternion){};
//

Eigen::MatrixXd wOcraExperimentalTrajectory::getDesiredValues(double _time)
{
    /**
    * For details on the analytical point to point min-jerk formulation see:
    * http://www.jneurosci.org/content/5/7/1688.full.pdf
    * http://shadmehrlab.org/book/minimum_jerk/minimumjerk.htm
    */
    if (startTrigger)
    {
        startTrigger = false;
        t0 = _time;
    }

    double t = _time - t0;

    Eigen::MatrixXd desiredValue = Eigen::MatrixXd::Zero(nDoF, TRAJ_DIM);
    if (t <= pointToPointDuration && currentWaypointIndex<nWaypoints-1)
    {
        double tau = t / pointToPointDuration;
        Eigen::VectorXd alpha = waypoints.col(currentWaypointIndex+1) - waypoints.col(currentWaypointIndex);

        desiredValue.col(POS_INDEX) = waypoints.col(currentWaypointIndex) + alpha * ( 10*pow(tau,3.0) - 15*pow(tau,4.0)  + 6*pow(tau,5.0)   );
        desiredValue.col(VEL_INDEX) = waypoints.col(currentWaypointIndex) + alpha * ( 30*pow(tau,2.0) - 60*pow(tau,3.0)  + 30*pow(tau,4.0)  );
        desiredValue.col(ACC_INDEX) = waypoints.col(currentWaypointIndex) + alpha * ( 60*pow(tau,1.0) - 180*pow(tau,2.0) + 120*pow(tau,3.0) );
    }
    else if (t > pointToPointDuration && currentWaypointIndex<nWaypoints-1)
    {
        startTrigger = true;
        currentWaypointIndex++;
        desiredValue.col(POS_INDEX) = waypoints.col(currentWaypointIndex);
    }
    else{
        desiredValue.col(POS_INDEX) = waypoints.col(nWaypoints-1);
        trajectoryFinished = true;
    }


    return desiredValue;
}

void wOcraExperimentalTrajectory::getDesiredValues(double _time, Eigen::Displacementd& _position, Eigen::Twistd& _velocity, Eigen::Twistd& _acceleration)
{


}









} //namespace wocra

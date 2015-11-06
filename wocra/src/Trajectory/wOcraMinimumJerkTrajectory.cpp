
#include "wocra/Trajectory/wOcraMinimumJerkTrajectory.h"
#include <math.h>



namespace wocra
{


Eigen::MatrixXd wOcraMinimumJerkTrajectory::getDesiredValues(double _time)
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

    Eigen::MatrixXd desiredValue = Eigen::MatrixXd::Zero(nDoF, TRAJ_DIM);

    double tau = (_time - t0) / pointToPointDuration;


    if ((tau <= TAU_MAX) && (currentWaypointIndex<(nWaypoints-1)))
    {

        if (nonRotationDof != 0)
        {
            Eigen::VectorXd alpha = waypoints.col(currentWaypointIndex+1) - waypoints.col(currentWaypointIndex);
            desiredValue.block(0,POS_INDEX,nonRotationDof,1) = waypoints.col(currentWaypointIndex) + alpha * ( 10*pow(tau,3.0) - 15*pow(tau,4.0)  + 6*pow(tau,5.0)   );
            desiredValue.block(0,VEL_INDEX,nonRotationDof,1) = Eigen::VectorXd::Zero(nDoF) + alpha * ( 30*pow(tau,2.0) - 60*pow(tau,3.0)  + 30*pow(tau,4.0)  );
            desiredValue.block(0,ACC_INDEX,nonRotationDof,1) = Eigen::VectorXd::Zero(nDoF) + alpha * ( 60*pow(tau,1.0) - 180*pow(tau,2.0) + 120*pow(tau,3.0) );
        }
        if (endsWithQuaternion)
        {
            Eigen::Rotation3d qStart, qEnd;
            eigenVectorToQuaternion(waypoints.block((nDoF-QUATERNION_DIM),(currentWaypointIndex), QUATERNION_DIM, 1),  qStart);
            eigenVectorToQuaternion(waypoints.block((nDoF-QUATERNION_DIM),(currentWaypointIndex+1), QUATERNION_DIM, 1),  qEnd);

            Eigen::Rotation3d interpolatedQuat = quaternionSlerp(tau, qStart, qEnd);

            Eigen::VectorXd interpolatedQuatVector = quaternionToEigenVector(interpolatedQuat);
            desiredValue.block((nDoF-QUATERNION_DIM),POS_INDEX,QUATERNION_DIM,1) = interpolatedQuatVector;
        }
        // Eigen::VectorXd alpha = waypoints.col(currentWaypointIndex+1) - waypoints.col(currentWaypointIndex);

        // desiredValue.col(POS_INDEX) = waypoints.col(currentWaypointIndex) + alpha * ( 10*pow(tau,3.0) - 15*pow(tau,4.0)  + 6*pow(tau,5.0)   );
        // desiredValue.col(VEL_INDEX) = waypoints.col(currentWaypointIndex) + alpha * ( 30*pow(tau,2.0) - 60*pow(tau,3.0)  + 30*pow(tau,4.0)  );
        // desiredValue.col(ACC_INDEX) = waypoints.col(currentWaypointIndex) + alpha * ( 60*pow(tau,1.0) - 180*pow(tau,2.0) + 120*pow(tau,3.0) );
    }
    else if ((tau > TAU_MAX) && (currentWaypointIndex<(nWaypoints-1)))
    {
        startTrigger = true;
        currentWaypointIndex++;
        desiredValue.col(POS_INDEX) = waypoints.col(currentWaypointIndex);
        if (currentWaypointIndex<nWaypoints-1) {
            pointToPointDuration = pointToPointDurationVector(currentWaypointIndex);
        }
    }
    else{
        desiredValue.col(POS_INDEX) = waypoints.col(nWaypoints-1);
        trajectoryFinished = true;
    }

    // std::cout<<"Test";
    // dumpToFile(desiredValue);
    return desiredValue;
}

// void wOcraMinimumJerkTrajectory::getDesiredValues(double _time, Eigen::Displacementd& _position, Eigen::Twistd& _velocity, Eigen::Twistd& _acceleration)
// {


// }









} //namespace wocra


#include "wocra/Trajectory/wOcraLinearInterpolationTrajectory.h"


namespace wocra
{

// wOcraLinearInterpolationTrajectory::wOcraLinearInterpolationTrajectory(Eigen::MatrixXd& _waypoints, bool _endsWithQuaternion):
//     wOcraTrajectory(_waypoints, _endsWithQuaternion){};
//
// wOcraLinearInterpolationTrajectory::wOcraLinearInterpolationTrajectory(const Eigen::VectorXd& _startingVector, const Eigen::VectorXd& _endingVector, bool _endsWithQuaternion):
//     wOcraTrajectory(_startingVector, _endingVector, _endsWithQuaternion){};
//
// wOcraLinearInterpolationTrajectory::wOcraLinearInterpolationTrajectory(Eigen::Displacementd& _startingDisplacement, Eigen::Displacementd& _endingDisplacement, bool _endsWithQuaternion):
//     wOcraTrajectory(_startingDisplacement, _endingDisplacement, _endsWithQuaternion){};
//
// wOcraLinearInterpolationTrajectory::wOcraLinearInterpolationTrajectory(Eigen::Rotation3d& _startingOrientation, Eigen::Rotation3d& _endingOrientation, bool _endsWithQuaternion):
//     wOcraTrajectory(_startingOrientation, _endingOrientation, _endsWithQuaternion){};





// void wOcraLinearInterpolationTrajectory::getDesiredValues()
// {
//     std::cout << "\n--> Warning, empty method. Please call one of the following:\n" << std::endl;
//     std::cout << "----> Eigen::VectorXd wOcraLinearInterpolationTrajectory::getDesiredValues(double _time)" << std::endl;
//     std::cout << "----> Eigen::Displacementd wOcraLinearInterpolationTrajectory::getDesiredValues(double _time, bool _asDisplacement)" << std::endl;
// }


Eigen::MatrixXd wOcraLinearInterpolationTrajectory::getDesiredValues(double _time)
{
    if (startTrigger)
    {
        startTrigger = false;
        t0 = _time;
    }

    Eigen::MatrixXd desiredValue = Eigen::MatrixXd::Zero(nDoF,TRAJ_DIM);

    double tau = (_time - t0) / pointToPointDuration;


    if ((tau <= TAU_MAX) && (currentWaypointIndex<(nWaypoints-1)))
    {
        // Interpolate
        if (nonRotationDof != 0)
        {
            desiredValue.block(0,POS_INDEX,nonRotationDof,1) = tau * (waypoints.block(0,(currentWaypointIndex+1),nonRotationDof,1) - waypoints.block(0,currentWaypointIndex,nonRotationDof,1)) + waypoints.block(0,currentWaypointIndex,nonRotationDof,1);
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
    }
    else if ((tau > TAU_MAX) && (currentWaypointIndex<(nWaypoints-1)))
    {
        // Set to the next waypoint when pointToPointDuration achieved
        startTrigger = true;
        currentWaypointIndex++;
        desiredValue.col(POS_INDEX) = waypoints.col(currentWaypointIndex);
    }
    else{
        // Set to final waypoint if no more waypoints and duration achieved
        desiredValue.col(POS_INDEX) = waypoints.col(nWaypoints-1);
        trajectoryFinished = true;
    }



    return desiredValue;
}

// void wOcraLinearInterpolationTrajectory::getDesiredValues(double _time, Eigen::Displacementd& _desiredDisplacement)
// {
//     Eigen::MatrixXd desVals = wOcraLinearInterpolationTrajectory::getDesiredValues(_time);

//     eigenVectorToDisplacement(desVals.col(POS_INDEX), _desiredDisplacement);
// }

// void wOcraLinearInterpolationTrajectory::getDesiredValues(double _time, Eigen::Rotation3d& _desiredOrientation)
// {
//     Eigen::MatrixXd desVals = wOcraLinearInterpolationTrajectory::getDesiredValues(_time);

//     eigenVectorToQuaternion(desVals.col(POS_INDEX), _desiredOrientation);
// }








} //namespace wocra

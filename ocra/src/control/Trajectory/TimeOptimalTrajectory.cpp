#include "ocra/control/Trajectory/TimeOptimalTrajectory.h"
#include <math.h>

namespace ocra
{

TimeOptimalTrajectory::~TimeOptimalTrajectory()
{
    delete gt_trajectory;
}


Eigen::MatrixXd TimeOptimalTrajectory::getDesiredValues(double _time)
{
    if (startTrigger)
    {
        startTrigger = false;
        t0 = _time;
    }

    Eigen::MatrixXd desiredValue = Eigen::MatrixXd::Zero(nDoF, TRAJ_DIM);

    double t = _time - t0;

    if (t <= duration) {
        desiredValue.col(POS_INDEX) = gt_trajectory->getPosition(t);
        desiredValue.col(VEL_INDEX) = gt_trajectory->getVelocity(t);
    } else {
        desiredValue.col(POS_INDEX) = gt_trajectory->getPosition(duration);
        desiredValue.col(VEL_INDEX) = gt_trajectory->getVelocity(duration);
    }
    return desiredValue;
}

void TimeOptimalTrajectory::initializeTrajectory()
{
    /* For reaching */
    // maximumVelocityVector = Eigen::VectorXd::Constant(nDoF, 0.05);
	// maximumAccelerationVector = Eigen::VectorXd::Constant(nDoF, 0.05);


    /* For standing up */
    maximumVelocityVector = Eigen::VectorXd::Constant(nDoF, 0.1);
    maximumAccelerationVector = Eigen::VectorXd::Constant(nDoF, 0.1);
    maxDeviation = 0.1;
    timeStep = 0.01;

    recalculateTrajectory();
}

double TimeOptimalTrajectory::getDuration()
{
    return gt_trajectory->getDuration();
}

void TimeOptimalTrajectory::recalculateTrajectory()
{
    gttraj::Path path = gttraj::Path(waypointList, maxDeviation);
    gt_trajectory = new gttraj::Trajectory(path, maximumVelocityVector, maximumAccelerationVector, timeStep);
    if(gt_trajectory->isValid()) {
		duration = this->getDuration();
        // std::cout << "Time-optimal duration: " << duration << "(sec)" << std::endl;
    }
	else {
		OCRA_WARNING("Trajectory generation failed.")
	}
}



} //namespace ocra

#include "ocra/control/Trajectory/TimeOptimalTrajectory.h"
#include <math.h>

namespace ocra
{

TimeOptimalTrajectory::~TimeOptimalTrajectory()
{
    delete trajectory;
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
        desiredValue.col(POS_INDEX) = trajectory->getPosition(t);
        desiredValue.col(VEL_INDEX) = trajectory->getVelocity(t);
    } else {
        desiredValue.col(POS_INDEX) = trajectory->getPosition(duration);
        desiredValue.col(VEL_INDEX) = trajectory->getVelocity(duration);
    }
    return desiredValue;
}

void TimeOptimalTrajectory::initializeTrajectory()
{
    Eigen::VectorXd maxAcceleration = Eigen::VectorXd::Ones(nDoF);
	Eigen::VectorXd maxVelocity = Eigen::VectorXd::Ones(nDoF);

    maxDeviation = 0.1;
    timeStep = 0.01;
    gttraj::Path path = gttraj::Path(waypointList, maxDeviation);
	trajectory = new gttraj::Trajectory(path, maxVelocity, maxAcceleration, timeStep);
	if(trajectory->isValid()) {
		duration = trajectory->getDuration();
        std::cout << "Time-optimal duration: " << duration << "(sec)" << std::endl;
    }
	else {
		std::cout << "Trajectory generation failed." << std::endl;
	}
}


} //namespace ocra

#include "ocra/control/Trajectory/TimeOptimalTrajectory.h"
#include <math.h>

namespace ocra
{

Eigen::MatrixXd TimeOptimalTrajectory::getDesiredValues(double _time)
{

    Eigen::MatrixXd desiredValue = Eigen::MatrixXd::Zero(nDoF, TRAJ_DIM);

    

    return desiredValue;
}

} //namespace ocra

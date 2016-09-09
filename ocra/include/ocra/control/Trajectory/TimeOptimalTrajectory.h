#ifndef TIME_OPTIMAL_TRAJECTORY_H
#define TIME_OPTIMAL_TRAJECTORY_H

#include "ocra/control/Trajectory/Trajectory.h"
#include "gttraj/Path.h"
#include "gttraj/Trajectory.h"

namespace ocra
{

class TimeOptimalTrajectory : public Trajectory {
    public:
        Eigen::MatrixXd getDesiredValues(double time);
    protected:
        double t0;
        gttraj::Trajectory trajectory;

};

} // end of namespace ocra
#endif // TIME_OPTIMAL_TRAJECTORY_H

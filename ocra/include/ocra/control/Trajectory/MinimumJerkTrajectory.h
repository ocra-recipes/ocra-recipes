#ifndef MINIMUMJERKTRAJECTORY_H
#define MINIMUMJERKTRAJECTORY_H

#include "ocra/control/Trajectory/Trajectory.h"

namespace ocra
{

class MinimumJerkTrajectory : public Trajectory
{
DEFINE_CLASS_POINTER_TYPEDEFS(MinimumJerkTrajectory)
    public:
        Eigen::MatrixXd getDesiredValues(double time);
    protected:
        double t0;
};

} // end of namespace ocra
#endif // MINIMUMJERKTRAJECTORY_H

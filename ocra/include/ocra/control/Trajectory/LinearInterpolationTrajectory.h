#ifndef LINEARINTERPOLATIONTRAJECTORY_H
#define LINEARINTERPOLATIONTRAJECTORY_H

#include "ocra/control/Trajectory/Trajectory.h"

namespace ocra
{

class LinearInterpolationTrajectory : public Trajectory
{
DEFINE_CLASS_POINTER_TYPEDEFS(LinearInterpolationTrajectory)

    public:
        Eigen::MatrixXd getDesiredValues(double time);

    protected:
        double t0;
};

} // end of namespace ocra
#endif // LINEARINTERPOLATIONTRAJECTORY_H

#ifndef TIME_OPTIMAL_TRAJECTORY_H
#define TIME_OPTIMAL_TRAJECTORY_H

#include "ocra/control/Trajectory/Trajectory.h"
#include "gttraj/Path.h"
#include "gttraj/Trajectory.h"

namespace ocra
{

class TimeOptimalTrajectory : public Trajectory
{
DEFINE_CLASS_POINTER_TYPEDEFS(TimeOptimalTrajectory)

    public:
        virtual ~TimeOptimalTrajectory();

        Eigen::MatrixXd getDesiredValues(double time);
        virtual double getDuration();
        virtual void recalculateTrajectory();


    protected:
        virtual void initializeTrajectory();

        double t0;
        double maxDeviation;
        double duration;
        double timeStep;
        gttraj::Trajectory* gt_trajectory;

};

} // end of namespace ocra
#endif // TIME_OPTIMAL_TRAJECTORY_H

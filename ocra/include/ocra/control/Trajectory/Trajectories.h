/**
*   \brief A simple header file which includes all of the different trajectory types in one place.
*
*/

#ifndef OCRA_TRAJECTORIES_H
#define OCRA_TRAJECTORIES_H

#include "ocra/control/Trajectory/Trajectory.h"
#include "ocra/control/Trajectory/MinimumJerkTrajectory.h"
#include "ocra/control/Trajectory/LinearInterpolationTrajectory.h"

#if USING_SMLT
#include "ocra/control/Trajectory/GaussianProcessTrajectory.h"
#endif

#if USING_GTTRAJ
#include "ocra/control/Trajectory/TimeOptimalTrajectory.h"
#endif

// Not sure if this will be junked eventually.
// #include "ocra/control/Trajectory/ExperimentalTrajectory.h"

#endif // OCRA_TRAJECTORIES_H

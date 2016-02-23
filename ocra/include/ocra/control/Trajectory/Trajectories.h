/**
*   \brief A simple header file which includes all of the different trajectory types in one place.
*
*/

#ifndef WOCRATRAJECTORIES_H
#define WOCRATRAJECTORIES_H

#include "ocra/control/Trajectory/Trajectory.h"
#include "ocra/control/Trajectory/MinimumJerkTrajectory.h"
#include "ocra/control/Trajectory/LinearInterpolationTrajectory.h"

#if USING_SMLT
#include "ocra/control/Trajectory/GaussianProcessTrajectory.h"
#endif

// Not sure if this will be junked eventually.
// #include "ocra/control/Trajectory/ExperimentalTrajectory.h"

#endif // WOCRATRAJECTORIES_H

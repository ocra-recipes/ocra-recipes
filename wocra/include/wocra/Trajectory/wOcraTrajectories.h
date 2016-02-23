/**
*   \brief A simple header file which includes all of the different trajectory types in one place.
*
*/

#ifndef WOCRATRAJECTORIES_H
#define WOCRATRAJECTORIES_H

#include "wocra/Trajectory/wOcraTrajectory.h"
#include "wocra/Trajectory/wOcraMinimumJerkTrajectory.h"
#include "wocra/Trajectory/wOcraLinearInterpolationTrajectory.h"

#if USING_SMLT
#include "wocra/Trajectory/wOcraGaussianProcessTrajectory.h"
#endif

// Not sure if this will be junked eventually.
// #include "wocra/Trajectory/wOcraExperimentalTrajectory.h"

#endif // WOCRATRAJECTORIES_H

#ifndef wOcraMINIMUMJERKTRAJECTORY_H
#define wOcraMINIMUMJERKTRAJECTORY_H

#include "wocra/Trajectory/wOcraTrajectory.h"

namespace wocra
{

class wOcraMinimumJerkTrajectory : public wocra::wOcraTrajectory {
    public:
        Eigen::MatrixXd getDesiredValues(double time);
    protected:
        double t0;
};

} // end of namespace wocra
#endif // wOcraMINIMUMJERKTRAJECTORY_H

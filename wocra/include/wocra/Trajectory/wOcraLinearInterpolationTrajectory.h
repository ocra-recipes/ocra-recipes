#ifndef wOcraLINEARINTERPOLATIONTRAJECTORY_H
#define wOcraLINEARINTERPOLATIONTRAJECTORY_H

#include "wocra/Trajectory/wOcraTrajectory.h"

namespace wocra
{

class wOcraLinearInterpolationTrajectory : public wocra::wOcraTrajectory {
    public:
        Eigen::MatrixXd getDesiredValues(double time);

    protected:
        double t0;
};

} // end of namespace wocra
#endif // wOcraLINEARINTERPOLATIONTRAJECTORY_H

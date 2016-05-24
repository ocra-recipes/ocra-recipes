#ifndef FC_QUADRATIC_FUNCTION_H
#define FC_QUADRATIC_FUNCTION_H


#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/SquaredLinearFunction.h"

/** \brief QuadraticFunction dedicated to the minimization of the force of contact (fc) variable.

 *

 */
namespace ocra
{

class FcQuadraticFunction : public ocra::QuadraticFunction
{
public:

    FcQuadraticFunction(ocra::Variable& x);

    virtual ~FcQuadraticFunction();

    void doUpdateInputSizeBegin();

    void updateHessian() const;

    void updateq() const;

    void updater() const;

};

}
#endif

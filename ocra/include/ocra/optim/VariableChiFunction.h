#ifndef VARIABLE_CHI_FUNCTION_H
#define VARIABLE_CHI_FUNCTION_H

#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/SquaredLinearFunction.h"

namespace ocra
{

class VariableChiFunction : public LinearFunction
{
public:
    VariableChiFunction(Variable& x, int dimension);

    void doUpdateInputSizeBegin();

    void doUpdateInputSizeEnd();
};

}
#endif

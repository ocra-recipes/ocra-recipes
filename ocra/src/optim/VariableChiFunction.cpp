#include "ocra/optim/VariableChiFunction.h"

using namespace ocra;

VariableChiFunction::VariableChiFunction(Variable& x, int dimension)
: NamedInstance("Variable Chi Linear Function")
, AbilitySet(PARTIAL_X)
, CoupledInputOutputSize(false)
, LinearFunction(x, dimension)
{

}

void VariableChiFunction::doUpdateInputSizeBegin()
{

}

void VariableChiFunction::doUpdateInputSizeEnd()
{

}

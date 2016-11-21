#include "ocra/optim/FcQuadraticFunction.h"


using namespace ocra;

FcQuadraticFunction::FcQuadraticFunction(ocra::Variable& x)
: NamedInstance("Variable Fc Quadratic Function")
, ocra::AbilitySet(ocra::PARTIAL_X, ocra::PARTIAL_XX)
, CoupledInputOutputSize(false)
, QuadraticFunction(x)
{

}

FcQuadraticFunction::~FcQuadraticFunction()
{

}

void FcQuadraticFunction::doUpdateInputSizeBegin()
{

}

void FcQuadraticFunction::updateHessian() const
{
    IFunction<ocra::PARTIAL_XX>::_val[0]->setIdentity(x.getSize(), x.getSize());
}

void FcQuadraticFunction::updateq() const
{
    _q[0]->setZero(x.getSize());
}

void FcQuadraticFunction::updater() const
{
    _r[0] = 0;
}

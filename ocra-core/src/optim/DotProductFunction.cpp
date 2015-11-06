#if 0

#include "ocra/optim/DotProductFunction.h"
#include "ocra/optim/FunctionProperties.h"

#include <stdexcept>

using namespace xde;

namespace ocra
{
    //TODO [todo]
/*  DotProductFunction::DotProductFunction(Function *f1, Function *f2)
    :Function(*FunctionProperties::getConcatenatedVariable(*f1, *f2), 1, 
    L_UNDEFINED, C_UNDEFINED,
    FunctionProperties::getAddContinuityProperty(*f1,*f2), 
    FunctionProperties::computeHessian(*f1,*f2), FunctionProperties::computeGradient(*f1,*f2)), 
    _f1(f1), _f2(f2), _twoFunctions(true), _tmp(f->getVariable().getSize())
  {
    //TODO [todo] : check size
    _f1->attach(*this);
    _f2->attach(*this);
  }*/

  DotProductFunction::DotProductFunction(Function* f, const Vector& v)
    :Function(f->getVariable(), 1, L_UNDEFINED, C_UNDEFINED,
    f->getContinuityProperty(), f->canComputeHessian(), f->canComputeGradient())
    , _f1(f), _v(v), _f2(NULL), _twoFunctions(false), _tmp(f->getVariable().getSize())
  {
    assert(_f1->getDimension()==_v.getSize());
    _f1->attach(*this);
  }


  void DotProductFunction::changeV(const Vector& v)
  {
    if (_twoFunctions)
      throw std::runtime_error("[ocra::SubtractionFunction::changeV]: no vector involved in this DotProductFunction");
    else
      _v.copyValuesFrom(v);
    invalidate();
  }


  void DotProductFunction::computeValue(void)  const
  {
    if (_twoFunctions)
      _value[0] = CML_dot(_f1->getValues(), _f2->getValues());
    else
      _value[0] = CML_dot(_f1->getValues(), _v);
  }


  void DotProductFunction::computeGradient(void)  const
  {
    //_tmp.setToZero();
    if (_twoFunctions)
    {
      CML_gemv<'t'>(1., _f1->getGradients(), _f2->getValues(), 0., _tmp);
      CML_gemv<'t'>(1., _f2->getGradients(), _f1->getValues(), 1., _tmp);
    }
    else
      CML_gemv<'t'>(1., _f1->getGradients(), _v, 0., _tmp);

    for (cfl_size_t i=0; i<_tmp.getSize(); ++i)
      _gradient(0,i) = _tmp[i];
  }

  void DotProductFunction::computeHessian(void)  const
  {
    //TODO [mineur] : might be optimizable
    if (_twoFunctions)
    {
      //TODO [todo]
    }
    else
    {
      (const_cast<Matrix*>(_hessians[0]))->copyValuesFrom(*_f1->getHessian(0));
      (const_cast<Matrix*>(_hessians[0]))->scalarMultInPlace(_v[0]);
      for (cfl_size_t i=1; i<_f1->getDimension(); ++i)
      {
        for (cfl_size_t j=0; j<_x->getSize(); ++j)
        {
          VectorWrap hessianColumn = ( (const_cast<Matrix*>(_hessians[0])) )->getColumnAsVector(j);
          CML_axpy(_v[i], _f1->getHessian(i)->getColumnAsVector(j), hessianColumn);
        }
      }
    }
  }

  void DotProductFunction::computeJdot(void) const
  {
    if (_twoFunctions)
    {
      //TODO [todo]
    }
    else
      CML_gemv<'t'>(1., _f1->getJdot(), _v, 0., _tmp);

    for (cfl_size_t i=0; i<_tmp.getSize(); ++i)
      _Jdot(0,i) = _tmp[i];
  }

  void DotProductFunction::computeJdotXdot(void) const
  {
    //this can be optimized with eigen : we're doing a dot product but getJdot gives a 1*n matrix
    _JdotXdot[0] = 0;
    for (cfl_size_t i=0; i<_tmp.getSize(); ++i)
      _JdotXdot[0] += getJdot()(0,i)*(*_x->getTimeDerivative())[i];
  }


  void DotProductFunction::doUpdateSize(void)
  {
  }

  void DotProductFunction::initHessian(void)
  {
    _hessians[0] = new Matrix(_x->getSize(), _x->getSize());
  }
}

#endif 

// cmake:sourcegroup=toBeUpdated


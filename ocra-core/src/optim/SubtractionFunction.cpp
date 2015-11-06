#if 0

#include "ocra/optim/SubtractionFunction.h"
#include "ocra/optim/FunctionProperties.h"

#include <stdexcept>


namespace ocra
{
  // TODO [todo] : 
/*  SubtractionFunction::SubtractionFunction(Function* f1, Function* f2)
    : Function(*FunctionProperties::getConcatenatedVariable(*f1, *f2), f1->getDimension(),
    FunctionProperties::getAddType(*f1,*f2), C_UNDEFINED,
    FunctionProperties::getAddContinuityProperty(*f1,*f2), 
    FunctionProperties::computeHessian(*f1,*f2), FunctionProperties::computeGradient(*f1,*f2))
    , _f1(f1), _f2(f2), _case(0)
  {
    //TODO [todo] : size checks
    _f1->attach(*this);
    _f2->attach(*this);
  }*/

  SubtractionFunction::SubtractionFunction(Function* f, const Vector& v)
    : Function(f->getVariable(), f->getDimension(), f->getType(), f->getConvexityProperty(),
    f->getContinuityProperty(), f->canComputeHessian(), f->canComputeGradient())
    , _f1(f), _v2(v), _f2(NULL), _case(1)
  {
    CML_scal(-1., _v2);
    _f1->attach(*this);
  }

  SubtractionFunction::SubtractionFunction(const Vector& v, Function* f)
    : Function(f->getVariable(), f->getDimension(), f->getType(), FunctionProperties::getOppositeConvexityProperty(*f),
    f->getContinuityProperty(), f->canComputeHessian(), f->canComputeGradient())
    , _f2(f), _v1(v), _f1(NULL), _case(2)
  {
    _f2->attach(*this);
  }

  SubtractionFunction::~SubtractionFunction()
  {
    //TODO [todo] : delete if no other function use _x
/*    if (_f1 == NULL || _f2 == NULL)
      delete _x;
*/      
    switch (_case)
    {
      case 0:
        _f1->detach(*this);
        _f2->detach(*this);
        break;
      case 1:
        _f1->detach(*this);
        break;
      case 2:
        _f2->detach(*this);
        break;
    }
  }


  void SubtractionFunction::changeV(const Vector& v)
  {
    switch (_case)
    {
      case 0:
        throw std::runtime_error("[ocra::SubtractionFunction::changeV]: no vector involved in this SubtractionFunction");
        break;
      case 1:
        _v2.copyValuesFrom(v);
        CML_scal(-1., _v2);
        break;
      case 2:
        _v1.copyValuesFrom(v);
        break;
    }
    invalidate();
  }



  void SubtractionFunction::computeValue(void) const
  {
    switch (_case)
    {
      case 0:
        _value.copyValuesFrom(_f1->getValues());
        CML_axpy(-1., _f2->getValues(), _value);
        break;
      case 1:
        _value.copyValuesFrom(_v2); //assume _v2 contains -v2
        CML_axpy(1., _f1->getValues(), _value);
        break;
      case 2:
        _value.copyValuesFrom(_v1);
        CML_axpy(-1., _f2->getValues(), _value);
        break;
    }
  }


  void SubtractionFunction::computeGradient(void) const
  {
    switch (_case)
    {
      case 0:
        _gradient.copyValuesFrom(_f1->getGradients());
        for (cfl_size_t i=0; i<_gradient.get_ncols(); ++i)
        {
          VectorWrap gradientColumn = _gradient.getColumnAsVector(i);
          CML_axpy(-1., _f2->getGradients().getColumnAsVector(i), gradientColumn);
        }
        break;
      case 1:
        _gradient.copyValuesFrom(_f1->getGradients());
        break;
      case 2:
        _gradient.copyValuesFrom(_f2->getGradients());
        _gradient.scalarMultInPlace(-1.);
        break;
    }
  }


  void SubtractionFunction::computeHessian(void) const
  {
    switch (_case)
    {
      case 0:
        for (cfl_size_t i=0; i<_dimension; ++i)
        {
          (const_cast<Matrix*>(_hessians[i]))->copyValuesFrom(*_f1->getHessian(i));
          for (cfl_size_t j=0; j<_gradient.get_ncols(); ++j)
          {
            VectorWrap f2HessianColumn = (const_cast<Matrix*>(_f2->getHessian(i)))->getColumnAsVector(j);
            VectorWrap hessianIColumn = (const_cast<Matrix*>(_hessians[i]))->getColumnAsVector(j);
            CML_axpy(-1., f2HessianColumn, hessianIColumn);
          }
        }
        break;
      case 1:
        for (cfl_size_t i=0; i<_dimension; ++i)
          (const_cast<Matrix*>(_hessians[i]))->copyValuesFrom(*_f1->getHessian(i));
        break;
      case 2:
        for (cfl_size_t i=0; i<_dimension; ++i)
        {
          (const_cast<Matrix*>(_hessians[i]))->copyValuesFrom(*_f2->getHessian(i));
          (const_cast<Matrix*>(_hessians[i]))->scalarMultInPlace(-1.);
        }
        break;
    }
  }


  void SubtractionFunction::computeJdot(void) const
  {
    switch (_case)
    {
      case 0:
        _Jdot.copyValuesFrom(_f1->getJdot());
        for (cfl_size_t i=0; i<_Jdot.get_ncols(); ++i)
        {
          VectorWrap gradientColumn = _Jdot.getColumnAsVector(i);
          CML_axpy(-1., _f2->getJdot().getColumnAsVector(i), gradientColumn);
        }
        break;
      case 1:
        _Jdot.copyValuesFrom(_f1->getJdot());
        break;
      case 2:
        _Jdot.copyValuesFrom(_f2->getJdot());
        _Jdot.scalarMultInPlace(-1.);
        break;
    }
  }


  void SubtractionFunction::computeJdotXdot(void) const
  {
    switch (_case)
    {
      case 0:
        _JdotXdot.copyValuesFrom(_f1->getJdotXdot());
        CML_axpy(-1., _f2->getJdotXdot(), _JdotXdot);
        break;
      case 1:
        _JdotXdot.copyValuesFrom(_v2); //assume _v2 contains -v2
        CML_axpy(1., _f1->getJdotXdot(), _JdotXdot);
        break;
      case 2:
        _JdotXdot.copyValuesFrom(_v1);
        CML_axpy(-1., _f2->getJdotXdot(), _JdotXdot);
        break;
    }
  }


  void SubtractionFunction::doUpdateSize(void)
  {
  }


  void SubtractionFunction::initHessian(void)
  {
    for (cfl_size_t i=0; i<_dimension; ++i)
      _hessians[i] = new Matrix(_x->getSize(), _x->getSize());
  }
}

#endif

// cmake:sourcegroup=toBeUpdated


#include "ocra/optim/Function.h"
#include <algorithm>

namespace ocra
{
  Function::Function(Variable& x, int dimension,  eFunctionLinearity linearity, eFunctionConvexity convexity,
                     int continuity, bool explicitlyTimeDependant, bool separableTimeDependancy)
    : ObserverSubject()
    , NamedInstance("function")
    , AbilitySet(FUN_VALUE)
    , CoupledInputOutputSize(false)
    , IFunctionProperties(linearity, convexity, continuity, explicitlyTimeDependant, separableTimeDependancy)
    , OCRA_FUNCTION_INTERFACE_INITIALIZE(getUsageSet())
    , x(x)
    , _value(IFunction<FUN_VALUE>::_val), _jacobian(IFunction<PARTIAL_X>::_val)
    , _dimension(dimension)
    , _dim(_dimension)
    , _hasDisconnected(false)
  {
    x.connect<EVT_RESIZE>(*this, &Function::updateInputSize);
    x.connect<EVT_CHANGE_VALUE>(*this, &Function::invalidateAll);

    resize();
  }


  Function::~Function()
  {
  }

  void Function::disconnectVariable() 
  {
    if(! _hasDisconnected) 
    {
      _hasDisconnected = true;
      x.disconnect<EVT_RESIZE>(*this, &Function::updateInputSize);
      x.disconnect<EVT_CHANGE_VALUE>(*this, &Function::invalidateAll);
    }
  }

  int Function::getDimension() const
  {
    return _dimension;
  }

  const Variable& Function::getVariable() const
  {
    return x;
  }

  Variable& Function::getVariable()
  {
    return x;
  }

  void Function::changeFunctionDimension(int newDimension)
  {
    changeFunctionDimensionImpl(newDimension);
    SubjectBase<EVT_RESIZE>::propagate();
  }

  void Function::changeFunctionDimensionImpl(int newDimension)
  {
    doUpdateDimensionBegin(newDimension);
    int oldDimension = _dimension;
    _dimension = newDimension;
    resize();
    doUpdateDimensionEnd(oldDimension);
  }

  void Function::updateInputSize(int timestamp)
  {
    doUpdateInputSizeBegin();

    if (!inputAndOutputSizesAreCoupled())
      stopPropagation<EVT_RESIZE>();
    else
      changeFunctionDimensionImpl(computeDimensionFromInputSize());

    resize();
    doUpdateInputSizeEnd();

    ocra_assert(!(OCRA_FUNCTION_INTERFACE_LIST( , ,::isValid() ||) false)); //we test that all values have been invalidated
  }

  void Function::resize()
  {
    const int m = _dimension;
    const int n = x.getSize();
    OCRA_APPLY_FUNCTION_ON_ALL_INTERFACE(resizeData(m,n));
  }

  void Function::doUpdateInputSizeBegin()
  {
    throw std::runtime_error("[Function::doUpdateInputSizeBegin]: by default, the function variable is not resizable. doUpdateInputSizeBegin must be overloaded if the function accept variable resizing.");
  }

  void Function::doUpdateInputSizeEnd()
  {
    //do nothing
  }

  int Function::computeDimensionFromInputSize() const
  {
    ocra_assert(false && "This should never happen, this function has to be overloaded in contexts where it might be called");
    return 0;
  }

  void Function::doUpdateDimensionBegin(int newDimension)
  {
    throw std::runtime_error("[Function::doUpdateDimensionBegin]: by default, a function is not resizable. doUpdateDimensionBegin must be overloaded if the function dimension can be changed.");
  }

  void Function::doUpdateDimensionEnd(int oldDimension)
  {
    //do nothing
  }

  void Function::updateFdot() const
  {
    ocra_assert(IFunction<PARTIAL_X>::canBeComputed());
    ocra_assert(!isExplicitlyTimeDependant() || IFunction<PARTIAL_T>::canBeComputed());
    ocra_assert(x.hasTimeDerivative());

    IFunction<FUN_DOT>::_val = getJacobian()*(static_cast<VectorXd>(x.getTimeDerivative()));
    if (isExplicitlyTimeDependant())
      IFunction<FUN_DOT>::_val += get<PARTIAL_T>();
  }
  
  void Function::updateFddot() const
  {
    ocra_assert(x.hasTimeDerivative());
    ocra_assert(x.getTimeDerivative().hasTimeDerivative());
    ocra_assert(IFunction<PARTIAL_X>::canBeComputed());
    ocra_assert(IFunction<PARTIAL_X_DOT>::canBeComputed());
    ocra_assert(!isExplicitlyTimeDependant() || IFunction<PARTIAL_XT>::canBeComputed());
    ocra_assert(!isExplicitlyTimeDependant() || IFunction<PARTIAL_TT>::canBeComputed());
    Variable& x_dot = x.getTimeDerivative();
    IFunction<FUN_DDOT>::_val = getJacobian()*static_cast<VectorXd>(x_dot.getTimeDerivative()) 
                                + get<PARTIAL_X_DOT>()*static_cast<VectorXd>(x_dot);
    if (isExplicitlyTimeDependant())
      IFunction<FUN_DDOT>::_val += get<PARTIAL_XT>()*static_cast<VectorXd>(x_dot) + get<PARTIAL_TT>();
  }

  void Function::updateJdotXdot() const
  {
    ocra_assert(x.hasTimeDerivative());
    ocra_assert(IFunction<PARTIAL_X_DOT>::canBeComputed());
    IFunction<PARTIAL_X_DOT>::_val = get<PARTIAL_X_DOT>()*x.getTimeDerivative().getValue();
  }
}



//test
class MyFunction : public ocra::Function
{
  public: 
    MyFunction(ocra::Variable& x, int dimension)
      : NamedInstance("myFunction")
      , AbilitySet(ocra::PARTIAL_X, ocra::FUN_DOT)
      , CoupledInputOutputSize(false)
      , Function(x, dimension)
    {
    }

    virtual void updateValue() const
    {
      std::cout << "Welcome in updateValue" << std::endl;
    }

    void updateJacobian() const
    {
      std::cout << "Welcome in updateJacobian" << std::endl;
    }

    void updateFdot() const
    {
      std::cout << "Welcome in updateFdot" << std::endl;
    }
};

#include "ocra/optim/Variable.h"
void testFunction()
{
  ocra::BaseVariable x("x", 10);
  MyFunction f(x, 10);
  f.get<ocra::FUN_VALUE>();
  f.get<ocra::PARTIAL_X>(3);
//  f.get<ocra::PARTIAL_T>();
  f.get<ocra::FUN_DOT>();
}

// cmake:sourcegroup=Function


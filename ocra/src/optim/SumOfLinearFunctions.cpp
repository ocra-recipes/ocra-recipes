#include "SumOfLinearFunctions.h"
#include "SolverUtilities.h"
#include "VariableMapping.h"

namespace ocra
{
  SumOfLinearFunctions::SumOfLinearFunctions(int dimension)
    : NamedInstance("sum of linear functions")
    , AbilitySet(PARTIAL_X)
    , CoupledInputOutputSize(false)
    , LinearFunction(*new CompositeVariable("SumOfLinearFunctionsVariable"), dimension)
    , _variable("SumOfLinearFunctionsVariable")
    , _offset(VectorXd::Zero(dimension))
  {
    static_cast<CompositeVariable&>(x).add( const_cast<CompositeVariable&>(_variable.getVariable()) );
    _b.setZero();
  }

  SumOfLinearFunctions::~SumOfLinearFunctions()
  {
    while(!_functions.empty())
      removeFunction(*_functions.back().first);

    disconnectVariable(); // since... (see two line below)

    delete &x; // we allocated it ourselves
  }

  SumOfLinearFunctions& SumOfLinearFunctions::addFunction(LinearFunction& f, double scale)
  {
    if(f.getDimension() != _dim)
      throw std::runtime_error("[ocra::SumOfLinearFunctions::addFunction] f must have the dimension set at construction");

    for (size_t i=0; i<_functions.size(); ++i)
    {
      if (_functions[i].first == &f)
      {
        _functions[i].second += scale;
        return *this;
      }
    }

    _functions.push_back(std::make_pair(&f,scale));
    _variable.insert(&f.getVariable());
    _variable.recomputeVariable();
    f.connect<EVT_CHANGE_VALUE>(*this, &SumOfLinearFunctions::invalidateAll);
    f.connect<EVT_CHANGE_VALUE>(*this, &SumOfLinearFunctions::invalidateb);
    f.connect<EVT_RESIZE>(*this, &SumOfLinearFunctions::checkThatFunctionsDimensionDoNotChange);

    invalidateAll();
    invalidateb(-1);

    return *this;
  }

  SumOfLinearFunctions& SumOfLinearFunctions::removeFunction(LinearFunction& f, double scale)
  {
    for (size_t i=0; i<_functions.size(); ++i)
    {
      if (_functions[i].first == &f)
      {
        _functions[i].second -= scale;
        if (std::abs(_functions[i].second) < 1.e-8)
        {
          f.disconnect<EVT_RESIZE>(*this, &SumOfLinearFunctions::checkThatFunctionsDimensionDoNotChange);
          f.disconnect<EVT_CHANGE_VALUE>(*this, &SumOfLinearFunctions::invalidateb);
          f.disconnect<EVT_CHANGE_VALUE>(*this, &SumOfLinearFunctions::invalidateAll);
          _variable.remove(&f.getVariable());
          _variable.recomputeVariable();
          _functions.erase(_functions.begin() + i);
        }
        invalidateAll();
        invalidateb(-1);
        return *this;
      }
    }
    ocra_assert(false && "the function to remove was not found");

    return *this;
  }

  SumOfLinearFunctions& SumOfLinearFunctions::removeFunction(LinearFunction& f)
  {
    for (size_t i=0;i<_functions.size(); ++i)
    {
      if (_functions[i].first == &f)
      {
        f.disconnect<EVT_RESIZE>(*this, &SumOfLinearFunctions::checkThatFunctionsDimensionDoNotChange);
        f.disconnect<EVT_CHANGE_VALUE>(*this, &SumOfLinearFunctions::invalidateb);
        f.disconnect<EVT_CHANGE_VALUE>(*this, &SumOfLinearFunctions::invalidateAll);
        _variable.remove(&f.getVariable());
        _variable.recomputeVariable();
        _functions.erase(_functions.begin() + i);
        invalidateAll();
        invalidateb(-1);
        return *this;
      }
    }
    ocra_assert(false && "the function to remove was not found");

    return *this;
  }

  void SumOfLinearFunctions::updateJacobian() const
  {
    if(!_variable.isVariableUpToDate())
      _variable.recomputeVariable();

    _jacobian.setZero();
    for (size_t i=0; i<_functions.size(); ++i)
      utils::addCompressedByCol(_functions[i].first->getJacobian(), _jacobian, _variable.find(&_functions[i].first->getVariable())->getMapping(), _functions[i].second);
  }

  void SumOfLinearFunctions::updateb() const
  {
    _b = _offset;
    for (size_t i=0; i<_functions.size(); ++i)
      _b += _functions[i].second * _functions[i].first->getb();
  }

  void SumOfLinearFunctions::doUpdateInputSizeBegin()
  {
    // does nothing : this overload allows to resize
  }

  void SumOfLinearFunctions::doChangeA(const MatrixXd& A)
  {
    throw std::runtime_error("[ocra::SumOfLinearFunctions::changeA] invalid operation on SumOfLinearFunctions");
  }

  void SumOfLinearFunctions::doChangeb(const VectorXd& b)
  {
    ocra_assert(b.size()==getDimension() && "Wrong size for b in sum of linear function");
    _offset = b;
    invalidateb(-1);
  }

  void SumOfLinearFunctions::checkThatFunctionsDimensionDoNotChange(int timestamp)
  {
    for (size_t i=0; i<_functions.size(); ++i)
      if(_functions[i].first->getDimension() != _dim)
        throw std::runtime_error("[ocra::SumOfLinearFunctions::checkThatFunctionsDimensionDoNotChange] Dimension of function " + _functions[i].first->getName() + " changed!");
  }
}

// cmake:sourcegroup=Function

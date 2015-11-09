#include "ocra/optim/BoundFunction.h"
#include <stdexcept>

namespace ocra
{
  BoundFunction::BoundFunction(Variable& x, const VectorXd& bound, eBoundType type)
    : NamedInstance("bound function")
    , AbilitySet(PARTIAL_X)
    , CoupledInputOutputSize(true)
    , DiagonalLinearFunction(x, static_cast<double>(type), bound, false)
  {
    if (type == BOUND_TYPE_SUPERIOR)
      _b *= -1;
    else if (type != BOUND_TYPE_INFERIOR)
      throw std::runtime_error("[ocra::BoundFunction::BoundFunction] invalid bound type");
  }

  BoundFunction::BoundFunction(Variable& x, const double bound, eBoundType type)
    : NamedInstance("bound function")
    , AbilitySet(PARTIAL_X)
    , CoupledInputOutputSize(true)
    , DiagonalLinearFunction(x, static_cast<double>(type), bound, true)
  {
    if (type == BOUND_TYPE_SUPERIOR)
      _b *= -1;
    else if (type != BOUND_TYPE_INFERIOR)
      throw std::runtime_error("[ocra::BoundFunction::BoundFunction] invalid bound type");
  }

  void BoundFunction::doChangeDiagonal(const VectorXd& d)
  {
    throw std::runtime_error("[ocra::BoundFunction::changeDiagonal] invalid operation on BoundFunction");
  }


  void BoundFunction::doChangeDiagonal(const double diagonalElementValue, const bool changeDefault)
  {
    throw std::runtime_error("[ocra::BoundFunction::changeDiagonal] invalid operation on BoundFunction");
  }


  void BoundFunction::doChangeDefaultDiagonalValue(const double)
  {
    throw std::runtime_error("[ocra::BoundFunction::changeDefaultDiagonalValue] invalid operation on BoundFunction");
  }


  void BoundFunction::doChangeDefaultbValue(const double)
  {
    throw std::runtime_error("[ocra::BoundFunction::changeDefaultbValue] invalid operation on BoundFunction");
  }


  void BoundFunction::changeBounds(const double bound)
  {
    if (_jacobian(0,0) == 1) //SUPERIOR_BOUND
      _b.setConstant(-bound);
    else
      _b.setConstant(bound);
  }


  void BoundFunction::changeBounds(const VectorXd& bounds)
  {
    _b = bounds;
    if (_jacobian(0,0) == 1) //SUPERIOR_BOUND
      _b *= -1.;
  }


  void BoundFunction::changeIthBound(const int i, const double bound)
  {
    ocra_assert(i<_dim && "[BoundFunction::changeIthBound] Invalid index i");
    if (_jacobian(0,0) == 1) //SUPERIOR_BOUND
      _b[i] = -bound;
    else
      _b[i] = bound;
  }
}

// cmake:sourcegroup=Function


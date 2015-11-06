#include "IdentityFunction.h"


namespace ocra
{
  IdentityFunction::IdentityFunction(Variable& x, bool resizable)
    : NamedInstance("identity function")
    , AbilitySet(PARTIAL_X)
    , CoupledInputOutputSize(resizable)
    , DiagonalLinearFunction(x)
    , _resizable(resizable)
  {
    buildIdentity();
  }


  bool IdentityFunction::isResizable() const
  {
    return _resizable;
  }

  void IdentityFunction::updateValue() const
  {
    _value = x.getValue();
  }

  void IdentityFunction::doUpdateInputSizeBegin()
  {
    if (!_resizable)
      throw std::runtime_error("[ocra::IdentityFunction::doUpdateInputSizeBegin] Function was set to non-resizable");
  }

  void IdentityFunction::doUpdateInputSizeEnd()
  {
    buildIdentity();
  }

  void IdentityFunction::doChangeDiagonal(const VectorXd& d)
  {
    throw std::runtime_error("[ocra::IdentityFunction::changeDiagonal] invalid operation on IdentityFunction");
  }


  void IdentityFunction::doChangeDiagonal(const double diagonalElementValue, const bool changeDefault)
  {
    throw std::runtime_error("[ocra::IdentityFunction::changeDiagonal] invalid operation on IdentityFunction");
  }


  void IdentityFunction::doChangeDefaultDiagonalValue(const double)
  {
    throw std::runtime_error("[ocra::IdentityFunction::changeDefaultDiagonalValue] invalid operation on IdentityFunction");
  }


  void IdentityFunction::doChangeDefaultbValue(const double)
  {
    throw std::runtime_error("[ocra::IdentityFunction::changeDefaultbValue] invalid operation on IdentityFunction");
  }


  void IdentityFunction::doChangeA(const MatrixXd& A)
  {
    ocra_assert(A.isIdentity());

    LinearFunction::doChangeA(A);
  }


  void IdentityFunction::buildIdentity()
  {
    _jacobian.setIdentity();
    _b.resize(_dim);
    _b.setZero();
  }
}

// cmake:sourcegroup=Function

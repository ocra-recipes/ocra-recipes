#include "DoubleDiagonalLinearFunction.h"

namespace ocra
{
  DoubleDiagonalLinearFunction::DoubleDiagonalLinearFunction(Variable& x, 
                            const double diagonalElementValue1, const double diagonalElementValue2, const double vectorElementValue,
                            const bool useDefaultValue)
    :NamedInstance("double diagonal linear function")
    , AbilitySet(PARTIAL_X)
    , CoupledInputOutputSize(true)
    , LinearFunction(x, x.getSize()/2)
    , _useDefaultValue(useDefaultValue)
    , _defaultDiagonalValue1(diagonalElementValue1)
    , _defaultDiagonalValue2(diagonalElementValue2)
    , _defaultbValue(vectorElementValue)
  {
    _d1.resize(_dim);
    _d1.setConstant(diagonalElementValue1);
    _d2.resize(_dim);
    _d2.setConstant(diagonalElementValue2);
    buildA();
    _b.resize(_dim);
    _b.setConstant(vectorElementValue);
  }


  void DoubleDiagonalLinearFunction::changeDiagonal1(const VectorXd& d)
  {
    if (d.size() != _dim)
      throw std::runtime_error("[ocra::DoubleDiagonalLinearFunction::changeDiagonal1] Size of diagonal d doesn't match the function dimension");
    _d1 = d;
    buildA();
  }

  void DoubleDiagonalLinearFunction::changeDiagonal2(const VectorXd& d)
  {
    if (d.size() != _dim)
      throw std::runtime_error("[ocra::DoubleDiagonalLinearFunction::changeDiagonal2] Size of diagonal d doesn't match the function dimension");
    _d2 = d;
    buildA();
  }

  void DoubleDiagonalLinearFunction::changeDiagonal1(const double diagonalElementValue, const bool changeDefault)
  {
    _d1.setConstant(diagonalElementValue);
    if (changeDefault)
      _defaultDiagonalValue1 = diagonalElementValue;
    buildA();
  }

  void DoubleDiagonalLinearFunction::changeDiagonal2(const double diagonalElementValue, const bool changeDefault)
  {
    _d2.setConstant(diagonalElementValue);
    if (changeDefault)
      _defaultDiagonalValue2 = diagonalElementValue;
    buildA();
  }

  void DoubleDiagonalLinearFunction::changeDefaultDiagonalValue1(const double v)
  {
    _defaultDiagonalValue1 = v;
  }

  void DoubleDiagonalLinearFunction::changeDefaultDiagonalValue2(const double v) 
  {
    _defaultDiagonalValue2 = v;
  }

  void DoubleDiagonalLinearFunction::changeDefaultbValue(const double v)
  {
    _defaultbValue = v;
  }

  void DoubleDiagonalLinearFunction::doChangeA(const MatrixXd& A)
  {
    //test if A is double diagonal
    ocra_assert(A.block(0,0,_dim,_dim).isDiagonal());
    ocra_assert(A.block(0,_dim,_dim,_dim).isDiagonal());
    
    LinearFunction::changeA(A);
  }


  void DoubleDiagonalLinearFunction::updateValue() const
  {
    const VectorXd& x_value = x.getValue();
    _value = _d1.asDiagonal()*x_value.head(_dim) + _d2.asDiagonal()*x_value.tail(_dim) + _b;
  }

  void DoubleDiagonalLinearFunction::doUpdateInputSizeBegin()
  {
    bool increase = _dim < x.getSize()/2;
    if (!_useDefaultValue && increase)
      throw std::runtime_error("[ocra::DoubleDiagonalLinearFunction::doUpdateSize] No default value to increase the size of A");

    int oldDim = _dim;
    int dim = x.getSize();

    if (increase)
    {
      VectorXd tmpb(_b);          //we need to save _b before resizing
      _b.resize(dim);
      _b.head(oldDim) = tmpb;
      _b.tail(dim-oldDim).setConstant(_defaultbValue);

      _d1.resize(dim);    //value of _d1 are in _jacobian, no need to save them
      _d2.resize(dim);    //value of _d2 are in _jacobian
      _d1.head(oldDim) = _jacobian.block(0,0,oldDim,oldDim).diagonal();
      _d2.head(oldDim) = _jacobian.block(0,oldDim,oldDim,oldDim).diagonal();
      _d1.tail(dim-oldDim).setConstant(_defaultDiagonalValue1);
      _d2.tail(dim-oldDim).setConstant(_defaultDiagonalValue2);
    }
    else
    {
      VectorXd tmpb(_b);          //we need to save _b before resizing
      _b.resize(dim);
      _b = tmpb.head(dim);

      _d1.resize(dim);
      _d2.resize(dim);
      _d1 = _jacobian.block(0,0,oldDim,oldDim).diagonal().head(dim);
      _d2 = _jacobian.block(0,oldDim,oldDim,oldDim).diagonal().head(dim);
    }
  }

  void DoubleDiagonalLinearFunction::doUpdateInputSizeEnd()
  {
    buildA();
  }

  int DoubleDiagonalLinearFunction::computeDimensionFromInputSize() const
  {
    ocra_assert(2*(x.getSize()/2) == x.getSize() && "double diagonal linear function can only work with variables whose size is even.");
    return x.getSize()/2;
  }

  void DoubleDiagonalLinearFunction::doUpdateDimensionBegin(int newDimension)
  {
    // do nothing: this overloaded method avoid a default exception to be thrown
  }

  void DoubleDiagonalLinearFunction::buildA()
  {
   _jacobian.setZero();
   _jacobian.block(0,0,_dim,_dim).diagonal() = _d1;
   _jacobian.block(0,_dim,_dim,_dim).diagonal() = _d2;
   propagate<EVT_CHANGE_VALUE>();
  }

}

// cmake:sourcegroup=Function

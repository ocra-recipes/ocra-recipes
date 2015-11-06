#include "ocra/optim/DiagonalLinearFunction.h"

#include <stdexcept>

namespace ocra
{
  DiagonalLinearFunction::DiagonalLinearFunction(Variable& x, const double diagonalElementValue, 
                                                 const double vectorElementValue,
                                                 const bool useDefaultValue)
    : NamedInstance("diagonal linear function")
    , AbilitySet(PARTIAL_X)
    , CoupledInputOutputSize(true)
    , LinearFunction(x, x.getSize())
    , _useDefaultValue(useDefaultValue), _defaultDiagonalValue(diagonalElementValue)
    , _defaultbValue(vectorElementValue)
  {
    _d.resize(_dim);
    _d.setConstant(diagonalElementValue);
    buildA();
    _b.resize(_dim);
    _b.setConstant(vectorElementValue);
  }


  DiagonalLinearFunction::DiagonalLinearFunction(Variable& x)
    : NamedInstance("diagonal linear function")
    , AbilitySet(PARTIAL_X)
    , CoupledInputOutputSize(true)
    , LinearFunction(x, x.getSize())
    , _useDefaultValue(false)
    , _defaultDiagonalValue(0.)
    , _defaultbValue(0.)
  {
  }


  void DiagonalLinearFunction::changeDiagonal(const VectorXd& d)
  {
    doChangeDiagonal(d);
    ocra_assert(_d.size() == _dim);
    buildA();
  }


  void DiagonalLinearFunction::changeDiagonal(const double diagonalElementValue, const bool changeDefault)
  {
    doChangeDiagonal(diagonalElementValue, changeDefault);
    buildA();
  }


  void DiagonalLinearFunction::changeDefaultDiagonalValue(const double v)
  {
    doChangeDefaultDiagonalValue(v);
  }


  void DiagonalLinearFunction::changeDefaultbValue(const double v)
  {
    doChangeDefaultbValue(v);
  }

  
  void DiagonalLinearFunction::doChangeDiagonal(const VectorXd& d)
  {
    _d = d;
  }


  void DiagonalLinearFunction::doChangeDiagonal(const double diagonalElementValue, const bool changeDefault)
  {
    _d.setConstant(diagonalElementValue);
    if (changeDefault)
      _defaultDiagonalValue = diagonalElementValue;
  }


  void DiagonalLinearFunction::doChangeDefaultDiagonalValue(const double v)
  {
    _defaultDiagonalValue = v;
  }


  void DiagonalLinearFunction::doChangeDefaultbValue(const double v)
  {
    _defaultbValue = v;
  }



  void DiagonalLinearFunction::doChangeA(const MatrixXd& A)
  {
    ocra_assert(A.isDiagonal());

    LinearFunction::doChangeA(A);
  }


  void DiagonalLinearFunction::updateValue() const
  {
    _value = _d.asDiagonal()*x.getValue() + _b;
  }


  void DiagonalLinearFunction::doUpdateInputSizeBegin()
  {
    bool increase = _dim < x.getSize();
    if (!_useDefaultValue && increase)
      throw std::runtime_error("[ocra::DiagonalLinearFunction::doUpdateSize] No default value to increase the size of A");

    int oldDim = _dim;
    int dim = x.getSize();

    if (increase)
    {
      VectorXd tmpb(_b);          //we need to save _b before resizing
      _b.resize(dim);
      _b.head(oldDim) = tmpb;
      _b.tail(dim-oldDim).setConstant(_defaultbValue);

      _d.resize(dim);    //value of _d are in _jacobian, no need to save them
      _d.head(oldDim) = _jacobian.diagonal();
      _d.tail(dim-oldDim).setConstant(_defaultDiagonalValue);
    }
    else
    {
      VectorXd tmpb(_b);          //we need to save _b before resizing
      _b.resize(dim);
      _b = tmpb.head(dim);

      _d.resize(dim);    //value of _d are in _jacobian, no need to save them
      _d = _jacobian.diagonal().head(dim);
    }
  }

  void DiagonalLinearFunction::doUpdateInputSizeEnd()
  {
    buildA();
  }


  void DiagonalLinearFunction::buildA()
  {
    _jacobian.setZero();
    _jacobian.diagonal() = _d;
    propagate<EVT_CHANGE_VALUE>();
  }


  int DiagonalLinearFunction::computeDimensionFromInputSize() const
  {
    return x.getSize();
  }

  void DiagonalLinearFunction::doUpdateDimensionBegin(int newDimension)
  {
    // do nothing: this overloaded method avoid a default exception to be thrown
  }



  void testDiagonalLinearFunction()
  {
    BaseVariable x("x", 5);
    DiagonalLinearFunction f(x, 1., 0., true);

    x.setValue(VectorXd::Random(5));
    std::cout << x.getValue() << std::endl;
    std::cout << f.getValue() << std::endl  << std::endl;

    x.resize(6);
    x.setValue(VectorXd::Random(6));
    std::cout << x.getValue() << std::endl;
    std::cout << f.getValue() << std::endl  << std::endl;
    std::cout << f.getJacobian() << std::endl  << std::endl;
  }
}

// cmake:sourcegroup=Function


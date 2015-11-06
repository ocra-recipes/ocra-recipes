#include "ocra/optim/LinearFunction.h"

//std includes
#include <sstream>
#include <stdexcept>

//using namespace xde;    //TODO: we comment that, we should not!!!!

namespace ocra
{
  LinearFunction::LinearFunction(Variable& x, int dimension)
    :NamedInstance("linear function")
    ,AbilitySet(PARTIAL_X)
    ,CoupledInputOutputSize(false)
    ,Function(x,dimension, LINEARITY_LINEAR, CONVEXITY_CONVEX_AND_CONCAVE, CONTINUITY_CINF)
    ,_bIsUpToDate(false)
    ,_inhibitPropagationFromb(false)
  {
    _b.resize(_dim);
  }

  LinearFunction::~LinearFunction()
  {
  }

  const MatrixXd& LinearFunction::getA() const
  {
    return getJacobian();
  }

  const VectorXd& LinearFunction::getb() const
  {
    if (!_bIsUpToDate)
    {
      updateb();
      _bIsUpToDate = true;
    }
    return _b;
  }

  void LinearFunction::changeA(const MatrixXd& A)
  {
    doChangeA(A);
    ocra_assert((_jacobian.rows() == _dim && _jacobian.cols() == x.getSize()) &&
      "The size of the new matrix A does not match with the size of previous A");
    propagate<EVT_CHANGE_VALUE>();
  }

  void LinearFunction::changeb(const VectorXd& b)
  {
    doChangeb(b);
    _bIsUpToDate = true;
    ocra_assert(_b.size() == _dim && "Size of b does not match with function dimension.");
    propagate<EVT_CHANGE_VALUE>();
  }

  void LinearFunction::doChangeA(const MatrixXd& A)
  {
    _jacobian = A;
  }

  void LinearFunction::doChangeb(const VectorXd& b)
  {
    _b = b;
  }

  void LinearFunction::invalidateb(int timestamp)
  {
    _bIsUpToDate = false;
    if (!_inhibitPropagationFromb)
      propagate<EVT_CHANGE_VALUE>();
  }

  void LinearFunction::updateValue() const
  {
    //force the evaluation of the jacobian and b
    _value.noalias() = getJacobian() * x.getValue() + getb();
  }

  void LinearFunction::updateJacobian() const
  {
    //do nothing, gradient is constant and initialized in the ctor
  }

  void LinearFunction::updateb() const
  {
    //do nothing
  }

  void LinearFunction::doUpdateInputSizeEnd()
  {
    _b.resize(_dim);
    _bIsUpToDate = false;
  }

  void LinearFunction::inhibitPropagationFromb() const
  {
    _inhibitPropagationFromb = true;
  }

  void LinearFunction::desinhibitPropagationFromb() const
  {
    _inhibitPropagationFromb = false;
  }

}


#include "ocra/optim/Variable.h"
#include <math.h>

namespace ocra
{
  void testLinearFunction()
  {
    BaseVariable x1("x1", 2);
    BaseVariable x3("x2", 2);
    BaseVariable x2("x3", 2);
    CompositeVariable y("y", x1);
    y.add(x2).add(x3);
    int n = 3;
    MatrixXd A(n,y.getSize());
    VectorXd b(n);
    VectorXd v(y.getSize());

    for (int i=0; i<n; ++i)
    {
      b[i] = (4*rand())/RAND_MAX - 2;
      for (int j=0; j<y.getSize(); ++j)
        A(i,j) = (5*rand())/(RAND_MAX) - 2;
    }
    std::cout << A << std::endl;
    std::cout << b << std::endl;

    LinearFunction f(y,A,b);

    for (int i=0; i<y.getSize(); ++i)
      v[i] = (4*rand())/RAND_MAX - 2;

    y.setValue(v);
    std::cout << (VectorXd)y << std::endl;
    std::cout << f.getValue() << std::endl;
  }
}

// cmake:sourcegroup=Function

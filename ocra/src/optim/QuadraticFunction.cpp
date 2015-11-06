#include "ocra/optim/QuadraticFunction.h"

namespace ocra
{
  QuadraticFunction::QuadraticFunction(Variable& x, int dimension)
    :NamedInstance("QuadraticFunction")
    ,AbilitySet(PARTIAL_X, PARTIAL_XX)
    ,CoupledInputOutputSize(false)
    ,Function(x,1, LINEARITY_QUADRATIC, CONVEXITY_UNDEFINED, CONTINUITY_CINF
    , false //not time dependant
    , true) //thus time separable
    , _qIsUpToDate(false)
    , _rIsUpToDate(false)
    ,_inhibitPropagationFrom_q_or_r(false)
  {
    for (int i=0; i<dimension; ++i)
      _q.push_back(x.getSize() ? new VectorXd(x.getSize()) : new VectorXd()); //TODO: Fix this (Shoot Eigen until it handles VectorXd(0))
    _r.resize(dimension);
  }

  QuadraticFunction::~QuadraticFunction()
  {
    for (size_t i=0; i<_q.size(); ++i)
      delete _q[i];
  }

  void QuadraticFunction::changePi(const MatrixXd& Pi, int index)
  {
    ocra_assert(index < _dim);

    doChangePi(Pi, index);

    ocra_assert(IFunction<PARTIAL_XX>::_val[index]->cols() == x.getSize() 
            && IFunction<PARTIAL_XX>::_val[index]->rows() == x.getSize()
            && "Pi does not have the good dimensions");

    propagate<EVT_CHANGE_VALUE>();
  }

  void QuadraticFunction::changeqi(const VectorXd& qi, int index)
  {
    ocra_assert(index <_dim);
    
    doChangeqi(qi, index);

    ocra_assert(_q[index]->size() == x.getSize());

    propagate<EVT_CHANGE_VALUE>();
  }

  void QuadraticFunction::changeri(real ri, int index)
  {
    ocra_assert(index <_dim);
    
    doChangeri(ri, index);

    propagate<EVT_CHANGE_VALUE>();
  }

  void QuadraticFunction::doChangePi(const MatrixXd& Pi, int index)
  {
    ocra_assert((Pi-Pi.transpose()).isZero()  && "Pi is not symmetric");
    *IFunction<PARTIAL_XX>::_val[index] = Pi;
  }

  void QuadraticFunction::doChangeqi(const VectorXd& qi, int index)
  {
    *_q[index] = qi;
  }

  void QuadraticFunction::doChangeri(real ri, int index)
  {
    _r[index] = ri;
  }


  void QuadraticFunction::updateValue() const
  {
    for (int i=0; i<_dim; ++i)
    {
      const MatrixXd& Pi = getPi(i);

      _value[i] = 0.5*x.getValue().dot(Pi*x.getValue());
      _value[i] += x.getValue().dot(getqi(i));
    }
    _value += getr();
  }


  void QuadraticFunction::updateJacobian() const
  {
    for (int i=0; i<_dim; ++i)
    {
      const MatrixXd& Pi = getPi(i);

      _jacobian.row(i) = Pi*x.getValue() + getqi(i);
    }
  }

  void QuadraticFunction::updateHessian() const
  {
    //do nothing
  }

  void QuadraticFunction::updateq() const
  {
    //do nothing
  }

  void QuadraticFunction::updater() const
  {
    //do nothing
  }

  void QuadraticFunction::doUpdateInputSizeEnd()
  {
    for (int i=0; i<_dim; ++i)
      _q[i]->resize(x.getSize());
    //invalidateq();
  }

  void QuadraticFunction::doUpdateDimensionEnd(int oldDimension)
  {
    if (oldDimension < _dim)
    {
      for (int i=oldDimension; i<_dim; ++i)
        _q.push_back(new VectorXd(x.getSize()));
    }
    else
    {
      for (int i=oldDimension-1; i>=_dim; --i)
      {
        delete _q[i];
        _q.pop_back();
      }
    }
    _r.resize(_dim);
    //invalidateq();
    //invalidater();
  }


  void QuadraticFunction::invalidateq(int timestamp)
  {
    _qIsUpToDate = false;
    propagate<EVT_CHANGE_VALUE>(timestamp);
  }

  void QuadraticFunction::invalidater(int timestamp)
  {
    _rIsUpToDate = false;
    propagate<EVT_CHANGE_VALUE>(timestamp);
  }

  void QuadraticFunction::inhibitPropagationFrom_q_or_r() const
  {
    _inhibitPropagationFrom_q_or_r = true;
  }

  void QuadraticFunction::desinhibitPropagationFrom_q_or_r() const
  {
    _inhibitPropagationFrom_q_or_r = false;
  }





  void testQuadraticFunction()
  {
    BaseVariable x("x",5);
    MatrixXd Q = MatrixXd::Random(5,5);
    MatrixXd P = 0.5*(Q + Q.transpose());
    VectorXd q = VectorXd::Random(5);
    double r = VectorXd::Random(1)[0];

    QuadraticFunction qf(x, P, q, r);

    x.setValue(VectorXd::Random(5));
    std::cout << qf.getValue() << std::endl << std::endl;
    std::cout << qf.getJacobian() << std::endl << std::endl;
    std::cout << qf.get<PARTIAL_XX>(0) << std::endl << std::endl;

    P.setZero();
    q << 1,2,3,4,5;
    P.diagonal() = q;
    x.setValue(VectorXd::Constant(5, 1));

    qf.changePi(P);
    qf.changeqi(q);
    qf.changeri(0);

    std::cout << qf.getValue() << std::endl << std::endl;
    std::cout << qf.getJacobian() << std::endl << std::endl;
    std::cout << qf.get<PARTIAL_XX>(0) << std::endl << std::endl;
  }

}

// cmake:sourcegroup=Function

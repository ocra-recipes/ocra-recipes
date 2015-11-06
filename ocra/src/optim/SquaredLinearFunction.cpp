#include "ocra/optim/SquaredLinearFunction.h"

//using namespace xde;      //TODO: we comment that, we should not!!!!

namespace ocra
{
  SquaredLinearFunction::SquaredLinearFunction(LinearFunction* f)
    :NamedInstance("squared linear function")
    ,AbilitySet(PARTIAL_X, PARTIAL_XX)
    ,CoupledInputOutputSize(false)
    ,QuadraticFunction(f->getVariable())
    , _f(f)
    , _weight(VectorXd::Ones(f->getDimension()))
  {
    f->connect<EVT_CHANGE_VALUE>(*this, &Function::invalidateAll);
    f->connect<EVT_CHANGE_VALUE>(*this, &SquaredLinearFunction::invalidateq);
    f->connect<EVT_CHANGE_VALUE>(*this, &SquaredLinearFunction::invalidater);
    f->connect<EVT_RESIZE>(*this, &SquaredLinearFunction::onResize);
    inhibitPropagationFrom_q_or_r();
    changeConvexityProperty(CONVEXITY_CONVEX);
  }

  SquaredLinearFunction::~SquaredLinearFunction()
  {
    _f->disconnect<EVT_CHANGE_VALUE>(*this, &Function::invalidateAll);
    _f->disconnect<EVT_CHANGE_VALUE>(*this, &SquaredLinearFunction::invalidateq);
    _f->disconnect<EVT_CHANGE_VALUE>(*this, &SquaredLinearFunction::invalidater);
    _f->disconnect<EVT_RESIZE>(*this, &SquaredLinearFunction::onResize);
  }


  void SquaredLinearFunction::changeWeight(const VectorXd& weight)
  {
    if(weight.size() != _f->getDimension())
      throw std::runtime_error("[SquaredLinearFunction::changeWeight] weight has not the appropriate size");
    _weight = weight;
  }

  // void SquaredLinearFunction::setWeight(const VectorXd& weight)
  // {
  //   if(weight.size() != _f->getDimension())
  //     throw std::runtime_error("[SquaredLinearFunction::changeWeight] weight has not the appropriate size");
  //   _weight = weight;
  // }


  LinearFunction& SquaredLinearFunction::getFunction()
  {
    return *_f;
  }


  const LinearFunction& SquaredLinearFunction::getFunction() const
  {
    return *_f;
  }


  void SquaredLinearFunction::updateHessian() const
  {
    *IFunction<PARTIAL_XX>::_val[0] = _f->getA().transpose()* _weight.asDiagonal() * _f->getA();
  }


  void SquaredLinearFunction::updateq() const
  {
    // std::cout << "updateQ in SquaredLinearFunction\n"<< _weight.transpose() << std::endl;
    *_q[0] = _f->getA().transpose()*_weight.asDiagonal()*_f->getb();
  }


  void SquaredLinearFunction::updater() const
  {
    _r[0] = 0.5 * _f->getb().transpose() * _weight.asDiagonal() *_f->getb();
  }

  
  void SquaredLinearFunction::doUpdateInputSizeBegin()
  {
    // Do nothing. This overload is just here to enable input resizing (default implementation throw an exception).
  }


  void SquaredLinearFunction::onResize(int)
  {
    if(_f->getDimension() < _weight.size())
    {
      VectorXd oldWeight = _weight;
      _weight = oldWeight.head(_f->getDimension());
    }
    else if(_f->getDimension() < _weight.size())
    {
      VectorXd oldWeight = _weight;
      _weight = VectorXd::Ones(_f->getDimension());
      _weight.head(oldWeight.size()) = oldWeight;
    }
  }

}

#include "ocra/optim/DiagonalLinearFunction.h"

namespace ocra
{
  void testSquaredLinearFunction()
  {
    BaseVariable x("x", 3);
    MatrixXd A(2,3); A << 1,0,0, 0,1,1;
    VectorXd b(2); b << 1,0;
    LinearFunction lf(x, A, b);
    SquaredLinearFunction sqf(&lf);

    VectorXd v(3); v << 1,2,1;
    x.setValue(v);

    std::cout << sqf.getValue() << std::endl;

    BaseVariable y("y", 3);
    DiagonalLinearFunction dl(y,1.,2.,true);
    SquaredLinearFunction sqf2(&dl);

    VectorXd vy(3); vy << 1,2,1;
    y.setValue(vy);
    std::cout << sqf2.getJacobian() << std::endl;

    y.resize(4);
    vy.resize(4); vy << 1,2,1,0;
    y.setValue(vy);
    std::cout << sqf2.getJacobian() << std::endl;
  }
}

// cmake:sourcegroup=Function

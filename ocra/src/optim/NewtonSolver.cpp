#include "NewtonSolver.h"
#include <limits>

namespace ocra
{
  NewtonSolver::NewtonSolver(bool fullNewton)
    : NamedInstance("NewtonSolver")
    , _adaptativeAlpha(true)
    , _completeMethod(fullNewton)
    , _maxIter(100)
    , _alpha(1)
    , _epsilonSqr(1e-12)
    , _x(0x0,0)
    , _H(0x0,0,0)
    , _g(0x0,0)
    , _p(0x0,0)
    , _buffer(1000)
    , _ldlt(10)
  {
  }

  void NewtonSolver::setAdaptativeAlpha(bool adapt)
  {
    _adaptativeAlpha = adapt;
  }

  bool NewtonSolver::getAdaptativeAlpha() const
  {
    return _adaptativeAlpha;
  }

  void NewtonSolver::setEpsilon(double eps)
  {
    ocra_assert(eps > 0);
    _epsilonSqr = eps*eps;
  }

  double NewtonSolver::getEpsilon() const
  {
    return std::sqrt(_epsilonSqr);
  }

  void NewtonSolver::setMaxIter(int maxIter)
  {
    ocra_assert(maxIter>0);
    _maxIter = maxIter;
  }

  int NewtonSolver::getMaxIter() const
  {
    return _maxIter;
  }

  void NewtonSolver::set_x0(const VectorXd& x0)
  {
    _x0 = x0;
  }

  const VectorXd& NewtonSolver::get_x0() const
  {
    return _x0;
  }

  void NewtonSolver::addObjective(GenericObjective& obj)
  {
    ocra_assert(obj.getFunction().canCompute<PARTIAL_X>());
    internalAddObjective(obj);
    _objectives.push_back(&obj);
  }

  void NewtonSolver::removeObjective(Function& obj)
  {
    for(size_t i = 0; i < _objectives.size(); ++i)
    {
      if(&_objectives[i]->getFunction() == &obj)
      {
        removeObjective(*_objectives[i]);
        break;
      }
    }
  }

  void NewtonSolver::removeObjective(GenericObjective& obj)
  {
    internalRemoveObjective(obj);
    _objectives.erase(std::find(_objectives.begin(), _objectives.end(), &obj));
  }

  void NewtonSolver::printValuesAtSolution()
  {
    std::cout << "objective(s):" <<std::endl;
    for (unsigned int i=0; i<_objectives.size(); ++i)
      std::cout << _objectives[i]->getValue() << std::endl;
  }

  std::string NewtonSolver::toString() const
  {
    return "non implemented yet";
  }

  void NewtonSolver::doPrepare()
  {
    const int s=n();
    if (s!=_H.rows())
    {
      _buffer.resize(s*(2*s+3));
      new (&_H) MatrixMap(_buffer.allocate(s*s), s, s);
      _tmpHbuf = _buffer.allocate(s*s);
      new (&_g) VectorMap(_buffer.allocate(s), s);
      _tmpgbuf = _buffer.allocate(s);
      new (&_p) VectorMap(_buffer.allocate(s), s);
      new (&_ldlt) LDLT<MatrixXd>(s);
    }
  }

  void NewtonSolver::doSolve()
  {
    initInfo();
    _alpha = 1;
    new (&_x) VectorMap(_result.solution.data(), _result.solution.size());
    if (_x0.size() != _x.size())
    {
      _x0 = VectorXd::Zero(_x.size());
      _info.usedDefaultGuess = false;
    }
    newtonSolve();
    _info.residual = std::sqrt(_info.residual);
    translateReturnInfo(_result.info);
  }

  void NewtonSolver::doConclude()
  {
  }

  void NewtonSolver::initInfo()
  {
    _info.iter = 1;
    _info.residual = std::numeric_limits<double>::max();
    _info.usedDefaultGuess = true;
    _info.nonPositiveH = false;
    _info.smallAlpha = false;
  }


  void NewtonSolver::newtonSolve()
  {
    _x=_x0;
    for (; _info.iter<=_maxIter; ++_info.iter)
    {
      setVariableValue(_x);
      compute_H();
      _ldlt.compute(_H);       //[TODO] this line create a temporary, see eigen's mailing list thread "Accepting different matrix type in the decompositions"
      if (!_ldlt.isPositive())
      {
        _info.nonPositiveH = true;
        _info.success = false;
        return;
      }
      compute_g();
      _p = _ldlt.solve(_g);
      _x -= compute_alpha()*_p;

      if (_info.residual < _epsilonSqr)
        break;
    }
    //std::cout << "nb iteration: " << i << std::endl;
    _info.success = _info.iter <= _maxIter;
    _info.smallAlpha = _alpha < std::sqrt(getEpsilon());
    _x0 = _x;
  }

  double NewtonSolver::compute_alpha()
  {
    if (_adaptativeAlpha)
    {
      double ri = _alpha*_alpha*_p.squaredNorm();  
      if (ri >= _info.residual)
      {
        _alpha /=2;
        _info.residual = ri/4;
        return _alpha;
      }
      else
      {
        double newAlpha = _alpha;
        _info.residual = ri;
        _alpha = std::min<double> (_alpha * 1.1, 1.0);
        return newAlpha;
      }
    }
    else
    {
      _info.residual = _alpha*_alpha*_p.squaredNorm();
      return _alpha;
    }
  }

  void NewtonSolver::compute_H()
  {
    _H.setZero();
    for (size_t i=0; i<_objectives.size(); ++i)
    {
      Function& obj = _objectives[i]->getFunction();
      if (obj.canCompute<PARTIAL_XX>() && _completeMethod)
      {
        ocra_assert(false && "non implemented yet");
      }
      else
      {
        const int s = obj.getVariable().getSize();
        MatrixMap _tmpH(_tmpHbuf, s, s);
        const MatrixXd& J = obj.getJacobian();
        _tmpH = J.transpose()*J;
        utils::addCompressed2d(_tmpH, _H, findMapping(obj.getVariable()), _objectives[i]->getWeight());
      }
    }
  }

  void NewtonSolver::compute_g()
  {
    _g.setZero();
    for (size_t i=0; i<_objectives.size(); ++i)
    {
      Function& obj = _objectives[i]->getFunction();
      const int s = obj.getVariable().getSize();
      VectorMap _tmpg(_tmpgbuf, s);
      _tmpg = obj.getJacobian().transpose()*obj.getValue();
      utils::addCompressedByRow(_tmpg, _g, findMapping(obj.getVariable()), _objectives[i]->getWeight());
    }
  }

  void NewtonSolver::translateReturnInfo(eReturnInfo& ocraInfo)
  {
    if (_info.success)
      ocraInfo = RETURN_SUCCESS;
    else if (_info.iter>_maxIter)
      ocraInfo = RETURN_MAX_ITER_REACHED;
    else if (_info.nonPositiveH)
      ocraInfo = RETURN_INCONSISTENT_PROBLEM;
    else
      ocraInfo = RETURN_ERROR;
  }

}


#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/SquaredLinearFunction.h"
#include "ocra/optim/QLDSolver.h"

namespace ocra
{
  void testNewtonSolver01()
  {
    NewtonSolver solver;
    //solver.setAdaptativeAlpha(false);
    //solver.setMaxIter(10);

    const int n1 = 15;
    const int n2 = 5;
    BaseVariable x("x", n1);
    BaseVariable y("y", n2);
    CompositeVariable xy("xy", x, y);

    MatrixXd A1 = MatrixXd::Random(n1,n1);
    VectorXd b1 = VectorXd::Random(n1);
    MatrixXd A2 = 100*MatrixXd::Random(n2,n2);
    VectorXd b2 = 100*VectorXd::Random(n2);
    LinearFunction* fx = new LinearFunction(x, A1, b1);
    LinearFunction* fy = new LinearFunction(y, A2, b2);
    
    ObjectivePtr<LinearFunction> obj_x(fx);
    ObjectivePtr<LinearFunction> obj_y(fy);

    solver.addObjective(obj_x);
    solver.addObjective(obj_y);
    std::cout << solver.solve().solution.transpose() << std::endl;

    ObjectivePtr<SquaredLinearFunction> obj_x2(new SquaredLinearFunction(fx));
    ObjectivePtr<SquaredLinearFunction> obj_y2(new SquaredLinearFunction(fy));
    QLDSolver solverQP;
    solverQP.addObjective(obj_x2);
    solverQP.addObjective(obj_y2);
    std::cout << solverQP.solve().solution.transpose() << std::endl;

    if ((solver.getLastResult().solution-solverQP.getLastResult().solution).isZero())
      std::cout << "It works !" << std::endl;
    else
      std::cout << "It doesn't work !" << std::endl;
  }


  void testNewtonSolver02()
  {
    NewtonSolver solver;
    //solver.setAdaptativeAlpha(false);
    //solver.setMaxIter(10);

    const int n1 = 15;
    const int n2 = 5;
    const double a=0.01;
    BaseVariable x("x", n1);
    BaseVariable y("y", n2);
    CompositeVariable xy("xy", x, y);

    MatrixXd A1 = MatrixXd::Random(n1,n1);
    VectorXd b1 = VectorXd::Random(n1);
    MatrixXd A2 = 100*MatrixXd::Random(n2,n2);
    VectorXd b2 = 100*VectorXd::Random(n2);
    LinearFunction* fx = new LinearFunction(x, A1, b1);
    LinearFunction* fy = new LinearFunction(y, A2, b2);
    
    ObjectivePtr<LinearFunction> obj_x(fx);
    ObjectivePtr<LinearFunction> obj_y(fy);

    solver.addObjective(obj_x);
    solver.addObjective(obj_y);

    for (int i=0; i<100; ++i)
    {
      std::cout << solver.solve().solution.transpose() << std::endl;
      A1 += a*MatrixXd::Random(n1,n1);
      b1 += a*VectorXd::Random(n1);
      A2 += a*MatrixXd::Random(n2,n2);
      b2 += a*VectorXd::Random(n2);
      fx->changeA(A1);
      fx->changeb(b1);
      fy->changeA(A2);
      fy->changeb(b2);
    }
  }
}

// cmake:sourcegroup=Solvers

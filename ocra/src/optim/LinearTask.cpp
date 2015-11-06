#include "ocra/optim/LinearTask.h"
#include "ocra/optim/FunctionUtilities.h"

#include <stdexcept>

//using namespace xde;        //TODO: we comment that, we should not!!!!

namespace ocra 
{
  LinearTask::LinearTask(Function& f, Function& L)
    :NamedInstance("linear task")
    ,AbilitySet(PARTIAL_X)
    ,CoupledInputOutputSize(false)
    ,LinearFunction(f.getVariable().getTimeDerivative(), f.getDimension())
    ,_f(&f), _L(&L)
  {
    //check
    if (_dim != L.getVariable().getSize())
      throw std::runtime_error("[ocra::LinearTask::LinearTask] Input size of L doesn't match with dimension of f");
    if (_dim != L.getDimension())
      throw std::runtime_error("[ocra::LinearTask::LinearTask] Dimension of L doesn't match with dimension of f");

    //dependencies
    f.connect<EVT_CHANGE_VALUE>(*this, &LinearFunction::invalidateb);
    f.connect<EVT_CHANGE_VALUE>(*this, &Function::invalidateAll);
    f.connect<EVT_RESIZE>(*this, &LinearTask::updateDimension);
    L.connect<EVT_CHANGE_VALUE>(*this, &LinearFunction::invalidateb);
    //f.connect<EVT_CHANGE_VALUE, Function>(L, &Function::invalidateAll); //emulate the fact that L takes f as an input
  }

  LinearTask::~LinearTask()
  {
    _f->disconnect<EVT_CHANGE_VALUE>(*this, &LinearFunction::invalidateb);
    _f->disconnect<EVT_CHANGE_VALUE>(*this, &Function::invalidateAll);
    _f->disconnect<EVT_RESIZE>(*this, &LinearTask::updateDimension);
    _L->disconnect<EVT_CHANGE_VALUE>(*this, &LinearFunction::invalidateb);
  }
 
  void LinearTask::updateb() const
  {
    inhibitPropagationFromb();
    _L->getVariable().setValue(_f->getValue());
    desinhibitPropagationFromb();
    _b = _L->getValue();
  }



  LinearTask* LinearTask::createEqualityTaskWithLinearLaw(Function* f, double weight)
  {
    VectorXd w(f->getDimension());
    w.setConstant(weight);
    return createEqualityTaskWithLinearLaw(f, w);
  }

  
  LinearTask* LinearTask::createEqualityTaskWithLinearLaw(Function* f, const VectorXd& weight)
  {
    Variable* v = utils::createOutputVariable(*f);
    int n = v->getSize();
    LinearFunction* L = new DiagonalLinearFunction(*v, weight, 0., true, weight[n-1]);
    return new LinearTask(*f, *L);
  }


  LinearTask* LinearTask::createInequalityTaskWithLinearLaw(Function* f, double weight, double fi)
  {
    int n = f->getDimension();
    double wi = weight / fi;
    if (n == 0)
    {
      Variable* v = utils::createOutputVariable(*f);
      LinearFunction* L = new DiagonalLinearFunction(*v, wi, 0., true);
      return new LinearTask(*f, *L);
    }
    else
    {
      VectorXd w(n);
      w.fill(wi);
      return createInequalityTaskWithLinearLaw(f, w);
    }
  }


  LinearTask* LinearTask::createInequalityTaskWithLinearLaw(Function* f, double weight, const VectorXd& fi)
  {
    int n = f->getDimension();
    ocra_assert(fi.size() == n);
    VectorXd w(n);
    for (int i=0; i<n; ++i)
      w[i] = weight / fi[i];
    return createInequalityTaskWithLinearLaw(f, w);
  }

  
  LinearTask* LinearTask::createInequalityTaskWithLinearLaw(Function* f, const VectorXd& weight, double fi)
  {
    int n = f->getDimension();
    ocra_assert(weight.size() == n);
    VectorXd w(n);
    double di = 1./fi;
    for (int i=0; i<n; ++i)
      w[i] = weight[i] * di;
    return createInequalityTaskWithLinearLaw(f, w);
  }


  LinearTask* LinearTask::createInequalityTaskWithLinearLaw(Function* f, const VectorXd& weight, const VectorXd& fi)
  {
    int n = f->getDimension();
    ocra_assert(fi.size() == n);
    ocra_assert(weight.size() == n);
    VectorXd w(n);
    for (int i=0; i<n; ++i)
      w[i] = weight[i] / fi[i];
    return createInequalityTaskWithLinearLaw(f, w);
  }


  LinearTask* LinearTask::createInequalityTaskWithLinearLaw(Function* f, const VectorXd& weightDividedFi)
  {
    ocra_assert(weightDividedFi.size() == f->getDimension());
    Variable* v = utils::createOutputVariable(*f);
    int n = v->getSize();
    VectorXd w(weightDividedFi);
    w*=-1;
    LinearFunction* L = new DiagonalLinearFunction(*v, w, 0., true, weightDividedFi[n-1]);
    return new LinearTask(*f, *L);
  }

  void LinearTask::updateJacobian() const
  {
    _jacobian = _f->getJacobian();
  }


  void LinearTask::doUpdateInputSizeBegin()
  {
    //do nothing : this overload allows to resize
  }

  void LinearTask::doUpdateDimensionBegin(int newDimension)
  {
    //do nothing : this overload allows to resize
  }

  void LinearTask::doUpdateDimensionEnd(int oldDimension)
  {
    assert(_L->getVariable().isBaseVariable() && "only base variable can be resized");
    static_cast<BaseVariable*>(&_L->getVariable())->resize(_f->getDimension());
    LinearFunction::doUpdateDimensionEnd(oldDimension);  //to resize _b
  }

  void LinearTask::updateDimension(int timestamp)
  {
    changeFunctionDimension(_f->getDimension());
  }

}

namespace ocra
{
  void testLinearTask()
  {
    int n=1;
    BaseVariable x("x", n);

    BaseVariable fx("fx", 2*n);

    MatrixXd id(2*n,n);
    MatrixXd gain(2*n,2*n);
    VectorXd limit(2*n);
    VectorXd zero(2*n);
    id.setZero();
    gain.setZero();
    zero.setZero();
    for (int i=0; i<n; ++i)
    {
      id(i,i) = 1;
      id(n+i,i) = 1;
      limit[i] = -i-1;    // -upper bound
      limit[n+i] = -i-1;   //lower bound
      gain(i,i) = 1./(i+1);
      gain(n+i,n+i) = 1./(n+i+1);
    }
    LinearFunction JointLimit(x, id, limit);
    LinearFunction TaskGain(fx, gain, zero);

    LinearTask JointTask(JointLimit, TaskGain);

    VectorXd v(n);
    for (int i=0; i<n; ++i)
      std::cin >> v[i];
    x.setValue(v);
    VectorXd dv(n);
    for (int i=0; i<n; ++i)
      std::cin >> dv[i];
    x.getTimeDerivative().setValue(dv);

    std::cout << JointTask.getValue() << std::endl;
  }
}

// cmake:sourcegroup=Function


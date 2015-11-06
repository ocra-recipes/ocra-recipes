#include "SecondOrderLinearTask.h"
#include "FunctionUtilities.h"
#include "DoubleDiagonalLinearFunction.h"
#include <sstream>

namespace ocra
{
  SecondOrderLinearTask::SecondOrderLinearTask(Function& f, Function& L)
    :NamedInstance("second order linear task")
    ,AbilitySet(PARTIAL_X)
    ,CoupledInputOutputSize(false)
    ,LinearFunction(f.getVariable().getTimeDerivative().getTimeDerivative(), f.getDimension())
    , _f(&f)
    , _L(&L)
    , _saturate(false)
  {
    //check
    //ocra_assert(f.canCompute<FUN_DOT>() && "function f needs to provide its time derivative.");
    ocra_assert(L.getVariable().getNumberOfChildren() == 2);

    if (f.getDimension()*2 != L.getVariable().getSize())
      throw std::runtime_error("[ocra::LinearTask::LinearTask] Input size of L doesn't match with dimension of f");
    if (f.getDimension() != L.getDimension())
      throw std::runtime_error("[ocra::LinearTask::LinearTask] Dimension of L doesn't match with dimension of f");
    
    //dependencies
    f.connect<EVT_CHANGE_VALUE>(*this, &LinearFunction::invalidateb);
    f.connect<EVT_CHANGE_VALUE>(*this, &Function::invalidateAll);
    f.connect<EVT_RESIZE>(*this, &SecondOrderLinearTask::updateDimension);
    L.connect<EVT_CHANGE_VALUE>(*this, &LinearFunction::invalidateb);
  }

  SecondOrderLinearTask::~SecondOrderLinearTask()
  {
    _f->disconnect<EVT_CHANGE_VALUE>(*this, &LinearFunction::invalidateb);
    _f->disconnect<EVT_CHANGE_VALUE>(*this, &Function::invalidateAll);
    _f->disconnect<EVT_RESIZE>(*this, &SecondOrderLinearTask::updateDimension);
    _L->disconnect<EVT_CHANGE_VALUE>(*this, &LinearFunction::invalidateb);
  }

  void SecondOrderLinearTask::updateb() const
  {
    inhibitPropagationFromb();
    CompositeVariable* v = static_cast<CompositeVariable*>(&_L->getVariable());
    (*v)(0).setValue(_f->getValue());
    if (_f->canCompute<FUN_DOT>())
      (*v)(1).setValue(_f->get<FUN_DOT>());
    else
      (*v)(1).setValue(_f->getJacobian()*_f->getVariable().getValue());
    desinhibitPropagationFromb();
    _b = _L->getValue();

    if (_saturate)
    {
      double max = _b.cwiseAbs().maxCoeff();
      if (max>_accelerationMax)
      {
        double factor = _accelerationMax/max;
        _b *= factor;
      }
    }

    if (_f->canCompute<PARTIAL_X_DOT_X_DOT>())
      _b += _f->get<PARTIAL_X_DOT_X_DOT>();
    else if (_f->canCompute<PARTIAL_X_DOT>())
      _b += _f->get<PARTIAL_X_DOT>()*_f->getVariable().getTimeDerivative().getValue();
  }


  SecondOrderLinearTask* SecondOrderLinearTask::createSecondOrderLinearTaskWithLinearLaw(Function* f, real weight0, real weight1)
  {
    VectorXd w0(f->getDimension());
    VectorXd w1(f->getDimension());
    w0.fill(weight0);
    w1.fill(weight1);
    return createSecondOrderLinearTaskWithLinearLaw(f, w0, w1);
  }


  SecondOrderLinearTask* SecondOrderLinearTask::createSecondOrderLinearTaskWithLinearLaw(Function* f, const VectorXd& weight0, const VectorXd& weight1)
  {
    Variable* v1 = utils::createOutputVariable(*f);
    Variable* v2 = utils::createOutputVariable(*f);
    int n = v1->getSize();
    std::stringstream vname;
    vname << v1->getName() << v2->getName();
    CompositeVariable* v = new CompositeVariable(vname.str(),*v1,*v2);
    LinearFunction* L = new DoubleDiagonalLinearFunction(*v, weight0, weight1, 0., true, weight0[n-1], weight1[n-1]);
    return new SecondOrderLinearTask(*f, *L);
  }

  void SecondOrderLinearTask::updateJacobian() const
  {
    _jacobian = _f->getJacobian();
  }

  void SecondOrderLinearTask::doUpdateInputSizeBegin()
  {
    //do nothing : this overload allows to resize
  }

  void SecondOrderLinearTask::doUpdateDimensionBegin(int newDimension)
  {
    //do nothing : this overload allows to resize
  }

  void SecondOrderLinearTask::doUpdateDimensionEnd(int oldDimension)
  {
    assert(!_L->getVariable().isBaseVariable() && "Variable of L is supposed to be a composite variable with two base variables.");
    CompositeVariable* v = static_cast<CompositeVariable*>(&_L->getVariable());
    assert((*v)(0).isBaseVariable() && "only base variables can be resized");
    assert((*v)(1).isBaseVariable() && "only base variables can be resized");
    static_cast<BaseVariable*>(&(*v)(0))->resize(_f->getDimension());
    static_cast<BaseVariable*>(&(*v)(1))->resize(_f->getDimension());
    LinearFunction::doUpdateDimensionEnd(oldDimension);  //to resize _b
  }

  void SecondOrderLinearTask::updateDimension(int timestamp)
  {
    changeFunctionDimension(_f->getDimension());
  }
}

// cmake:sourcegroup=Function

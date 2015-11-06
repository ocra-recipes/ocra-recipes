#include "WeightedSquaredSumOfControlEquationsFunction.h"


namespace ocra
{
  WeightedSquaredSumOfControlEquationsFunction::WeightedSquaredSumOfControlEquationsFunction(Model& model)
    :NamedInstance("weighted square sum of control equations function")
    ,AbilitySet(PARTIAL_X, PARTIAL_XX)
    ,CoupledInputOutputSize(false)
    ,QuadraticFunction(model.getAccelerationVariable())
    ,_model(model)
    ,_W(model.nbDofs())
    ,_Wf(model.nbDofs())
    ,_WN(model.nbDofs())
    ,_tempAreUpToDate(false)
  {
    _W.setZero();
    inhibitPropagationFrom_q_or_r();
  }

  WeightedSquaredSumOfControlEquationsFunction::WeightedSquaredSumOfControlEquationsFunction(
                                                  const ControlEquationFunction& f, const VectorXd& weight)
    :NamedInstance("weighted square sum of control equations function")
    ,AbilitySet(PARTIAL_X, PARTIAL_XX)
    ,CoupledInputOutputSize(false)
    ,QuadraticFunction(f.getModel().getAccelerationVariable())
    ,_model(f.getModel())
    ,_W(_model.nbDofs())
    ,_Wf(_model.nbDofs())
    ,_WN(_model.nbDofs())
    ,_tempAreUpToDate(false)
  {
    _W.setZero();
    inhibitPropagationFrom_q_or_r();
    add(f, weight);
  }

  WeightedSquaredSumOfControlEquationsFunction::WeightedSquaredSumOfControlEquationsFunction(
                                                  const ControlEquationFunction& f1, const VectorXd& weight1,
                                                  const ControlEquationFunction& f2, const VectorXd& weight2)
    :NamedInstance("weighted square sum of control equations function")
    ,AbilitySet(PARTIAL_X, PARTIAL_XX)
    ,CoupledInputOutputSize(false)
    ,QuadraticFunction(f1.getModel().getAccelerationVariable())
    ,_model(f1.getModel())
    ,_W(_model.nbDofs())
    ,_Wf(_model.nbDofs())
    ,_WN(_model.nbDofs())
    ,_tempAreUpToDate(false)
  {
    _W.setZero();
    inhibitPropagationFrom_q_or_r();
    add(f1, weight1);
    add(f2, weight2);
  }

  WeightedSquaredSumOfControlEquationsFunction::~WeightedSquaredSumOfControlEquationsFunction()
  {
    while (_functions.size())
      remove(_functions.back().getFunction());
  }

  WeightedSquaredSumOfControlEquationsFunction& WeightedSquaredSumOfControlEquationsFunction::add(const ControlEquationFunction& f, const VectorXd& weight)
  {
    ocra_assert(&f.getModel() == &_model && "the function to be added is not a control equation on the same model as the others.");
    ocra_assert(f.isUsingNLTerms() && "current implementation consider all functions are using non-linear terms");
    ocra_assert(f.getDimension() == weight.size());
    ocra_assert(!contains(f) && "this function was already added to the sum before");

    _functions.push_back(WeightAndFunction(f, weight));
    connect(f);
    _W += weight;
    invalidateTmp(0);

    return *this;
  }

  WeightedSquaredSumOfControlEquationsFunction& WeightedSquaredSumOfControlEquationsFunction::remove(const ControlEquationFunction& f)
  {
    ocra_assert(contains(f) && "this function was not added to the sum before");

    fun_iterator it = find(f);
    _W -= it->getWeight();
    _functions.erase(it);
    disconnect(f);
    invalidateTmp(0);

    return *this;
  }

  void WeightedSquaredSumOfControlEquationsFunction::changeFunctionWeight(const ControlEquationFunction& f, const VectorXd& weight)
  {
    ocra_assert(contains(f) && "this function was not added to the sum before");
    ocra_assert(f.getDimension() == weight.size());
    fun_iterator it = find(f);
    _W -= it->getWeight();
    it->changeWeight(weight);
    _W += weight;
    invalidateTmp(0);
  } 

  const Model& WeightedSquaredSumOfControlEquationsFunction::getModel() const
  {
    return _model;
  }

  void WeightedSquaredSumOfControlEquationsFunction::updateHessian()  const
  {
    (*IFunction<PARTIAL_XX>::_val[0]).noalias() = _model.getInertiaMatrix().transpose() * _W.asDiagonal() * _model.getInertiaMatrix();
  }

  void WeightedSquaredSumOfControlEquationsFunction::updateq()  const
  {
    updateTmp();
    (*_q[0]) = _model.getInertiaMatrix().transpose()*(_WN - _Wf);
  }

  void WeightedSquaredSumOfControlEquationsFunction::updater()  const
  {
    updateTmp();
    _r[0] = 0;
    //r = 0.5*sum(f_i(x)^t*W_i*f_i(x)) + 0.5*N^t*W*N - N^t*sum(W_i*f_i(x))
    for (size_t i=0; i<_functions.size(); ++i)
        _r[0] += _functions[i].getFunction().getValue().dot(_functions[i].getWeight().asDiagonal()*_functions[i].getFunction().getValue());
    _r[0] *= 0.5;

    _r[0] += 0.5*_model.getNonLinearTerms().dot(_WN) - _model.getNonLinearTerms().dot(_Wf);
  }

  void WeightedSquaredSumOfControlEquationsFunction::doChangePi(const MatrixXd& Pi, int index)
  {
    ocra_assert(false && "It has no meaning to change Pi in WeightedSquaredSumOfControlEquationsFunction and it would have no effect");
  }

  void WeightedSquaredSumOfControlEquationsFunction::doChangeqi(const VectorXd& qi, int index)
  {
    ocra_assert(false && "It has no meaning to change ri in WeightedSquaredSumOfControlEquationsFunction and it would have no effect");
  }

  void WeightedSquaredSumOfControlEquationsFunction::doChangeri(double ri, int index)
  {
    ocra_assert(false && "It has no meaning to change qi in WeightedSquaredSumOfControlEquationsFunction and it would have no effect");
  }




    // ------------------------ private methods ---------------------------------
  WeightedSquaredSumOfControlEquationsFunction::fun_iterator 
    WeightedSquaredSumOfControlEquationsFunction::find(const ControlEquationFunction& f)
  {
    fun_iterator it;
    for (it = _functions.begin(); it!=_functions.end(); ++it)
    {
      if (it->functionEquals(f))
        break;
    }
    return it;
  }

  WeightedSquaredSumOfControlEquationsFunction::const_fun_iterator 
    WeightedSquaredSumOfControlEquationsFunction::find(const ControlEquationFunction& f) const
  {
    const_fun_iterator it;
    for (it = _functions.begin(); it!=_functions.end(); ++it)
    {
      if (it->functionEquals(f))
        break;
    }
    return it;
  }

  void WeightedSquaredSumOfControlEquationsFunction::invalidateTmp(int timestamp)
  {
    _tempAreUpToDate = false;
  }

  void WeightedSquaredSumOfControlEquationsFunction::updateTmp() const
  {
    if (!_tempAreUpToDate)
    {
      _Wf.setZero();
      for (size_t i=0; i<_functions.size(); ++i)
        _Wf.noalias() +=  _functions[i].getWeight().asDiagonal() * _functions[i].getFunction().getValue();

      _WN = _W.asDiagonal() * _model.getNonLinearTerms();

      _tempAreUpToDate = true;
    }
  }

  void WeightedSquaredSumOfControlEquationsFunction::connect(const ControlEquationFunction& f)
  {
    f.connect<EVT_CHANGE_VALUE>(*this, &WeightedSquaredSumOfControlEquationsFunction::invalidateAll);
    f.connect<EVT_CHANGE_VALUE>(*this, &WeightedSquaredSumOfControlEquationsFunction::invalidateq);
    f.connect<EVT_CHANGE_VALUE>(*this, &WeightedSquaredSumOfControlEquationsFunction::invalidater);
    f.connect<EVT_CHANGE_VALUE>(*this, &WeightedSquaredSumOfControlEquationsFunction::invalidateTmp);
  }

  void WeightedSquaredSumOfControlEquationsFunction::disconnect(const ControlEquationFunction& f)
  {
    f.disconnect<EVT_CHANGE_VALUE>(*this, &WeightedSquaredSumOfControlEquationsFunction::invalidateAll);
    f.disconnect<EVT_CHANGE_VALUE>(*this, &WeightedSquaredSumOfControlEquationsFunction::invalidateq);
    f.disconnect<EVT_CHANGE_VALUE>(*this, &WeightedSquaredSumOfControlEquationsFunction::invalidater);
    f.disconnect<EVT_CHANGE_VALUE>(*this, &WeightedSquaredSumOfControlEquationsFunction::invalidateTmp);
  }
}

// cmake:sourcegroup=Functions

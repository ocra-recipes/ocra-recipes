#include "ControlEquationFunction.h"


namespace ocra
{
  ControlEquationFunction::ControlEquationFunction(Model& model, const Function& f, bool useNLTerms)
    :NamedInstance("force tracking function")
    ,AbilitySet(PARTIAL_X)
    ,CoupledInputOutputSize(false)
    ,LinearFunction(model.getAccelerationVariable(), model.nbDofs())
    ,_useNLTerms(useNLTerms)
    ,_model(model)
    ,_f(f)
  {
    //check
    if (_dim != f.getDimension())
      throw std::runtime_error("[ocra::ControlEquationFunction::ControlEquationFunction] dimension of f does not match the number of dofs of the model");

    //dependencies
    _f.connect<EVT_CHANGE_VALUE>(*this, &LinearFunction::invalidateb);
    _f.connect<EVT_CHANGE_VALUE>(*this, &Function::invalidateAll);
    _model.connect<EVT_CHANGE_VALUE>(*this, &LinearFunction::invalidateb);
    _model.connect<EVT_CHANGE_VALUE>(*this, &Function::invalidateAll);
  }


  ControlEquationFunction::~ControlEquationFunction()
  {
    _f.disconnect<EVT_CHANGE_VALUE>(*this, &LinearFunction::invalidateb);
    _f.disconnect<EVT_CHANGE_VALUE>(*this, &Function::invalidateAll);
    _model.disconnect<EVT_CHANGE_VALUE>(*this, &LinearFunction::invalidateb);
    _model.disconnect<EVT_CHANGE_VALUE>(*this, &Function::invalidateAll);
  }

  bool ControlEquationFunction::isUsingNLTerms() const
  {
    return _useNLTerms;
  }

  const Model& ControlEquationFunction::getModel() const
  {
    return _model;
  }

  Model& ControlEquationFunction::getModel()
  {
    return _model;
  }

  const Function& ControlEquationFunction::getFunction() const
  {
    return _f;
  }


  void ControlEquationFunction::updateJacobian() const
  {
    _jacobian = _model.getInertiaMatrix();
  }

  void ControlEquationFunction::updateb() const
  {
    _b = _model.getNonLinearTerms() - _f.getValue();
  }

  void ControlEquationFunction::doChangeA(const MatrixXd& A)
  {
    ocra_assert(false && "It has no meaning to change A in ControlEquationFunction and it would have no effect");
  }

  void ControlEquationFunction::doChangeb(const VectorXd& b)
  {
    ocra_assert(false && "It has no meaning to change b in ControlEquationFunction and it would have no effect");
  }
}

// cmake:sourcegroup=Functions

#include "DynamicEquationFunction.h"
#include "ocra/control/Model.h"
#include "ocra/optim/Variable.h"
#include <sstream>

namespace ocra
{
  DynamicEquationFunction::DynamicEquationFunction(const Model& model)
    :NamedInstance("Dynamic Equation Function")
    ,AbilitySet(PARTIAL_X)
    ,CoupledInputOutputSize(false)
    ,LinearFunction(createDEVariable(model), model.nbDofs())
    , _model(model), _q_ddot(model.getAccelerationVariable()), _tau(model.getJointTorqueVariable())
    , _f(model.getModelContacts().getContactForcesVariable())
  {
    buildA();

    model.connect<EVT_CHANGE_VALUE>(*this, &DynamicEquationFunction::invalidateAll);
    model.connect<EVT_CHANGE_VALUE>(*this, &DynamicEquationFunction::invalidateb);
  }

  DynamicEquationFunction::~DynamicEquationFunction()
  {
    _model.disconnect<EVT_CHANGE_VALUE>(*this, &DynamicEquationFunction::invalidateAll);
    _model.disconnect<EVT_CHANGE_VALUE>(*this, &DynamicEquationFunction::invalidateb);
    delete &getVariable();
  }

  Variable& DynamicEquationFunction::createDEVariable(const Model& model)
  {
    static int cpt = 0;
    std::stringstream name;
    name << "de" << cpt++;
    CompositeVariable* var = new CompositeVariable(name.str(), model.getAccelerationVariable());
    var->add(model.getJointTorqueVariable());
    var->add(model.getModelContacts().getContactForcesVariable());
    return *var;
  }

  void DynamicEquationFunction::updateb() const
  {
    _b = _model.getLinearTerms() + _model.getNonLinearTerms();
  }

  void DynamicEquationFunction::updateJacobian() const
  {
    //A = (M -L J^t)
    _jacobian.block(0, 0, _dim, _dim) = _model.getInertiaMatrix();
    if (_f.getSize()>0)
      _jacobian.block(0, _dim+_tau.getSize(), _dim, _f.getSize()) = _model.getModelContacts().getJct();
  }

  void DynamicEquationFunction::buildA()
  {
    _jacobian.setZero();
    int index = _model.hasFixedRoot() ? 0 : 6;
    _jacobian.block(0, _dim, _dim, _tau.getSize()).setZero();
    _jacobian.block(index, _dim, _tau.getSize(), _tau.getSize()).diagonal() = -_model.getActuatedDofs();
  }

  void DynamicEquationFunction::doUpdateInputSizeBegin()
  {
    //do nothing : this overload allows to resize
  }

  void DynamicEquationFunction::doUpdateInputSizeEnd(void)
  {
    buildA();
  }
}

// cmake:sourcegroup=Functions

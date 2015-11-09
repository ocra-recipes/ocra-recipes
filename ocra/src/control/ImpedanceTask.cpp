#include "ocra/control/ImpedanceTask.h"

#include "ocra/control/Model.h"
#include "ocra/control/Feature.h"

#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/SquaredLinearFunction.h"
#include "ocra/optim/LinearizedCoulombFunction.h"
#include "ocra/optim/QuadraticSolver.h"
#include "ocra/optim/WeightedSquareDistanceFunction.h"
#include "ocra/optim/SumOfLinearFunctions.h"

#define CHECK_SOLVER_SET(funname) \
  if(!pimpl->solver) \
  throw std::runtime_error("["#funname"] task " + getName() + " solver not set. Has the task been added to a controller?");

namespace ocra
{
  struct ImpedanceTask::Pimpl
  {
    const Model& model;
    QuadraticSolver* solver;
    BaseVariable contactVar;
    EqualZeroConstraintPtr<LinearFunction> taskAsConstraint;
    ObjectivePtr<SquaredLinearFunction> taskAsObjective;
    ObjectivePtr<WeightedSquareDistanceFunction> minVarObjective;
    EqualZeroConstraintPtr<LinearFunction> contactConstraint;
    LessThanZeroConstraintPtr<LinearizedCoulombFunction> frictionConstraint;

    Pimpl(const std::string& name, const Model& m, const Feature& s)
      : model(m)
      , solver(0x0)
      , contactVar("fc", 3)
      , taskAsConstraint( new LinearFunction(m.getAccelerationVariable(), MatrixXd::Zero(s.getDimension(), model.nbDofs()), VectorXd::Zero(s.getDimension())) )
      , taskAsObjective( new SquaredLinearFunction(&taskAsConstraint.getFunction()) )
      , minVarObjective( new WeightedSquareDistanceFunction(contactVar, 1.e-6, Vector3d::Zero()) )
      , contactConstraint( new LinearFunction(m.getAccelerationVariable(), MatrixXd::Zero(s.getDimension(), model.nbDofs()), VectorXd::Zero(s.getDimension())) )
      , frictionConstraint( new LinearizedCoulombFunction(contactVar, 1., 4, 0.) )
    {
    }
  };

  ImpedanceTask::ImpedanceTask(const std::string& name, const Model& model, const Feature& feature, const Feature& featureDes)
    : Task(name, model, feature, featureDes)
    , pimpl(new Pimpl(name, model, feature))
  {
  }

  ImpedanceTask::ImpedanceTask(const std::string& name, const Model& model, const Feature& feature)
    : Task(name, model, feature)
    , pimpl(new Pimpl(name, model, feature))
  {
  }

  void ImpedanceTask::setSolver(QuadraticSolver& solver)
  {
    pimpl->solver = &solver;
  }

  void ImpedanceTask::update()
  {
    const MatrixXd& Ainv = getDesiredMassInverse();
    const MatrixXd& Kp = getStiffness();
    const MatrixXd& Kd = getDamping();

    const VectorXd& e = getFeatureDes() ? getFeature().computeError(*getFeatureDes()) : getFeature().computeError();
    const VectorXd& edot = getFeatureDes() ? getFeature().computeErrorDot(*getFeatureDes()) : getFeature().computeErrorDot();
    const VectorXd& fd = getFeatureDes() ? getFeature().computeEffort(*getFeatureDes()) : getFeature().computeEffort();
    VectorXd f = Ainv * (Kp * e + Kd * edot + fd); // There would also ideally be a Jdot T term
    pimpl->taskAsConstraint.getFunction().changeb(f + getErrorDdot());

    const MatrixXd& J = getFeatureDes() ? getFeature().computeJacobian(*getFeatureDes()) : getFeature().computeJacobian();
    pimpl->taskAsConstraint.getFunction().changeA(J);
    pimpl->contactConstraint.getFunction().changeA(J);
  }

  void ImpedanceTask::doActivateAsObjective()
  {
    CHECK_SOLVER_SET(ImpedanceTask_doActivateAsObjective);
    if(isBodyContactConstraint() || isPointContactTask())
      return;
    else
      pimpl->solver->addObjective(pimpl->taskAsObjective.getObjective());
  }

  void ImpedanceTask::doActivateAsConstraint()
  {
    CHECK_SOLVER_SET(ImpedanceTask_doActivateAsConstraint);
    if(isBodyContactConstraint() || isPointContactTask())
      return;
    else
      pimpl->solver->addConstraint(pimpl->taskAsConstraint.getConstraint());
  }

  void ImpedanceTask::doActivateContactMode()
  {
    CHECK_SOLVER_SET(ImpedanceTask_doActivateContactMode);
    if(isBodyContactConstraint())
      pimpl->solver->addConstraint(pimpl->contactConstraint.getConstraint());
    else
    {
      pimpl->solver->addObjective(pimpl->minVarObjective.getObjective());
      pimpl->solver->addConstraint(pimpl->frictionConstraint.getConstraint());
      pimpl->model.getModelContacts().addContactPoint(pimpl->contactVar, getFeature());
    }
  }

  void ImpedanceTask::doDeactivateAsObjective()
  {
    CHECK_SOLVER_SET(ImpedanceTask_doDeactivateAsObjective);
    if(isBodyContactConstraint() || isPointContactTask())
      return;
    else
      pimpl->solver->removeObjective(pimpl->taskAsObjective.getObjective());
  }

  void ImpedanceTask::doDeactivateAsConstraint()
  {
    CHECK_SOLVER_SET(ImpedanceTask_doDeactivateAsConstraint);
    if(isBodyContactConstraint() || isPointContactTask())
      return;
    else
      pimpl->solver->removeConstraint(pimpl->taskAsConstraint.getConstraint());
  }

  void ImpedanceTask::doDeactivateContactMode()
  {
    CHECK_SOLVER_SET(ImpedanceTask_doDeactivateContactMode);
    if(isBodyContactConstraint())
      pimpl->solver->removeConstraint(pimpl->contactConstraint.getConstraint());
    else
    {
      pimpl->solver->removeObjective(pimpl->minVarObjective.getObjective());
      pimpl->solver->removeConstraint(pimpl->frictionConstraint.getConstraint());
      pimpl->model.getModelContacts().removeContactPoint(pimpl->contactVar);
    }
  }

  void ImpedanceTask::doSetWeight()
  {
    pimpl->taskAsObjective.getFunction().changeWeight(getWeight());
  }

  void ImpedanceTask::doSetFrictionCoeff()
  {
    pimpl->frictionConstraint.getFunction().setFrictionCoeff(getFrictionCoeff());
  }

  void ImpedanceTask::doSetMargin()
  {
    pimpl->frictionConstraint.getFunction().setMargin(getMargin());
  }

  void ImpedanceTask::doGetOutput(VectorXd& output) const
  {
    output = pimpl->contactVar.getValue();
  }
}

// cmake:sourcegroup=Controllers

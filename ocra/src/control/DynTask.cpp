#include "DynTask.h"

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
  struct DynTask::Pimpl
  {
    const Model& model;

    BaseVariable var;

    QuadraticSolver* solver;
    SumOfLinearFunctions* eqFunction;

		boost::shared_ptr<LinearFunction> accObjectiveFunction;
    ObjectivePtr<SquaredLinearFunction> accTaskAsObjective;
    ObjectivePtr<WeightedSquareDistanceFunction> forceTaskAsObjective;
    EqualZeroConstraintPtr<LinearFunction> forceTaskAsConstraint;
    ObjectivePtr<WeightedSquareDistanceFunction> minVarObjective;
    EqualZeroConstraintPtr<LinearFunction> bodyContactConstraint;
    LessThanZeroConstraintPtr<LinearizedCoulombFunction> frictionConstraint;
    boost::shared_ptr<LinearFunction> eqSubFunction;

    Pimpl(const std::string& name, const Model& m, const Feature& s)
      : model(m)
      , var(name+".var", s.getDimension())
      , solver(0x0)
      , eqFunction(0x0)
			, accObjectiveFunction( new LinearFunction(m.getAccelerationVariable(), MatrixXd::Zero(s.getDimension(), model.nbDofs()), VectorXd::Zero(s.getDimension())) )
      , accTaskAsObjective( new SquaredLinearFunction(accObjectiveFunction.get()) )
      , forceTaskAsObjective( new WeightedSquareDistanceFunction(var, 1., VectorXd::Zero(s.getDimension())) )
      , forceTaskAsConstraint( new LinearFunction(var, MatrixXd::Identity(s.getDimension(), s.getDimension()), VectorXd::Zero(s.getDimension())) )
      , minVarObjective( new WeightedSquareDistanceFunction(var, 1.e-12, VectorXd::Zero(s.getDimension())) )
      , bodyContactConstraint( new LinearFunction(m.getAccelerationVariable(), MatrixXd::Zero(s.getDimension(), model.nbDofs()), VectorXd::Zero(s.getDimension())) )
      , frictionConstraint()
      , eqSubFunction( buildSubFunction(var, m, s) )
    {
      if(var.getSize() == 3) // candidate for contact mode
        frictionConstraint.set( new LinearizedCoulombFunction(var, 1., 4, 0.) );
    }

    static LinearFunction* buildSubFunction(Variable& var, const Model& m, const Feature& s)
    {
      const int n = m.nbInternalDofs();
      const int dim = var.getSize();
      return new LinearFunction( var, MatrixXd::Zero(n, dim), VectorXd::Zero(n) );
    }
  };

  DynTask::DynTask(const std::string& name, const Model& model, const Feature& feature, const Feature& featureDes)
    : Task(name, model, feature, featureDes)
    , pimpl(new Pimpl(name, model, feature))
  {
  }

  DynTask::DynTask(const std::string& name, const Model& model, const Feature& feature)
    : Task(name, model, feature)
    , pimpl(new Pimpl(name, model, feature))
  {
  }

  void DynTask::setSolver(QuadraticSolver& solver, SumOfLinearFunctions& eqFunction)
  {
    pimpl->solver = &solver;
    pimpl->eqFunction = &eqFunction;
  }

  void DynTask::update()
  {
    const MatrixXd& J = getJacobian();
    const MatrixXd& Ainv = getDesiredMassInverse();
    const MatrixXd& Kp = getStiffness();
    const MatrixXd& Kd = getDamping();
    const VectorXd f = -Kp * getError() - Kd * getErrorDot() - getEffort();
    const MatrixXd Jt_art = J.transpose().bottomRows(pimpl->eqSubFunction->getDimension());
		const VectorXd accobj = -Ainv * f + getErrorDdot();

    pimpl->bodyContactConstraint.getFunction().changeA(J);

    pimpl->accObjectiveFunction->changeA(J);
    pimpl->accObjectiveFunction->changeb(accobj);

    pimpl->forceTaskAsObjective.getFunction().changeReference(f);
    pimpl->forceTaskAsConstraint.getFunction().changeb(-f);

    pimpl->eqSubFunction->changeA(Jt_art);
  }

  void DynTask::doActivateAsObjective()
  {
    CHECK_SOLVER_SET(DynTask_doActivateAsObjective);

    if(isBodyContactConstraint())
      return;
    
    if(!isPointContactTask())
		{
      pimpl->solver->addObjective(pimpl->accTaskAsObjective);
			pimpl->solver->addObjective(pimpl->forceTaskAsObjective);
		}

    pimpl->eqFunction->addFunction(*pimpl->eqSubFunction);
  }

  void DynTask::doActivateAsConstraint()
  {
    CHECK_SOLVER_SET(DynTask_doActivateAsConstraint);

    if(isBodyContactConstraint())
      return;
    
    if(!isPointContactTask())
		{
      pimpl->solver->addObjective(pimpl->accTaskAsObjective);
			pimpl->solver->addConstraint(pimpl->forceTaskAsConstraint);
			pimpl->solver->addObjective(pimpl->minVarObjective);
		}

    pimpl->eqFunction->addFunction(*pimpl->eqSubFunction);
  }

  void DynTask::doActivateContactMode()
  {
    CHECK_SOLVER_SET(DynTask_doActivateContactMode);

    if(isBodyContactConstraint())
      pimpl->solver->addConstraint(pimpl->bodyContactConstraint);
    else // isPointContactTask() ensured by mother class
    {
      pimpl->solver->addObjective(pimpl->minVarObjective);
      pimpl->solver->addConstraint(pimpl->frictionConstraint);
      pimpl->model.getModelContacts().addContactPoint(pimpl->var, getFeature());
    }
  }

  void DynTask::doDeactivateAsObjective()
  {
    CHECK_SOLVER_SET(DynTask_doDeactivateAsObjective);

    if(isBodyContactConstraint())
      return;

    if(!isPointContactTask())
		{
      pimpl->solver->removeObjective(pimpl->accTaskAsObjective.getObjective());
			pimpl->solver->removeObjective(pimpl->forceTaskAsObjective.getObjective());
		}

    pimpl->eqFunction->removeFunction(*pimpl->eqSubFunction);
  }

  void DynTask::doDeactivateAsConstraint()
  {
    CHECK_SOLVER_SET(DynTask_doDeactivateAsConstraint);

    if(isBodyContactConstraint())
      return;

    if(!isPointContactTask())
		{
      pimpl->solver->removeObjective(pimpl->accTaskAsObjective.getObjective());
			pimpl->solver->removeConstraint(pimpl->forceTaskAsConstraint.getConstraint());
			pimpl->solver->removeObjective(pimpl->minVarObjective.getObjective());
		}

    pimpl->eqFunction->removeFunction(*pimpl->eqSubFunction);
  }

  void DynTask::doDeactivateContactMode()
  {
    CHECK_SOLVER_SET(DynTask_doDeactivateContactMode);

    if(isBodyContactConstraint())
      pimpl->solver->removeConstraint(pimpl->bodyContactConstraint.getConstraint());
    else // isPointContactTask() ensured by mother class
    {
      pimpl->solver->removeObjective(pimpl->minVarObjective.getObjective());
      pimpl->solver->removeConstraint(pimpl->frictionConstraint);
      pimpl->model.getModelContacts().removeContactPoint(pimpl->var);
    }
  }

  void DynTask::doSetWeight()
  {
    pimpl->accTaskAsObjective.getFunction().changeWeight(getWeight());
    pimpl->forceTaskAsObjective.getFunction().changeWeight(getWeight());
  }

  void DynTask::doSetFrictionCoeff()
  {
    pimpl->frictionConstraint.getFunction().setFrictionCoeff(getFrictionCoeff());
  }

  void DynTask::doSetMargin()
  {
    pimpl->frictionConstraint.getFunction().setMargin(getMargin());
  }

  void DynTask::doGetOutput(VectorXd& output) const
  {
    output = VectorXd::Zero(getDimension());
  }
}

// cmake:sourcegroup=Controllers

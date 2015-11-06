#include "SFCTask.h"

#include "ocra/control/Feature.h"
#include "ocra/control/Model.h"

#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/LinearFunction.h"
#include "ocra/optim/WeightedSquareDistanceFunction.h"
#include "ocra/optim/LinearizedCoulombFunction.h"
#include "ocra/optim/QuadraticSolver.h"
#include "ocra/optim/SumOfLinearFunctions.h"

#define CHECK_SOLVER_SET(funname) \
  if(!pimpl->solver) \
  throw std::runtime_error("["#funname"] task " + getName() + " solver not set. Has the task been added to a controller?");

namespace ocra
{
  struct SFCTask::Pimpl
  {
    QuadraticSolver* solver;
    SumOfLinearFunctions* seConstraint;
    BaseVariable var;
    boost::shared_ptr<LinearFunction> taskSEConstraintFunction;
    ObjectivePtr<WeightedSquareDistanceFunction> taskAsObjective;
    ObjectivePtr<WeightedSquareDistanceFunction> minVarObjective;
    EqualZeroConstraintPtr<LinearFunction> taskAsConstraint;
		boost::shared_ptr<LinearizedCoulombFunction> frictionFunction;
    LessThanZeroConstraintPtr<LinearFunction> frictionConstraint;

    Pimpl(const std::string& name, const Model& m, const Feature& s)
      : solver(0x0)
      , var(name+".var", s.getDimension())
      , taskSEConstraintFunction(buildSEFunction(var, m, s))
      , taskAsObjective( new WeightedSquareDistanceFunction(var, 1., VectorXd::Zero(s.getDimension())) )
      , minVarObjective( new WeightedSquareDistanceFunction(var, 1., VectorXd::Zero(s.getDimension())) )
      , taskAsConstraint( new LinearFunction(var, MatrixXd::Identity(s.getDimension(), s.getDimension()), VectorXd::Zero(s.getDimension())) )
      , frictionFunction()
      , frictionConstraint()
    {
			if(var.getSize() == 3) { // candidate for contact mode
				frictionFunction.reset( new LinearizedCoulombFunction(var, 1., 4, 0.) );

        const int outDim = frictionFunction->getDimension();
        const int inDim = 3;
        frictionConstraint.set( new LinearFunction(var, MatrixXd::Identity(outDim, inDim), VectorXd::Zero(outDim)) );
			}
    }

    static LinearFunction* buildSEFunction(Variable& var, const Model& m, const Feature& s)
    {
      return new LinearFunction( var, MatrixXd::Zero(m.nbDofs() - m.nbInternalDofs(), var.getSize()), VectorXd::Zero(m.nbDofs() - m.nbInternalDofs()) );
    }
  };

  SFCTask::SFCTask(const std::string& name, const Model& model, const Feature& feature, const Feature& featureDes)
    : Task(name, model, feature, featureDes)
    , pimpl(new Pimpl(name, model, feature))
  {
  }

  SFCTask::SFCTask(const std::string& name, const Model& model, const Feature& feature)
    : Task(name, model, feature)
    , pimpl(new Pimpl(name, model, feature))
  {
  }

  void SFCTask::setSolver(QuadraticSolver& solver, SumOfLinearFunctions& seConstraint)
  {
    pimpl->solver = &solver;
    pimpl->seConstraint = &seConstraint;
  }

  void SFCTask::update()
  {
    if(isBodyContactConstraint())
      return;

    const MatrixXd Jt = getJacobian().transpose();
    pimpl->taskSEConstraintFunction->changeA( Jt.topRows(pimpl->taskSEConstraintFunction->getDimension()) );

    const MatrixXd& Kp = getStiffness();
    const MatrixXd& Kd = getDamping();
    const VectorXd& e = getFeatureDes() ? getFeature().computeError(*getFeatureDes()) : getFeature().computeError();
    const VectorXd& edot = getFeatureDes() ? getFeature().computeErrorDot(*getFeatureDes()) : getFeature().computeErrorDot();
    const VectorXd& fd = getFeatureDes() ? getFeature().computeEffort(*getFeatureDes()) : getFeature().computeEffort();
    const VectorXd f = -Kp * e - Kd * edot - fd;

    pimpl->taskAsObjective.getFunction().changeReference(f);
    pimpl->taskAsConstraint.getFunction().changeb(-f);

		if(isPointContactTask())
		{
			const MatrixXd& A = pimpl->frictionFunction->getA();
			const VectorXd b = pimpl->frictionFunction->getb() + A * getFrictionConstraintOffset();
			pimpl->frictionConstraint.getFunction().changeA(A);
			pimpl->frictionConstraint.getFunction().changeb(b);
		}
  }

  const Variable& SFCTask::getVariable() const
  {
    return pimpl->var;
  }

  void SFCTask::doActivateAsObjective()
  {
    CHECK_SOLVER_SET(SFCTask_doActivateAsObjective);
    if(isBodyContactConstraint()) return;
    pimpl->solver->addObjective(pimpl->taskAsObjective.getObjective());
    pimpl->seConstraint->addFunction(*pimpl->taskSEConstraintFunction);
  }

  void SFCTask::doActivateAsConstraint()
  {
    CHECK_SOLVER_SET(SFCTask_doActivateAsConstraint);
    if(isBodyContactConstraint()) return;
    pimpl->solver->addObjective(pimpl->minVarObjective.getObjective());
    pimpl->solver->addConstraint(pimpl->taskAsConstraint.getConstraint());
    pimpl->seConstraint->addFunction(*pimpl->taskSEConstraintFunction);
  }

  void SFCTask::doActivateContactMode()
  {
    CHECK_SOLVER_SET(SFCTask_doActivateContactMode);
    if(isBodyContactConstraint()) return;
    pimpl->solver->addConstraint(pimpl->frictionConstraint.getConstraint());
  }

  void SFCTask::doDeactivateAsObjective()
  {
    CHECK_SOLVER_SET(SFCTask_doDeactivateAsObjective);
    if(isBodyContactConstraint()) return;
    pimpl->solver->removeObjective(pimpl->taskAsObjective.getObjective());
    pimpl->seConstraint->removeFunction(*pimpl->taskSEConstraintFunction);
  }

  void SFCTask::doDeactivateAsConstraint()
  {
    CHECK_SOLVER_SET(SFCTask_doDeactivateAsConstraint);
    if(isBodyContactConstraint()) return;
    pimpl->solver->removeObjective(pimpl->minVarObjective.getObjective());
    pimpl->solver->removeConstraint(pimpl->taskAsConstraint.getConstraint());
    pimpl->seConstraint->removeFunction(*pimpl->taskSEConstraintFunction);
  }

  void SFCTask::doDeactivateContactMode()
  {
    CHECK_SOLVER_SET(SFCTask_doDeactivateContactMode);
    if(isBodyContactConstraint()) return;
    pimpl->solver->removeConstraint(pimpl->frictionConstraint.getConstraint());
  }

  void SFCTask::doSetFrictionCoeff()
  {
    pimpl->frictionFunction->setFrictionCoeff(getFrictionCoeff());
  }

  void SFCTask::doSetMargin()
  {
    pimpl->frictionFunction->setMargin(getMargin());
  }

  void SFCTask::doSetWeight()
  {
    pimpl->taskAsObjective.getFunction().changeWeight(getWeight());
  }

  void SFCTask::doGetOutput(VectorXd& output) const
  {
    output = pimpl->var.getValue();
  }
}

// cmake:sourcegroup=Controllers

#include "FFIDTask.h"

#include "ocra/control/Model.h"
#include "ocra/control/Feature.h"

#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/SquaredLinearFunction.h"
#include "ocra/optim/LinearizedCoulombFunction.h"
#include "ocra/optim/QuadraticSolver.h"
#include "ocra/optim/WeightedSquareDistanceFunction.h"
#include "ocra/optim/SumOfLinearFunctions.h"

#define CHECK_SOLVER_SET(funname) \
  if(!pimpl->ffSolver) \
  throw std::runtime_error("["#funname"] task " + getName() + " solver not set. Has the task been added to a controller?");

namespace ocra
{
  struct FFIDTask::Pimpl
  {
    const Model& model;

    BaseVariable var;

    QuadraticSolver* ffSolver;
    EqualZeroConstraintPtr<LinearFunction> ffTaskAsConstraint;
    ObjectivePtr<SquaredLinearFunction> ffTaskAsObjective;
    ObjectivePtr<WeightedSquareDistanceFunction> ffMinVarObjective;
    EqualZeroConstraintPtr<LinearFunction> ffContactConstraint;
    LessThanZeroConstraintPtr<LinearizedCoulombFunction> ffFrictionConstraint;

    QuadraticSolver* compSolver;
    SumOfLinearFunctions* compEqFunction;
    ObjectivePtr<WeightedSquareDistanceFunction> compTaskAsObjective;
    ObjectivePtr<WeightedSquareDistanceFunction> compMinVarObjective;
    EqualZeroConstraintPtr<LinearFunction> compTaskAsConstraint;
    boost::shared_ptr<LinearFunction> compEqSubFunction;
    LessThanZeroConstraintPtr<LinearFunction> compFrictionConstraint;

    Pimpl(const std::string& name, const Model& m, const Feature& s)
      : model(m)
      , var(name+".var", s.getDimension())
      , ffSolver(0x0)
      , ffTaskAsConstraint( new LinearFunction(m.getAccelerationVariable(), MatrixXd::Zero(s.getDimension(), model.nbDofs()), VectorXd::Zero(s.getDimension())) )
      , ffTaskAsObjective( new SquaredLinearFunction(&ffTaskAsConstraint.getFunction()) )
      , ffMinVarObjective( new WeightedSquareDistanceFunction(var, 1.e-12, VectorXd::Zero(s.getDimension())) )
      , ffContactConstraint( new LinearFunction(m.getAccelerationVariable(), MatrixXd::Zero(s.getDimension(), model.nbDofs()), VectorXd::Zero(s.getDimension())) )
      , ffFrictionConstraint()
      , compSolver(0x0)
      , compEqFunction(0x0)
      , compTaskAsObjective( new WeightedSquareDistanceFunction(var, 1., VectorXd::Zero(s.getDimension())) )
      , compMinVarObjective( new WeightedSquareDistanceFunction(var, 1., VectorXd::Zero(s.getDimension())) )
      , compTaskAsConstraint( new LinearFunction(var, MatrixXd::Identity(s.getDimension(), s.getDimension()), VectorXd::Zero(s.getDimension())) )
      , compEqSubFunction( buildSubFunction(var, m, s) )
      , compFrictionConstraint()
    {
      if(var.getSize() == 3) // candidate for contact mode
      {
        ffFrictionConstraint.set( new LinearizedCoulombFunction(var, 1., 4, 0.) );

        const int outDim = ffFrictionConstraint.getFunction().getDimension();
        const int inDim = 3;
        compFrictionConstraint.set( new LinearFunction(var, MatrixXd::Identity(outDim, inDim), VectorXd::Zero(outDim)) );
      }
    }

    static LinearFunction* buildSubFunction(Variable& var, const Model& m, const Feature& s)
    {
      const int nfree = m.nbDofs() - m.nbInternalDofs();
      const int dim = var.getSize();
      return new LinearFunction( var, MatrixXd::Zero(nfree, dim), VectorXd::Zero(nfree) );
    }
  };

  FFIDTask::FFIDTask(const std::string& name, const Model& model, const Feature& feature, const Feature& featureDes)
    : Task(name, model, feature, featureDes)
    , pimpl(new Pimpl(name, model, feature))
  {
  }

  FFIDTask::FFIDTask(const std::string& name, const Model& model, const Feature& feature)
    : Task(name, model, feature)
    , pimpl(new Pimpl(name, model, feature))
  {
  }

  void FFIDTask::setSolvers(QuadraticSolver& ffSolver, QuadraticSolver& compSolver, SumOfLinearFunctions& compEqFunction)
  {
    pimpl->ffSolver = &ffSolver;
    pimpl->compSolver = &compSolver;
    pimpl->compEqFunction = &compEqFunction;
  }

  void FFIDTask::updateFeedForward()
  {
    const MatrixXd& J = getJacobian();

    pimpl->ffTaskAsConstraint.getFunction().changeA(J);
    pimpl->ffContactConstraint.getFunction().changeA(J);
    pimpl->ffTaskAsConstraint.getFunction().changeb(getErrorDdot());
  }

  void FFIDTask::updateCompensation()
  {
    const MatrixXd& Kp = getStiffness();
    const MatrixXd& Kd = getDamping();
    const VectorXd f = -Kp * getError() - Kd * getErrorDot() - getEffort();
    const MatrixXd Jt_root = getJacobian().transpose().topRows(pimpl->compEqSubFunction->getDimension());

    pimpl->compTaskAsObjective.getFunction().changeReference(f);
    pimpl->compTaskAsConstraint.getFunction().changeb(-f);
    pimpl->compEqSubFunction->changeA(Jt_root);

    if(isPointContactTask())
    {
      const MatrixXd& A = pimpl->ffFrictionConstraint.getFunction().getA();
      const VectorXd b = pimpl->ffFrictionConstraint.getFunction().getb() + A * pimpl->var.getValue();
      pimpl->compFrictionConstraint.getFunction().changeA(A);
      pimpl->compFrictionConstraint.getFunction().changeb(b);
    }
  }

  void FFIDTask::addCompensation(VectorXd& tau) const
  {
    if(isBodyContactConstraint())
      return;

    tau += getJacobian().rightCols(pimpl->model.nbInternalDofs()).transpose() * pimpl->var.getValue();
  }

  void FFIDTask::doActivateAsObjective()
  {
    CHECK_SOLVER_SET(FFIDTask_doActivateAsObjective);

    if(isBodyContactConstraint())
      return;
    
    if(!isPointContactTask())
      pimpl->ffSolver->addObjective(pimpl->ffTaskAsObjective);

    pimpl->compSolver->addObjective(pimpl->compTaskAsObjective);
    pimpl->compEqFunction->addFunction(*pimpl->compEqSubFunction);
  }

  void FFIDTask::doActivateAsConstraint()
  {
    CHECK_SOLVER_SET(FFIDTask_doActivateAsConstraint);

    if(isBodyContactConstraint())
      return;
    
    if(!isPointContactTask())
      pimpl->ffSolver->addConstraint(pimpl->ffTaskAsConstraint);

    pimpl->compSolver->addObjective(pimpl->compMinVarObjective);
    pimpl->compSolver->addConstraint(pimpl->compTaskAsConstraint);
    pimpl->compEqFunction->addFunction(*pimpl->compEqSubFunction);
  }

  void FFIDTask::doActivateContactMode()
  {
    CHECK_SOLVER_SET(FFIDTask_doActivateContactMode);

    if(isBodyContactConstraint())
      pimpl->ffSolver->addConstraint(pimpl->ffContactConstraint);
    else // isPointContactTask() ensured by mother class
    {
      pimpl->ffSolver->addObjective(pimpl->ffMinVarObjective);
      pimpl->ffSolver->addConstraint(pimpl->ffFrictionConstraint);
      pimpl->compSolver->addConstraint(pimpl->compFrictionConstraint);
      pimpl->model.getModelContacts().addContactPoint(pimpl->var, getFeature());
    }
  }

  void FFIDTask::doDeactivateAsObjective()
  {
    CHECK_SOLVER_SET(FFIDTask_doDeactivateAsObjective);

    if(isBodyContactConstraint())
      return;

    if(!isPointContactTask())
      pimpl->ffSolver->removeObjective(pimpl->ffTaskAsObjective.getObjective());

    pimpl->compSolver->removeObjective(pimpl->compTaskAsObjective.getObjective());
    pimpl->compEqFunction->removeFunction(*pimpl->compEqSubFunction);
  }

  void FFIDTask::doDeactivateAsConstraint()
  {
    CHECK_SOLVER_SET(FFIDTask_doDeactivateAsConstraint);

    if(isBodyContactConstraint())
      return;

    if(!isPointContactTask())
      pimpl->ffSolver->removeConstraint(pimpl->ffTaskAsConstraint.getConstraint());

    pimpl->compSolver->removeObjective(pimpl->compMinVarObjective.getObjective());
    pimpl->compSolver->removeConstraint(pimpl->compTaskAsConstraint.getConstraint());
    pimpl->compEqFunction->removeFunction(*pimpl->compEqSubFunction);
  }

  void FFIDTask::doDeactivateContactMode()
  {
    CHECK_SOLVER_SET(FFIDTask_doDeactivateContactMode);

    if(isBodyContactConstraint())
      pimpl->ffSolver->removeConstraint(pimpl->ffContactConstraint.getConstraint());
    else // isPointContactTask() ensured by mother class
    {
      pimpl->ffSolver->removeObjective(pimpl->ffMinVarObjective.getObjective());
      pimpl->ffSolver->removeConstraint(pimpl->ffFrictionConstraint.getConstraint());
      pimpl->compSolver->removeConstraint(pimpl->compFrictionConstraint.getConstraint());
      pimpl->model.getModelContacts().removeContactPoint(pimpl->var);
    }
  }

  void FFIDTask::doSetWeight()
  {
    pimpl->ffTaskAsObjective.getFunction().changeWeight(getWeight());
    pimpl->compTaskAsObjective.getFunction().changeWeight(getWeight());
  }

  void FFIDTask::doSetFrictionCoeff()
  {
    pimpl->ffFrictionConstraint.getFunction().setFrictionCoeff(getFrictionCoeff());
  }

  void FFIDTask::doSetMargin()
  {
    pimpl->ffFrictionConstraint.getFunction().setMargin(getMargin());
  }

  void FFIDTask::doGetOutput(VectorXd& output) const
  {
    output = VectorXd::Zero(getDimension());
  }
}

// cmake:sourcegroup=Controllers

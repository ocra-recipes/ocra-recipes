#include "DynController.h"

#include "ocra/control/DynTask.h"
#include "ocra/control/Model.h"
#include "ocra/control/Feature.h"
#include "ocra/control/FullState.h"
#include "ocra/control/DynamicEquationFunction.h"
#include "ocra/control/controlUtils.h"
#include "ocra/control/ContactSet.h"

#include "ocra/optim/QLDSolver.h"
#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/WeightedSquareDistanceFunction.h"
#include "ocra/optim/SumOfLinearFunctions.h"
#include "ocra/optim/SquaredLinearFunction.h"

#include <fstream>

namespace ocra
{
  struct DynController::Pimpl
  {
    const Model& model;

    boost::shared_ptr<QuadraticSolver> solver;
    ObjectivePtr<WeightedSquareDistanceFunction> minTau;
    ObjectivePtr<WeightedSquareDistanceFunction> minTdot;
    EqualZeroConstraintPtr<DynamicEquationFunction> dynamicEquation;
    EqualZeroConstraintPtr<SumOfLinearFunctions> eqConstraint;
		boost::shared_ptr<LinearFunction> tauSubFunction;

		boost::shared_ptr<BaseVariable> gamma;
		ObjectivePtr<WeightedSquareDistanceFunction> gravityObjective;
		boost::shared_ptr<LinearFunction> gravitySubFunction;

    std::ofstream ffDebugOut;

    Pimpl(const std::string& name, const Model& m)
      : model(m)
      , solver( new QLDSolver )
      , minTau( new WeightedSquareDistanceFunction(m.getJointTorqueVariable(), 1.e-12, VectorXd::Zero(m.nbInternalDofs())) )
      , minTdot( new WeightedSquareDistanceFunction(m.getAccelerationVariable(), 1e-12, VectorXd::Zero(m.nbDofs())) )
      , dynamicEquation( new DynamicEquationFunction(m) )
      , eqConstraint( new SumOfLinearFunctions(m.nbInternalDofs()) )
      , tauSubFunction( buildSubFunction(m.getJointTorqueVariable(), m) )
			, gamma( new BaseVariable("gamma", m.nbInternalDofs()) )
			, gravityObjective( new WeightedSquareDistanceFunction(*gamma, 100000., VectorXd::Zero(m.nbInternalDofs())) )
      , gravitySubFunction( new LinearFunction(*gamma, MatrixXd::Identity(m.nbInternalDofs(), m.nbInternalDofs()), VectorXd::Zero(m.nbInternalDofs())) )
			, ffDebugOut()
    {
      solver->addObjective(minTau.getObjective());
      solver->addObjective(minTdot.getObjective());
			solver->addObjective(gravityObjective.getObjective());
      solver->addConstraint(dynamicEquation.getConstraint());
      solver->addConstraint(eqConstraint.getConstraint());

			eqConstraint.getFunction().addFunction(*tauSubFunction);
			eqConstraint.getFunction().addFunction(*gravitySubFunction);
    }

    static LinearFunction* buildSubFunction(Variable& var, const Model& m)
    {
      const int nact = m.nbInternalDofs();
      return new LinearFunction( var, -MatrixXd::Identity(nact, nact), VectorXd::Zero(nact) );
    }
  };

  DynController::DynController(const std::string& name, Model& model)
    : Controller(name, model)
    , pimpl( new Pimpl(name, model) )
  {
  }

  DynController::~DynController()
  {
  }

  void DynController::setStateDamping(double val)
  {
    pimpl->minTdot.getFunction().changeWeight(val);
  }

  void DynController::setTauDamping(double val)
  {
    pimpl->minTau.getFunction().changeWeight(val);
  }

  void DynController::doComputeOutput(Eigen::VectorXd& tau)
  {
    const std::vector<Task*>& tasks = getActiveTasks();

		pimpl->gravityObjective.getFunction().changeReference(pimpl->model.getGravityTerms().tail(pimpl->model.nbInternalDofs()));

    for(size_t i = 0; i < tasks.size(); ++i)
    {
      DynTask& currentTask = static_cast<DynTask&>(*tasks[i]); // addTask throws if this cast is not possible
      currentTask.update();
    }

    if(!pimpl->solver->solve().info)
    {
      tau = pimpl->model.getJointTorqueVariable().getValue();
    }
    else
    {
      setErrorMessage("Feed forward solver error");
      setErrorFlag(OTHER | CRITICAL_ERROR);
      pimpl->ffDebugOut.open("ffSolver.txt");
      pimpl->ffDebugOut << pimpl->solver->toString() << std::endl;
    }
  }

  void DynController::doAddTask(Task& task)
  {
    try {
      DynTask& Dyntask = dynamic_cast<DynTask&>(task);
      Dyntask.setSolver(*pimpl->solver, pimpl->eqConstraint.getFunction());
    }
    catch(...) {
      throw std::runtime_error("[DynController::doAddTask] cannot add task to controller (wrong type)");
    }
  }

  void DynController::doAddContactSet(const ContactSet& contacts)
  {
    addTasks(contacts.getTasks());
    addTask(contacts.getBodyTask());
  }

  Task* DynController::doCreateTask(const std::string& name, const Feature& feature, const Feature& featureDes) const
  {
    return new DynTask(name, pimpl->model, feature, featureDes);
  }

  Task* DynController::doCreateTask(const std::string& name, const Feature& feature) const
  {
    return new DynTask(name, pimpl->model, feature);
  }

  Task* DynController::doCreateContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const
  {
    return new DynTask(name, pimpl->model, feature);
  }
}

// cmake:sourcegroup=Controllers

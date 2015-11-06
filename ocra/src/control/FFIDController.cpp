#include "FFIDController.h"

#include "ocra/control/FFIDTask.h"
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
  struct FFIDController::Pimpl
  {
    const Model& model;

    boost::shared_ptr<QuadraticSolver> ffSolver;
    ObjectivePtr<WeightedSquareDistanceFunction> minTau;
    ObjectivePtr<WeightedSquareDistanceFunction> minTdot;
    EqualZeroConstraintPtr<DynamicEquationFunction> dynamicEquation;

    boost::shared_ptr<QuadraticSolver> compSolver;
    EqualZeroConstraintPtr<SumOfLinearFunctions> compEqConstraint;

    Twistd T_root;
    VectorXd T_art;
    Twistd actualT_root;
    VectorXd actualT_art;
    double dt;
    double xi;
		bool compensationPhase;
		bool useActualVelocities;

    std::ofstream ffDebugOut;

    Pimpl(const std::string& name, const Model& m)
      : model(m)
      , ffSolver( new QLDSolver )
      , minTau( new WeightedSquareDistanceFunction(m.getJointTorqueVariable(), 1.e-12, VectorXd::Zero(m.nbInternalDofs())) )
      , minTdot( new WeightedSquareDistanceFunction(m.getAccelerationVariable(), 1e-12, VectorXd::Zero(m.nbDofs())) )
      , dynamicEquation( new DynamicEquationFunction(m) )
      , compSolver( new QLDSolver )
      , compEqConstraint( new SumOfLinearFunctions(m.nbDofs() - m.nbInternalDofs()) )
      , T_root(Twistd::Zero())
      , T_art(VectorXd::Zero(m.nbInternalDofs()))
      , actualT_root(Twistd::Zero())
      , actualT_art(VectorXd::Zero(m.nbInternalDofs()))
      , dt(.01)
      , xi(.1)
			, compensationPhase(true)
			, useActualVelocities(true)
      , ffDebugOut()
    {
      ffSolver->addObjective(minTau.getObjective());
      ffSolver->addObjective(minTdot.getObjective());
      ffSolver->addConstraint(dynamicEquation.getConstraint());

      compSolver->addConstraint(compEqConstraint.getConstraint());
    }
  };

  FFIDController::FFIDController(const std::string& name, Model& model)
    : Controller(name, model)
    , pimpl( new Pimpl(name, model) )
  {
  }

	void FFIDController::enableCompensationPhase(bool activate)
	{
		pimpl->compensationPhase = activate;
	}

	bool FFIDController::compensationPhaseEnabled() const
	{
		return pimpl->compensationPhase;
	}

	void FFIDController::useActualVelocities(bool use)
	{
		pimpl->useActualVelocities = use;
	}

	bool FFIDController::actualVelocitiesUsed() const
	{
		return pimpl->useActualVelocities;
	}

  void FFIDController::setDt(double dt)
  {
    pimpl->dt = dt;
  }

  void FFIDController::setXi(double xi)
  {
    pimpl->xi = xi;
  }

  void FFIDController::setStateDamping(double val)
  {
    pimpl->minTdot.getFunction().changeWeight(val);
  }

  void FFIDController::setTauDamping(double val)
  {
    pimpl->minTau.getFunction().changeWeight(val);
  }

  void FFIDController::doComputeOutput(Eigen::VectorXd& tau)
  {
    const std::vector<Task*>& tasks = getActiveTasks();

    if(tasks.size()==0)
    {
      tau.resize(pimpl->model.nbInternalDofs());
      tau.setZero();
      return;
    }

    // Save velocity to restore them for compensation phase

		if(!pimpl->useActualVelocities)
		{
			Model& m = const_cast<Model&>(pimpl->model);
			pimpl->actualT_art = m.getJointVelocities();
			m.setJointVelocities(pimpl->T_art);
			if(!m.hasFixedRoot())
			{
				pimpl->actualT_root = m.getFreeFlyerVelocity();
				m.setFreeFlyerVelocity(pimpl->T_root);
			}
		}

    // Feed-forward phase

    for(size_t i = 0; i < tasks.size(); ++i)
    {
      FFIDTask& currentTask = static_cast<FFIDTask&>(*tasks[i]); // addTask throws if this cast is not possible
      currentTask.updateFeedForward();
    }

    if(!pimpl->ffSolver->solve().info)
    {
      tau = pimpl->model.getJointTorqueVariable().getValue();
    }
    else
    {
      setErrorMessage("Feed forward solver error");
      setErrorFlag(OTHER | CRITICAL_ERROR);
      pimpl->ffDebugOut.open("ffSolver.txt");
      pimpl->ffDebugOut << pimpl->ffSolver->toString() << std::endl;
    }

    // integrate acceleration result and restore velocities

		if(!pimpl->useActualVelocities)
		{
			Model& m = const_cast<Model&>(pimpl->model);
			const VectorXd& Tdot = m.getAccelerationVariable();
			pimpl->T_art = (1. - pimpl->xi) * pimpl->T_art + Tdot.head(m.nbInternalDofs()) * pimpl->dt;
			m.setJointVelocities(pimpl->actualT_art);
			if(!m.hasFixedRoot())
			{
				pimpl->T_root = (1. - pimpl->xi) * pimpl->T_root + Tdot.head(6) * pimpl->dt;
				m.setFreeFlyerVelocity(pimpl->actualT_root);
			}
		}

		if(!pimpl->compensationPhase)
			return;

    // Compensation phase

    for(size_t i = 0; i < tasks.size(); ++i)
    {
      FFIDTask& currentTask = static_cast<FFIDTask&>(*tasks[i]); // addTask throws if this cast is not possible
      currentTask.updateCompensation();
    }

    if(!pimpl->compSolver->solve().info)
    {
      for(size_t i = 0; i < tasks.size(); ++i)
      {
        FFIDTask& currentTask = static_cast<FFIDTask&>(*tasks[i]);
        currentTask.addCompensation(tau);
      }
    }
    else
    {
      setErrorMessage("Compensation solver error");
      setErrorFlag(OTHER | CRITICAL_ERROR);
    }
  }

	const VectorXd& FFIDController::getFFAcceleration() const
	{
		return pimpl->model.getAccelerationVariable().getValue();
	}

  void FFIDController::doAddTask(Task& task)
  {
    try {
      FFIDTask& ffidtask = dynamic_cast<FFIDTask&>(task);
      ffidtask.setSolvers(*pimpl->ffSolver, *pimpl->compSolver, pimpl->compEqConstraint.getFunction());
    }
    catch(...) {
      throw std::runtime_error("[FFIDController::doAddTask] cannot add task to controller (wrong type)");
    }
  }

  void FFIDController::doAddContactSet(const ContactSet& contacts)
  {
    addTasks(contacts.getTasks());
    addTask(contacts.getBodyTask());
  }

  Task* FFIDController::doCreateTask(const std::string& name, const Feature& feature, const Feature& featureDes) const
  {
    return new FFIDTask(name, pimpl->model, feature, featureDes);
  }

  Task* FFIDController::doCreateTask(const std::string& name, const Feature& feature) const
  {
    return new FFIDTask(name, pimpl->model, feature);
  }

  Task* FFIDController::doCreateContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const
  {
    return new FFIDTask(name, pimpl->model, feature);
  }
}

// cmake:sourcegroup=Controllers

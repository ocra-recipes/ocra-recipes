#include "ImpedanceController.h"

#include "ocra/control/ImpedanceTask.h"
#include "ocra/control/Model.h"
#include "ocra/control/Feature.h"
#include "ocra/control/DynamicEquationFunction.h"
#include "ocra/control/controlUtils.h"
#include "ocra/control/ContactSet.h"

#include "ocra/optim/QLDSolver.h"
#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/WeightedSquareDistanceFunction.h"
#include "ocra/optim/SumOfLinearFunctions.h"
#include "ocra/optim/SquaredLinearFunction.h"

namespace ocra
{
  struct ImpedanceController::Pimpl
  {
    Model& model;
    boost::shared_ptr<QuadraticSolver> solver;
    ObjectivePtr<WeightedSquareDistanceFunction> minTau;
    ObjectivePtr<WeightedSquareDistanceFunction> minTdot;
    EqualZeroConstraintPtr<DynamicEquationFunction> dynamicEquation;

		Displacementd meas_H;
		VectorXd meas_q;
		VectorXd meas_T;
		double kp_art;
		double kp_root_ang;
		double kp_root_lin;
		double kd_art;
		double kd_root_ang;
		double kd_root_lin;
		double dt;
		bool enableIntegration;

    Pimpl(const std::string& name, Model& m)
      : model(m)
      , solver( new QLDSolver )
      , minTau( new WeightedSquareDistanceFunction(m.getJointTorqueVariable(), 1.e-12, VectorXd::Zero(m.nbInternalDofs())) )
      , minTdot( new WeightedSquareDistanceFunction(m.getAccelerationVariable(), 1.e-12, VectorXd::Zero(m.nbDofs())) )
      , dynamicEquation( new DynamicEquationFunction(m) )

			, meas_H(Displacementd::Identity())
			, meas_q(VectorXd::Zero(m.nbInternalDofs()))
			, meas_T(VectorXd::Zero(m.nbDofs()))
			, kp_art(40)
			, kp_root_ang(40)
			, kp_root_lin(40)
			, kd_art(13)
			, kd_root_ang(13)
			, kd_root_lin(13)
			, dt(.01)
			, enableIntegration(false)
    {
      solver->addObjective(minTau.getObjective());
      solver->addObjective(minTdot.getObjective());
      solver->addConstraint(dynamicEquation.getConstraint());
    }
  };

  ImpedanceController::ImpedanceController(const std::string& name, Model& model)
    : Controller(name, model)
    , pimpl( new Pimpl(name, model) )
  {
  }

  void ImpedanceController::setStateDamping(double val)
  {
    pimpl->minTdot.getFunction().changeWeight(val);
  }

  void ImpedanceController::setTauDamping(double val)
  {
    pimpl->minTau.getFunction().changeWeight(val);
	}

	void ImpedanceController::setTimeStep(double dt)
	{
		pimpl->dt = dt;
	}

	void ImpedanceController::setArticularMeasGains(double kp, double kd)
	{
		pimpl->kp_art = kp;
		pimpl->kd_art = kd;
	}

	void ImpedanceController::setRootAngularMeasGains(double kp, double kd)
	{
		pimpl->kp_root_ang = kp;
		pimpl->kd_root_ang = kd;
	}

	void ImpedanceController::setRootLinearMeasGains(double kp, double kd)
	{
		pimpl->kp_root_lin = kp;
		pimpl->kd_root_lin = kd;
	}

	void ImpedanceController::setMeasuredRootPosition(const Eigen::Displacementd& H)
	{
		pimpl->meas_H = H;
	}

	void ImpedanceController::setMeasuredJointPositions(const Eigen::VectorXd& q)
	{
		pimpl->meas_q = q;
	}

	void ImpedanceController::setMeasuredVelocity(const Eigen::VectorXd& T)
	{
		pimpl->meas_T = T;
	}

	void ImpedanceController::enableIntegration(bool enable)
	{
		pimpl->enableIntegration = enable;
	}

  void ImpedanceController::doComputeOutput(Eigen::VectorXd& tau)
  {
    const std::vector<Task*>& tasks = getActiveTasks();

    if(tasks.size()==0)
    {
      tau.resize(pimpl->model.nbInternalDofs());
      tau.setZero();
      return;
    }

    for(size_t i = 0; i < tasks.size(); ++i)
    {
      ImpedanceTask& currentTask = static_cast<ImpedanceTask&>(*tasks[i]); // addTask throws if this cast is not possible
      currentTask.update();
    }

    if(!pimpl->solver->solve().info)
    {
      tau = pimpl->model.getJointTorqueVariable().getValue();
    }
    else
    {
      setErrorMessage("Solver error");
      setErrorFlag(OTHER | CRITICAL_ERROR);
      tau.setZero();
			return;
    }

		if(pimpl->enableIntegration)
		{
			Model& model = pimpl->model;
			const int n_art = model.nbInternalDofs();
			double dt = pimpl->dt;
			const VectorXd& q = model.getJointPositions();
			const VectorXd& qdot = model.getJointVelocities();
			const VectorXd& qddot = model.getInternalAccelerationVariable().getValue();
			const VectorXd aug_qddot = qddot + pimpl->kp_art * (pimpl->meas_q - q) + pimpl->kd_art * (pimpl->meas_T.tail(n_art) - qdot);
			model.setJointVelocities(qdot + dt * aug_qddot);
			model.setJointPositions(q + dt * model.getJointVelocities());

			if(!model.hasFixedRoot())
			{
				const int n_free = 6;
				const Displacementd& Hroot = model.getFreeFlyerPosition();
				const Twistd& Troot = model.getFreeFlyerVelocity();
				const VectorXd& Aroot = model.getRootAccelerationVariable().getValue();
				const Displacementd Herror = Hroot.inverse() * pimpl->meas_H;
				VectorXd aug_Aroot_vec = VectorXd::Zero(6);
				aug_Aroot_vec.head(3) = Aroot.head(3) + pimpl->kp_root_ang * Herror.getRotation().log() + pimpl->kd_root_ang * (pimpl->meas_T.head(3) - Troot.getAngularVelocity());
				aug_Aroot_vec.tail(3) = Aroot.tail(3) + pimpl->kp_root_lin * Herror.getTranslation() + pimpl->kd_root_lin * (pimpl->meas_T.block(0,0,3,1) - Troot.getLinearVelocity());
				const Twistd aug_Aroot = aug_Aroot_vec;
				model.setFreeFlyerVelocity(Troot + dt * aug_Aroot);
				model.setFreeFlyerPosition(Hroot * Twistd(model.getFreeFlyerVelocity() * dt).exp());
			}
		}
  }

  void ImpedanceController::doAddTask(Task& task)
  {
    try {
      ImpedanceTask& imptask = dynamic_cast<ImpedanceTask&>(task);
      imptask.setSolver(*pimpl->solver);
    }
    catch(...) {
      throw std::runtime_error("[ImpedanceController::doAddTask] cannot add task to controller (wrong type)");
    }
  }

  void ImpedanceController::doAddContactSet(const ContactSet& contacts)
  {
    addTasks(contacts.getTasks());
    addTask(contacts.getBodyTask());
  }

  Task* ImpedanceController::doCreateTask(const std::string& name, const Feature& feature, const Feature& featureDes) const
  {
    return new ImpedanceTask(name, pimpl->model, feature, featureDes);
  }

  Task* ImpedanceController::doCreateTask(const std::string& name, const Feature& feature) const
  {
    return new ImpedanceTask(name, pimpl->model, feature);
  }

  Task* ImpedanceController::doCreateContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const
  {
    return new ImpedanceTask(name, pimpl->model, feature);
  }
}

// cmake:sourcegroup=Controllers

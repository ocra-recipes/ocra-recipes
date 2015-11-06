#pragma warning(disable: 4181) // XXX Eigen 3
#include "SFCController.h"

#include "ocra/control/SFCTask.h"
#include "ocra/control/FullState.h"
#include "ocra/control/Feature.h"
#include "ocra/control/Model.h"
#include "ocra/control/ContactSet.h"

#include "ocra/optim/QLDSolver.h"
#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/SumOfLinearFunctions.h"
#pragma warning(default: 4181) // XXX Eigen 3

#include <cmath>

namespace ocra
{
  struct SFCController::Pimpl
  {
    const Model& model;

    boost::shared_ptr<QuadraticSolver> solver;
    EqualZeroConstraintPtr<SumOfLinearFunctions> seConstraint;

    FullModelState rootState;
    FullTargetState rootStateDes;
    FullStateFeature sgrav;
    FullStateFeature sgravDes;
    SFCTask gravityTask;

		Eigen::Vector3d center;
		double Rmax;
		double Rmin;
		bool passiveGravityCompensation;

    Pimpl(const std::string& name, const Model& m)
      : model(m)
      , solver( new QLDSolver )
      , seConstraint( new SumOfLinearFunctions(m.nbDofs() - m.nbInternalDofs()) )
      , rootState(name+".root.state", m, FullState::FREE_FLYER)
      , rootStateDes(name+".root.state_des", m, FullState::FREE_FLYER)
      , sgrav(name+".s_grav", rootState)
      , sgravDes(name+".s_grav_des", rootStateDes)
      , gravityTask(name+".gravity.task", m, sgrav, sgravDes)
			, Rmax(.04)
			, Rmin(.03)
			, passiveGravityCompensation(false)
    {
      int nfree = m.nbDofs() - m.nbInternalDofs();
      const MatrixXd nada = MatrixXd::Zero(nfree, nfree);

      gravityTask.setDesiredMass(nada);
      gravityTask.setDamping(nada);
      gravityTask.setStiffness(nada);
      gravityTask.setWeight(10000.);

      solver->addConstraint(seConstraint.getConstraint());
    }
  };

  SFCController::SFCController(const std::string& name, Model& model)
    : Controller(name, model)
    , pimpl( new Pimpl(name, model) )
  {
    doAddTask(pimpl->gravityTask);
    pimpl->gravityTask.activateAsObjective();
  }

	void SFCController::setCenter(const Eigen::Vector3d& center)
	{
		pimpl->center = center;
	}

	void SFCController::setRmax(double Rmax)
	{
		pimpl->Rmax = Rmax;
	}

	void SFCController::setRmin(double Rmin)
	{
		pimpl->Rmin = Rmin;
	}

	void SFCController::enablePassiveGravityCompensation(bool b)
	{
		pimpl->passiveGravityCompensation = b;
	}

	void SFCController::enableGravityCompensation(bool b)
	{
		if(b)
			pimpl->gravityTask.activateAsObjective();
		else
			pimpl->gravityTask.deactivate();
	}

  void SFCController::doComputeOutput(Eigen::VectorXd& tau)
  {
		typedef Eigen::Matrix<double,6,1> Vector6d;
		int nfree = pimpl->model.nbDofs() - pimpl->model.nbInternalDofs();
		Eigen::VectorXd gamma_q = VectorXd::Zero(pimpl->model.nbInternalDofs());

		if(pimpl->passiveGravityCompensation)
		{
			double Rmax = pimpl->Rmax;
			double Rmin = pimpl->Rmin;
			const Eigen::Vector3d& X = pimpl->model.getCoMPosition();
			double r = (X - pimpl->center).head(2).norm();
			if(r <= Rmin)
			{
				pimpl->rootStateDes.set_tau(pimpl->model.getGravityTerms().head(nfree));
				gamma_q = pimpl->model.getGravityTerms().tail(pimpl->model.nbInternalDofs());
			}
			else if(r >= Rmax)
			{
				pimpl->rootStateDes.set_tau(Vector6d::Zero());
			}
			else
			{
				double harg = (Rmin - Rmax) * (1./(r-Rmax) + 1./(r-Rmin));
				double h0 = std::tanh(harg);
				double h = h0 - 1.;
				double subdh0 = std::cosh(harg);
				double dh1_dr = -(Rmin - Rmax) * (1/((r-Rmax)*(r-Rmax)) + 1/((r-Rmin)*(r-Rmin)));
				double dh0_dr = dh1_dr / (subdh0 * subdh0);
				Eigen::Displacementd H_r_0 = pimpl->model.getFreeFlyerPosition();
				Eigen::Vector3d b((X[0] - pimpl->center[0])/r, (X[1] - pimpl->center[1])/r, 0.);
				Vector6d d_H_r = Vector6d::Zero();
				Eigen::VectorXd dr_dq = Eigen::VectorXd::Zero(pimpl->model.nbInternalDofs());
				Eigen::Vector3d g(0,0,9.8066499999999994);
				double P = 0.;
				for(int i = 0; i < pimpl->model.nbSegments(); ++i)
				{
					Eigen::Vector3d x_s_com = pimpl->model.getSegmentCoM(i);
					Eigen::Displacementd H_i_s(x_s_com[0], x_s_com[1], x_s_com[2]);
					Eigen::Displacementd H_i_0 = pimpl->model.getSegmentPosition(i) * H_i_s;
					Eigen::Displacementd H_r_i = H_i_0.inverse() * H_r_0;
					Vector6d W_b_i = Vector6d::Zero();
					W_b_i.tail(3) = H_i_0.getRotation().inverse() * b;
					d_H_r += H_r_i.adjoint().transpose() * W_b_i;

					const Eigen::Matrix<double, 6, Eigen::Dynamic>& Js = pimpl->model.getSegmentJacobian(i);
					Eigen::MatrixXd Ji = (H_i_s.inverse().adjoint() * Js).bottomRows(3);
					dr_dq += pimpl->model.getSegmentMass(i) * Ji.transpose() * b;

					P += pimpl->model.getSegmentMass(i) * g.transpose() * H_i_0.getTranslation();
				}

				Vector6d d_H_h = -.5 * (P * dh0_dr * d_H_r + h * pimpl->model.getGravityTerms().head(nfree));
				gamma_q = -.5 * (P * dh0_dr * dr_dq + h * pimpl->model.getGravityTerms().tail(pimpl->model.nbInternalDofs()));
				pimpl->rootStateDes.set_tau(d_H_h);
			}
			pimpl->gravityTask.update();
		}
		else
		{
			gamma_q = pimpl->model.getGravityTerms().tail(pimpl->model.nbInternalDofs());
			pimpl->rootStateDes.set_tau(pimpl->model.getGravityTerms().head(nfree));
			pimpl->gravityTask.update();
		}
    
    const std::vector<Task*>& tasks = getActiveTasks();

    if(tasks.size()==0)
    {
      tau.resize(pimpl->model.nbInternalDofs());
      tau.setZero();
      return;
    }

    for(size_t i = 0; i < tasks.size(); ++i)
    {
      SFCTask& currentTask = static_cast<SFCTask&>(*tasks[i]); // addTask throws if this cast is not possible
      currentTask.update();
    }

    if(!pimpl->solver->solve().info)
    {
			if(pimpl->passiveGravityCompensation)
				tau = gamma_q;
			else if(pimpl->gravityTask.isActiveAsObjective())
				tau = pimpl->model.getGravityTerms().tail(pimpl->model.nbInternalDofs());

      for(size_t i = 0; i < tasks.size(); ++i)
      {
        SFCTask& currentTask = static_cast<SFCTask&>(*tasks[i]);
        if(!currentTask.isBodyContactConstraint())
          tau += currentTask.getJacobian().transpose().bottomRows(pimpl->model.nbInternalDofs()) * currentTask.getVariable().getValue();
      }

      const VectorXd& tau_max = getMaxJointTorques();
      for(int i = 0; i < tau.size(); ++i)
      {
        if(tau(i) > tau_max(i)) tau(i) = tau_max(i);
        else if(tau(i) < -tau_max(i)) tau(i) = -tau_max(i);
      }
    }
    else
    {
      setErrorMessage("Solver error");
      setErrorFlag(OTHER | CRITICAL_ERROR);
    }
  }

  void SFCController::doAddTask(Task& task)
  {
    try {
      SFCTask& sfctask = dynamic_cast<SFCTask&>(task);
      sfctask.setSolver(*pimpl->solver, pimpl->seConstraint.getFunction());
    }
    catch(...) {
      throw std::runtime_error("[SFCController::doAddTask] cannot add task to controller (wrong type)");
    }
  }

  void SFCController::doAddContactSet(const ContactSet& contacts)
  {
    addTasks(contacts.getTasks());
    addTask(contacts.getBodyTask());
  }

  Task* SFCController::doCreateTask(const std::string& name, const Feature& feature, const Feature& featureDes) const
  {
    return new SFCTask(name, pimpl->model, feature, featureDes);
  }

  Task* SFCController::doCreateTask(const std::string& name, const Feature& feature) const
  {
    return new SFCTask(name, pimpl->model, feature);
  }

  Task* SFCController::doCreateContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const
  {
    return new SFCTask(name, pimpl->model, feature);
  }
}

// cmake:sourcegroup=Controllers

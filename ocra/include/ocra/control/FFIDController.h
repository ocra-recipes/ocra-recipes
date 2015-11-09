#ifndef _OCRA_FFID_CONTROLLER_H_
#define _OCRA_FFID_CONTROLLER_H_

#include "ocra/control/Controller.h"

namespace ocra
{
  class FFIDController
    : public Controller
  {
  public:
    FFIDController(const std::string& name, Model& model);

		void enableCompensationPhase(bool activate);
		bool compensationPhaseEnabled() const;
		void useActualVelocities(bool use);
		bool actualVelocitiesUsed() const;
    void setDt(double dt);
    void setXi(double xi);
    void setStateDamping(double val);
    void setTauDamping(double val);
		const Eigen::VectorXd& getFFAcceleration() const;

  protected:
    void doComputeOutput(Eigen::VectorXd& tau);
    void doAddTask(Task& task);
    void doAddContactSet(const ContactSet& contacts);

  protected:
    Task* doCreateTask(const std::string& name, const Feature& feature, const Feature& featureDes) const;
    Task* doCreateTask(const std::string& name, const Feature& feature) const;
    Task* doCreateContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };
}

#endif

// cmake:sourcegroup=Controllers

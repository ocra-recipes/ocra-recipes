#ifndef _OCRA_IMPEDANCE_CONTROLLER_H_
#define _OCRA_IMPEDANCE_CONTROLLER_H_

#include "ocra/control/Controller.h"
#include <Eigen/Lgsm>

namespace ocra
{
  class ImpedanceController
    : public Controller
  {
  public:
    ImpedanceController(const std::string& name, Model& model);

    void setStateDamping(double val);
    void setTauDamping(double val);

		void setTimeStep(double dt);
		void setArticularMeasGains(double kp, double kd);
		void setRootAngularMeasGains(double kp, double kd);
		void setRootLinearMeasGains(double kp, double kd);
		void setMeasuredRootPosition(const Eigen::Displacementd& H);
		void setMeasuredJointPositions(const Eigen::VectorXd& q);
		void setMeasuredVelocity(const Eigen::VectorXd& T);
		void enableIntegration(bool enable);

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

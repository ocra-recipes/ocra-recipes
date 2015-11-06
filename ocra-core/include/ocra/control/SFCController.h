#ifndef _OCRA_SFC_CONTROLLER_H_
#define _OCRA_SFC_CONTROLLER_H_

#include "ocra/control/Controller.h"

namespace ocra
{
  class SFCController
    : public Controller
  {
  public:
    SFCController(const std::string& name, Model& model);

		void setCenter(const Eigen::Vector3d& center);
		void setRmax(double Rmax);
		void setRmin(double Rmin);
		void enablePassiveGravityCompensation(bool b);
		void enableGravityCompensation(bool b);

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

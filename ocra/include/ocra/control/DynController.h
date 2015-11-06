#ifndef _OCRA_Dyn_CONTROLLER_H_
#define _OCRA_Dyn_CONTROLLER_H_

#include "ocra/control/Controller.h"

namespace ocra
{
  class DynController
    : public Controller
  {
  public:
    DynController(const std::string& name, Model& model);
	~DynController();

    void setStateDamping(double val);
    void setTauDamping(double val);

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

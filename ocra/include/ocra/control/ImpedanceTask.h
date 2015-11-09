#ifndef _OCRA_IMPEDANCE_TASK_H_
#define _OCRA_IMPEDANCE_TASK_H_

#include "ocra/control/Task.h"

namespace ocra
{
  class QuadraticSolver;
}

namespace ocra
{
  class ImpedanceTask
    : public Task
  {
  public:
    ImpedanceTask(const std::string& name, const Model& model, const Feature& feature, const Feature& featureDes);
    ImpedanceTask(const std::string& name, const Model& model, const Feature& feature);

    void setSolver(QuadraticSolver& solver);
    void update();

  protected:
    void doActivateAsObjective();
    void doActivateAsConstraint();
    void doActivateContactMode();
    void doDeactivateAsObjective();
    void doDeactivateAsConstraint();
    void doDeactivateContactMode();
    void doSetWeight();
    void doSetFrictionCoeff();
    void doSetMargin();
    void doGetOutput(Eigen::VectorXd& output) const;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };
}

#endif

// cmake:sourcegroup=Controllers

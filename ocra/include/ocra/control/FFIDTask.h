#ifndef _OCRA_FFID_TASK_H_
#define _OCRA_FFID_TASK_H_

#include "ocra/control/Task.h"

namespace ocra
{
  class QuadraticSolver;
  class SumOfLinearFunctions;
}

namespace ocra
{
  class FFIDTask
    : public Task
  {
  public:
    FFIDTask(const std::string& name, const Model& model, const Feature& feature, const Feature& featureDes);
    FFIDTask(const std::string& name, const Model& model, const Feature& feature);

    void setSolvers(QuadraticSolver& ffSolver, QuadraticSolver& compSolver, SumOfLinearFunctions& compEqFunction);
    void updateFeedForward();
    void updateCompensation();
    void addCompensation(Eigen::VectorXd& tau) const;

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

#ifndef _OCRA_Dyn_TASK_H_
#define _OCRA_Dyn_TASK_H_

#include "ocra/control/Task.h"

namespace ocra
{
  class QuadraticSolver;
  class SumOfLinearFunctions;
}

namespace ocra
{
  class DynTask
    : public Task
  {
  public:
    DynTask(const std::string& name, const Model& model, const Feature& feature, const Feature& featureDes);
    DynTask(const std::string& name, const Model& model, const Feature& feature);

    void setSolver(QuadraticSolver& solver, SumOfLinearFunctions& eqFunction);
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

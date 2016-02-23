#ifndef _OCRA_SFC_TASK_H_
#define _OCRA_SFC_TASK_H_

#include "ocra/control/Task.h"
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

namespace ocra
{
  class QuadraticSolver;
  class SumOfLinearFunctions;
  class Variable;
}

namespace ocra
{
  class SFCTask
    : public Task
  {
  public:
    SFCTask(const std::string& name, const Model& model, const Feature& feature, const Feature& featureDes);
    SFCTask(const std::string& name, const Model& model, const Feature& feature);

    void setSolver(QuadraticSolver& solver, SumOfLinearFunctions& seConstraint);
    void update();
    const Variable& getVariable() const;

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

#ifndef _OCRA_TASK_API_H_
#define _OCRA_TASK_API_H_

#include "ocra/optim/NamedInstance.h"
#include <Eigen/Core>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

namespace ocra
{
  class Feature;
  class Solver;
  class Model;
}

namespace ocra
{
  class Task
    : public NamedInstance
    , boost::noncopyable
  {
  protected:
    Task(const std::string& name, const Model& model, const Feature& feature, const Feature& featureDes);
    Task(const std::string& name, const Model& model, const Feature& feature);
  public:
    virtual ~Task() = 0;

    void activateAsObjective();
    void activateAsConstraint();
    void deactivate();
    bool isActiveAsObjective() const;
    bool isActiveAsConstraint() const;

    void setDesiredMassToActualOne();
    void setDesiredMass(double Md);
    void setDesiredMass(const Eigen::VectorXd& Md);
    void setDesiredMass(const Eigen::MatrixXd& Md);

    void setDamping(double B);
    void setDamping(const Eigen::VectorXd& B);
    void setDamping(const Eigen::MatrixXd& B);

    void setStiffness(double K);
    void setStiffness(const Eigen::VectorXd& K);
    void setStiffness(const Eigen::MatrixXd& K);

    void setAutoGains(double freq);
    void setAutoGains(double freq, double massSaturation);

    bool isDesiredMassTheActualOne() const;
    const Eigen::MatrixXd& getDesiredMass() const;
    const Eigen::MatrixXd& getDesiredMassInverse() const;
    const Eigen::MatrixXd& getDamping() const;
    const Eigen::MatrixXd& getStiffness() const;
    
    void activateContactMode();
    void deactivateContactMode();
    bool isContactModeActive() const;

    bool isBodyContactConstraint() const;
    bool isPointContactTask() const;

    double getFrictionCoeff() const;
    double getMargin() const;
		const Eigen::Vector3d& getFrictionConstraintOffset() const;
    void setFrictionCoeff(double coeff);
    void setMargin(double margin);
		void setFrictionConstraintOffset(const Eigen::Vector3d& offset);

    void setWeight(double weight);
    void setWeight(const Eigen::VectorXd& weight);
    const Eigen::VectorXd& getWeight() const;

    int getDimension() const;
    const Eigen::VectorXd& getOutput() const;
    const Eigen::VectorXd& getError() const;
    const Eigen::VectorXd& getErrorDot() const;
    const Eigen::VectorXd& getErrorDdot() const;
    const Eigen::VectorXd& getEffort() const;
    const Eigen::MatrixXd& getJacobian() const;

  protected:
    const Feature& getFeature() const;
    const Feature* getFeatureDes() const;

  protected:
    virtual void doActivateAsObjective() = 0;
    virtual void doActivateAsConstraint() = 0;
    virtual void doActivateContactMode() = 0;
    virtual void doDeactivateAsObjective() = 0;
    virtual void doDeactivateAsConstraint() = 0;
    virtual void doDeactivateContactMode() = 0;
    virtual void doSetFrictionCoeff() = 0;
    virtual void doSetMargin() = 0;
    virtual void doSetWeight() = 0;
    virtual void doGetOutput(Eigen::VectorXd& output) const = 0;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };
}

#endif

// cmake:sourcegroup=Api

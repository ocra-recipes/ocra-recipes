#pragma warning(disable: 4244) // XXX Eigen 3 JacobiSVD
#include "Task.h"

#include "ocra/control/Feature.h"
#include "ocra/control/Model.h"
#pragma warning(default: 4244)

using namespace Eigen;

namespace
{
  enum
  {
    TASK_DEACTIVATED,
    TASK_AS_OBJECTIVE,
    TASK_AS_CONSTRAINT
  };

  struct GainsWorkspace
  {
    JacobiSVD<MatrixXd> svd;
    VectorXd s;
    VectorXd kps;
    VectorXd kds;

    GainsWorkspace(int n) : svd(n, n), s(n), kps(n), kds(n) {}
  };
}

namespace ocra
{
  struct Task::Pimpl
  {
    const Feature& feature;
    const Feature* featureDes;
    int mode;
    GainsWorkspace gainsWorkspace;
    MatrixXd M;
    MatrixXd M_inverse;
    MatrixXd B;
    MatrixXd K;
    VectorXd weight;
    VectorXd output;
    VectorXd error;
    VectorXd errorDot;
    VectorXd errorDdot;
    VectorXd effort;
    MatrixXd jacobian;
		Vector3d frictionOffset;
    double frictionCoeff;
    double margin;
    bool useActualMass;
    bool contactActive;

    Pimpl(const Feature& s, const Feature* sdes)
      : feature(s)
      , featureDes(sdes)
      , mode(TASK_DEACTIVATED)
      , gainsWorkspace(s.getDimension()) 
      , M( MatrixXd::Identity(s.getDimension(), s.getDimension()) )
      , M_inverse( MatrixXd::Identity(s.getDimension(), s.getDimension()) )
      , B( MatrixXd::Zero(s.getDimension(), s.getDimension()) )
      , K( MatrixXd::Zero(s.getDimension(), s.getDimension()) )
      , weight( VectorXd::Ones(s.getDimension()) )
			, frictionOffset(Vector3d::Zero())
      , frictionCoeff(1.)
      , margin(0.)
      , useActualMass(true)
      , contactActive(false)
    {}
  };

  Task::Task(const std::string& name, const Model& model, const Feature& feature, const Feature& featureDes)
    : NamedInstance(name)
    , pimpl(new Pimpl(feature, &featureDes))
  {
  }

  Task::Task(const std::string& name, const Model& model, const Feature& feature)
    : NamedInstance(name)
    , pimpl(new Pimpl(feature, 0x0))
  {
  }

  Task::~Task()
  {
  }

  void Task::activateAsObjective()
  {
    if(pimpl->mode == TASK_AS_OBJECTIVE)
      return;
    else if(pimpl->mode == TASK_AS_CONSTRAINT)
      deactivate();

    doActivateAsObjective();
    if(pimpl->contactActive)
      doActivateContactMode();

    pimpl->mode = TASK_AS_OBJECTIVE;
  }

  void Task::activateAsConstraint()
  {
    if(pimpl->mode == TASK_AS_CONSTRAINT)
      return;
    else if(pimpl->mode == TASK_AS_OBJECTIVE)
      deactivate();

    doActivateAsConstraint();
    if(pimpl->contactActive)
      doActivateContactMode();

    pimpl->mode = TASK_AS_CONSTRAINT;
  }

  void Task::deactivate()
  {
    if(pimpl->mode == TASK_DEACTIVATED)
      return;
    
    if(pimpl->mode == TASK_AS_OBJECTIVE)
      doDeactivateAsObjective();
    else if(pimpl->mode == TASK_AS_CONSTRAINT)
      doDeactivateAsConstraint();
    
    if(pimpl->contactActive)
      doDeactivateContactMode();

    pimpl->mode = TASK_DEACTIVATED;
  }

  bool Task::isActiveAsObjective() const
  {
    return (pimpl->mode == TASK_AS_OBJECTIVE);
  }

  bool Task::isActiveAsConstraint() const
  {
    return (pimpl->mode == TASK_AS_CONSTRAINT);
  }

  void Task::setDesiredMassToActualOne()
  {
    pimpl->useActualMass = true;
  }

  void Task::setDesiredMass(double Md)
  {
    pimpl->useActualMass = false;
    pimpl->M = MatrixXd::Zero(pimpl->feature.getDimension(), pimpl->feature.getDimension());
    pimpl->M.diagonal().setConstant(Md);
  }

  void Task::setDesiredMass(const VectorXd& Md)
  {
    pimpl->useActualMass = false;
    pimpl->M = MatrixXd::Zero(pimpl->feature.getDimension(), pimpl->feature.getDimension());
    pimpl->M.diagonal() = Md;
  }

  void Task::setDesiredMass(const MatrixXd& Md)
  {
    pimpl->useActualMass = false;
    pimpl->M = Md;
  }

  void Task::setDamping(double B)
  {
    pimpl->B = MatrixXd::Zero(pimpl->feature.getDimension(), pimpl->feature.getDimension());
    pimpl->B.diagonal().setConstant(B);
  }

  void Task::setDamping(const VectorXd& B)
  {
    pimpl->B = MatrixXd::Zero(pimpl->feature.getDimension(), pimpl->feature.getDimension());
    pimpl->B.diagonal() = B;
  }

  void Task::setDamping(const MatrixXd& B)
  {
    pimpl->B = B;
  }

  void Task::setStiffness(double K)
  {
    pimpl->K = MatrixXd::Zero(pimpl->feature.getDimension(), pimpl->feature.getDimension());
    pimpl->K.diagonal().setConstant(K);
  }

  void Task::setStiffness(const VectorXd& K)
  {
    pimpl->K = MatrixXd::Zero(pimpl->feature.getDimension(), pimpl->feature.getDimension());
    pimpl->K.diagonal() = K;
  }

  void Task::setStiffness(const MatrixXd& K)
  {
    pimpl->K = K;
  }

  void Task::setAutoGains(double freq)
  {
    setAutoGains(freq, 100.);
  }

  void Task::setAutoGains(double freq, double massSaturation)
  {
    JacobiSVD<MatrixXd>& svd = pimpl->gainsWorkspace.svd;
    svd.compute(getDesiredMass(), ComputeFullU | ComputeFullV);

    const VectorXd& s = svd.singularValues();
    pimpl->gainsWorkspace.s = (s.array() < massSaturation).select(s, massSaturation);

    const double w = 2. * M_PI * freq;
    VectorXd& kps = pimpl->gainsWorkspace.kps;
    VectorXd& kds = pimpl->gainsWorkspace.kds;

    kps = s * w*w; // kp[i] = s[i] * w^2
    kds = (kps.array()*s.array()).sqrt() * 2.; // kd[i] = 2 * sqrt(kp[i]*s[i])

    pimpl->K = svd.matrixU() * kps.asDiagonal() * svd.matrixV().transpose();
    pimpl->B = svd.matrixU() * kds.asDiagonal() * svd.matrixV().transpose();
  }

  bool Task::isDesiredMassTheActualOne() const
  {
    return pimpl->useActualMass;
  }

  const MatrixXd& Task::getDesiredMass() const
  {
    if(pimpl->useActualMass)
    {
      if(pimpl->featureDes)
        return pimpl->feature.computeProjectedMass(*pimpl->featureDes);
      else
        return pimpl->feature.computeProjectedMass();
    }
    return pimpl->M;
  }

  const MatrixXd& Task::getDesiredMassInverse() const
  {
    if(pimpl->useActualMass)
    {
      if(pimpl->featureDes)
        return pimpl->feature.computeProjectedMassInverse(*pimpl->featureDes);
      else
        return pimpl->feature.computeProjectedMassInverse();
    }
    pimpl->M_inverse = pimpl->M.inverse();
    return pimpl->M_inverse;
  }

  const MatrixXd& Task::getDamping() const
  {
    return pimpl->B;
  }

  const MatrixXd& Task::getStiffness() const
  {
    return pimpl->K;
  }

  void Task::activateContactMode()
  {
    if(pimpl->contactActive)
      return;

    if((pimpl->feature.getDimension() != 3) && (pimpl->feature.getDimension() != 6))
      throw std::runtime_error("[Task::activateContactMode] Contact mode is available only for features with dimension 3 or 6");

    if(isActiveAsConstraint() || isActiveAsObjective())
      doActivateContactMode();

    pimpl->contactActive = true;
  }

  void Task::deactivateContactMode()
  {
    if(!pimpl->contactActive)
      return;

    if(isActiveAsConstraint() || isActiveAsObjective())
      doDeactivateContactMode();

    pimpl->contactActive = false;
  }

  void Task::setWeight(double weight)
  {
    pimpl->weight.setConstant(weight);
    doSetWeight();
  }

  bool Task::isContactModeActive() const
  {
    return pimpl->contactActive;
  }

  bool Task::isBodyContactConstraint() const
  {
    return isContactModeActive() && getDimension() != 3;
  }

  bool Task::isPointContactTask() const
  {
    return isContactModeActive() && getDimension() == 3;
  }

  double Task::getFrictionCoeff() const
  {
    return pimpl->frictionCoeff;
  }

  double Task::getMargin() const
  {
    return pimpl->margin;
  }

	const Vector3d& Task::getFrictionConstraintOffset() const
	{
		return pimpl->frictionOffset;
	}

  void Task::setFrictionCoeff(double coeff)
  {
    pimpl->frictionCoeff = coeff;
    doSetFrictionCoeff();
  }

  void Task::setMargin(double margin)
  {
    pimpl->margin = margin;
    doSetMargin();
  }

  void Task::setWeight(const VectorXd& weight)
  {
    pimpl->weight = weight;
    doSetWeight();
  }

	void Task::setFrictionConstraintOffset(const Vector3d& offset)
	{
		pimpl->frictionOffset = offset;
	}

  const VectorXd& Task::getWeight() const
  {
    return pimpl->weight;
  }

  int Task::getDimension() const
  {
    return getFeature().getDimension();
  }

  const VectorXd& Task::getOutput() const
  {
    doGetOutput(pimpl->output);
    return pimpl->output;
  }

  const VectorXd& Task::getError() const
  {
    pimpl->error = pimpl->featureDes ? pimpl->feature.computeError(*pimpl->featureDes) : pimpl->feature.computeError();
    return pimpl->error;
  }

  const VectorXd& Task::getErrorDot() const
  {
    pimpl->errorDot = pimpl->featureDes ? pimpl->feature.computeErrorDot(*pimpl->featureDes) : pimpl->feature.computeErrorDot();
    return pimpl->errorDot;
  }

  const VectorXd& Task::getErrorDdot() const
  {
    pimpl->errorDdot = pimpl->featureDes ? pimpl->feature.computeAcceleration(*pimpl->featureDes) : pimpl->feature.computeAcceleration();
    return pimpl->errorDdot;
  }

  const VectorXd& Task::getEffort() const
  {
    pimpl->effort = pimpl->featureDes ? pimpl->feature.computeEffort(*pimpl->featureDes) : pimpl->feature.computeEffort();
    return pimpl->effort;
  }

  const MatrixXd& Task::getJacobian() const
  {
    pimpl->jacobian = pimpl->featureDes ? pimpl->feature.computeJacobian(*pimpl->featureDes) : pimpl->feature.computeJacobian();
    return pimpl->jacobian;
  }

  const Feature& Task::getFeature() const
  {
    return pimpl->feature;
  }

  const Feature* Task::getFeatureDes() const
  {
    return pimpl->featureDes;
  }
}

// cmake:sourcegroup=Api

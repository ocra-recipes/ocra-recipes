#include "ocra/control/Feature.h"


#include <iostream>

using namespace Eigen;

namespace ocra
{
  // --- ABSTRACT -----------------------------------------------

  Feature::Feature(const std::string& name)
    : NamedInstance(name)
  {
  }

  Feature::~Feature()
  {
  }


  // --- POSITION -----------------------------------------------

  struct PositionFeature::Pimpl
  {
    ControlFrame::Ptr controlFrame;
    ECartesianDof axes;
    MatrixXd u;
    MatrixXd spaceTransform;
    VectorXd error;
    VectorXd errorDot;
    VectorXd acceleration;
    VectorXd effort;
    MatrixXd jacobian;
    MatrixXd M;
    MatrixXd Minv;

    Pimpl(ControlFrame::Ptr cf, ECartesianDof a)
      : controlFrame(cf), axes(a)
    {
      int dim = utils::computeDimensionFor(axes, NONE);

      u.resize(3, dim);
      int k = 0;
      if(axes & X)
        u.col(k++) = Vector3d(1., 0., 0.);
      if(axes & Y)
        u.col(k++) = Vector3d(0., 1., 0.);
      if(axes & Z)
        u.col(k++) = Vector3d(0., 0., 1.);

      error = VectorXd::Zero(dim);
      errorDot = VectorXd::Zero(dim);
      effort = VectorXd::Zero(dim);
      acceleration = VectorXd::Zero(dim);
      jacobian = MatrixXd::Zero(dim, cf->getJacobian().cols());
    }
  };

  PositionFeature::PositionFeature(const std::string& name, ControlFrame::Ptr frame, ECartesianDof axes)
    : Feature(name)
    , pimpl(new Pimpl(frame, axes))
  {

  }

  const MatrixXd& PositionFeature::getSpaceTransform() const
  {
    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
    pimpl->spaceTransform = pimpl->u.transpose() * R.adjoint();
    return pimpl->spaceTransform;
  }

  int PositionFeature::getDimension() const
  {
    return utils::computeDimensionFor(pimpl->axes, NONE);
  }

  const VectorXd& PositionFeature::computeEffort(const Feature& featureDes) const
  {
    const PositionFeature& sdes = dynamic_cast<const PositionFeature&>(featureDes);

//    // first compute the linear velocity error in the mobile frame
//    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
//    const Eigen::Displacementd::Rotation3D& Rdes = sdes.pimpl->controlFrame->getPosition().getRotation();
//    const Eigen::Displacementd::Rotation3D Rdes_in_r = R.inverse() * Rdes;
//    const Vector3d eff = pimpl->controlFrame->getWrench().getForce() - Rdes_in_r.adjoint() * sdes.pimpl->controlFrame->getWrench().getForce();

//    // then project it on the controlled axes
//    pimpl->effort = getSpaceTransform() * eff;

    pimpl->effort = pimpl->u.transpose() * (pimpl->controlFrame->getWrench().getForce() - sdes.pimpl->controlFrame->getWrench().getForce());

    return pimpl->effort;
  }

  const VectorXd& PositionFeature::computeEffort() const
  {
//    // first compute the linear velocity error in the mobile frame
//    const Vector3d eff = pimpl->controlFrame->getWrench().getForce();

//    // then project it on the controlled axes
//    pimpl->effort = getSpaceTransform() * eff;

    pimpl->effort = pimpl->u.transpose() * pimpl->controlFrame->getWrench().getForce();
    return pimpl->effort;
  }

  const VectorXd& PositionFeature::computeAcceleration(const Feature& featureDes) const
  {
    const PositionFeature& sdes = dynamic_cast<const PositionFeature&>(featureDes);

//    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
//    const Eigen::Displacementd::Rotation3D& Rdes = sdes.pimpl->controlFrame->getPosition().getRotation();
//    const Eigen::Displacementd::Rotation3D Rdes_in_r = R.inverse() * Rdes;
//    const VectorXd acc = pimpl->controlFrame->getAcceleration().getLinearVelocity() - Rdes_in_r.adjoint() * sdes.pimpl->controlFrame->getAcceleration().getLinearVelocity();

//    pimpl->acceleration = getSpaceTransform() * acc;

    pimpl->acceleration = pimpl->u.transpose() * (pimpl->controlFrame->getAcceleration().getLinearVelocity() - sdes.pimpl->controlFrame->getAcceleration().getLinearVelocity());
    return pimpl->acceleration;
  }

  const VectorXd& PositionFeature::computeAcceleration() const
  {
//    const VectorXd acc = pimpl->controlFrame->getAcceleration().getLinearVelocity();
//    pimpl->acceleration = getSpaceTransform() * acc;

    pimpl->acceleration = pimpl->u.transpose() * pimpl->controlFrame->getAcceleration().getLinearVelocity();
    return pimpl->acceleration;
  }

  const VectorXd& PositionFeature::computeError(const Feature& featureDes) const
  {
    const PositionFeature& sdes = dynamic_cast<const PositionFeature&>(featureDes);

////    // first compute the position error in the mobile frame
//    const Vector3d e0 = pimpl->controlFrame->getPosition().getTranslation() - sdes.pimpl->controlFrame->getPosition().getTranslation();
//    const Vector3d e_in_r = pimpl->controlFrame->getPosition().getRotation().inverse() * e0;

//    // then project it on the controlled axes
//    pimpl->error = getSpaceTransform() * e_in_r;

    pimpl->error = pimpl->u.transpose() * (pimpl->controlFrame->getPosition().getTranslation() - sdes.pimpl->controlFrame->getPosition().getTranslation());

    return pimpl->error;
  }

  const VectorXd& PositionFeature::computeError() const
  {
//    // first compute the position error in the mobile frame
//    const Vector3d e0 = pimpl->controlFrame->getPosition().getTranslation();
//    const Vector3d e_in_r = pimpl->controlFrame->getPosition().getRotation().inverse() * e0;

//    // then project it on the controlled axes
//    pimpl->error = getSpaceTransform() * e_in_r;

    pimpl->error = pimpl->u.transpose() * pimpl->controlFrame->getPosition().getTranslation();
    return pimpl->error;
  }

  const VectorXd& PositionFeature::computeErrorDot(const Feature& featureDes) const
  {
    const PositionFeature& sdes = dynamic_cast<const PositionFeature&>(featureDes);

//    // first compute the linear velocity error in the mobile frame
//    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
//    const Eigen::Displacementd::Rotation3D& Rdes = sdes.pimpl->controlFrame->getPosition().getRotation();
//    const Eigen::Displacementd::Rotation3D Rdes_in_r = R.inverse() * Rdes;
//    const Vector3d errDot = pimpl->controlFrame->getVelocity().getLinearVelocity() - Rdes_in_r.adjoint() * sdes.pimpl->controlFrame->getVelocity().getLinearVelocity();


//    // then project it on the controlled axes
//    pimpl->errorDot = getSpaceTransform() * errDot;

    pimpl->errorDot = pimpl->u.transpose() * (pimpl->controlFrame->getVelocity().getLinearVelocity()-sdes.pimpl->controlFrame->getVelocity().getLinearVelocity());


    return pimpl->errorDot;
  }

  const VectorXd& PositionFeature::computeErrorDot() const
  {
//    // first compute the linear velocity error in the mobile frame
//    const Vector3d errDot = pimpl->controlFrame->getVelocity().getLinearVelocity();

//    // then project it on the controlled axes
//    pimpl->errorDot = getSpaceTransform() * errDot;

    pimpl->errorDot = pimpl->u.transpose() * pimpl->controlFrame->getVelocity().getLinearVelocity();

    return pimpl->errorDot;
  }

  const MatrixXd& PositionFeature::computeJacobian(const Feature& featureDes) const
  {
//    const PositionFeature& sdes = dynamic_cast<const PositionFeature&>(featureDes);

//    // first compute the jacobian of the position error in the mobile frame
//    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
//    const Eigen::Displacementd::Rotation3D& Rdes = sdes.pimpl->controlFrame->getPosition().getRotation();
//    const Eigen::Displacementd::Rotation3D Rdes_in_r = R.inverse() * Rdes;
//    const MatrixXd jacobian = pimpl->controlFrame->getJacobian().bottomRows(3) - Rdes_in_r.adjoint() * sdes.pimpl->controlFrame->getJacobian().bottomRows(3);


    // then project it on the controlled axes
//    pimpl->jacobian = getSpaceTransform() * jacobian;

    pimpl->jacobian = pimpl->u.transpose() * pimpl->controlFrame->getJacobian().bottomRows(3);

    return pimpl->jacobian;
  }

  const MatrixXd& PositionFeature::computeJacobian() const
  {
//    // first compute the jacobian of the position error in the mobile frame
//    const MatrixXd jacobian = pimpl->controlFrame->getJacobian().bottomRows(3);

//    // then project it on the controlled axes
//    pimpl->jacobian = getSpaceTransform() * jacobian;

    pimpl->jacobian = pimpl->u.transpose() * pimpl->controlFrame->getJacobian().bottomRows(3);
    return pimpl->jacobian;
  }

  const Eigen::MatrixXd& PositionFeature::computeProjectedMass(const Feature& featureDes) const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[PositionFeature::computeProjectedMass] feature does not depend on configuration");

    pimpl->M = computeProjectedMassInverse(featureDes).inverse();

    return pimpl->M;
  }

  const Eigen::MatrixXd& PositionFeature::computeProjectedMass() const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[PositionFeature::computeProjectedMass] feature does not depend on configuration");

    pimpl->M = computeProjectedMassInverse().inverse();

    return pimpl->M;
  }

  const Eigen::MatrixXd& PositionFeature::computeProjectedMassInverse(const Feature& featureDes) const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[PositionFeature::computeProjectedMassInverse] feature does not depend on configuration");

    const MatrixXd& J = computeJacobian(featureDes);
    const MatrixXd& Minv = pimpl->controlFrame->getModel().getInertiaMatrixInverse();
    pimpl->Minv = J * Minv * J.transpose();

    return pimpl->Minv;
  }

  const Eigen::MatrixXd& PositionFeature::computeProjectedMassInverse() const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[PositionFeature::computeProjectedMassInverse] feature does not depend on configuration");

    const MatrixXd& J = computeJacobian();
    const MatrixXd& Minv = pimpl->controlFrame->getModel().getInertiaMatrixInverse();
    pimpl->Minv = J * Minv * J.transpose();

    return pimpl->Minv;
  }

  TaskState PositionFeature::getState() const
  {
      TaskState state;
      state.setPosition(pimpl->controlFrame->getPosition());
      state.setVelocity(pimpl->controlFrame->getVelocity());
    //   state.setAcceleration(pimpl->controlFrame->getAcceleration());
    //   state.setWrench(pimpl->controlFrame->getWrench());

      return state;
  }

  void PositionFeature::setState(const TaskState& newState)
  {
      try {
          TargetFrame::Ptr targetFrame = std::dynamic_pointer_cast<TargetFrame>(pimpl->controlFrame);
          if(newState.hasPosition()) {
                targetFrame->setPosition(newState.getPosition());
            }
            if(newState.hasVelocity()) {
                targetFrame->setVelocity(newState.getVelocity());
            }
            if(newState.hasAcceleration()) {
                targetFrame->setAcceleration(newState.getAcceleration());
            }
            if(newState.hasWrench()) {
                targetFrame->setWrench(newState.getWrench());
            }
      } catch (int errCode) {
          std::cout << "You cannot set the state of this feature because it is not a desired feature. It must be constructed with a TargetFrame." << errCode << std::endl;
      }
  }


  // --- POINT CONTACT ------------------------------------------

  struct PointContactFeature::Pimpl
  {
    ControlFrame::Ptr controlFrame;
    MatrixXd spaceTransform;
    VectorXd error;
    VectorXd errorDot;
    VectorXd effort;
    VectorXd acceleration;
    MatrixXd jacobian;
    MatrixXd M;
    MatrixXd Minv;

    Pimpl(ControlFrame::Ptr cf)
      : controlFrame(cf)
      , spaceTransform(Matrix3d::Identity())
      , error(VectorXd::Zero(3))
      , errorDot(VectorXd::Zero(3))
      , effort(VectorXd::Zero(3))
      , acceleration(VectorXd::Zero(3))
      , jacobian(MatrixXd::Zero(3, cf->getJacobian().cols()))
    {
    }
  };

  PointContactFeature::PointContactFeature(const std::string& name, ControlFrame::Ptr frame)
    : Feature(name)
    , pimpl(new Pimpl(frame))
  {
  }

  const MatrixXd& PointContactFeature::getSpaceTransform() const
  {
    return pimpl->spaceTransform;
  }

  int PointContactFeature::getDimension() const
  {
    return 3;
  }

  const VectorXd& PointContactFeature::computeEffort(const Feature& featureDes) const
  {
    throw std::runtime_error("[PointContactFeature::computeEffort(const Feature&)] Desired feature are irrelevant in PointContactFeatures");
  }

  const VectorXd& PointContactFeature::computeEffort() const
  {
    pimpl->effort = pimpl->controlFrame->getWrench().getForce();
    return pimpl->effort;
  }

  const VectorXd& PointContactFeature::computeAcceleration(const Feature& featureDes) const
  {
    throw std::runtime_error("[PointContactFeature::computeAcceleration(const Feature&)] Desired feature are irrelevant in PointContactFeatures");
  }

  const VectorXd& PointContactFeature::computeAcceleration() const
  {
    pimpl->acceleration = pimpl->controlFrame->getAcceleration().getLinearVelocity();
    return pimpl->acceleration;
  }

  const VectorXd& PointContactFeature::computeError(const Feature& featureDes) const
  {
    throw std::runtime_error("[PointContactFeature::computeError(const Feature&)] Desired feature are irrelevant in PointContactFeatures");
  }

  const VectorXd& PointContactFeature::computeError() const
  {
//    const Vector3d& e0 = pimpl->controlFrame->getPosition().getTranslation();
//    pimpl->error = pimpl->controlFrame->getPosition().getRotation().inverse() * e0;

    pimpl->error = pimpl->controlFrame->getPosition().getTranslation();
    return pimpl->error;
  }

  const VectorXd& PointContactFeature::computeErrorDot(const Feature& featureDes) const
  {
    throw std::runtime_error("[PointContactFeature::computeErrorDot(const Feature&)] Desired feature are irrelevant in PointContactFeatures");
  }

  const VectorXd& PointContactFeature::computeErrorDot() const
  {
    pimpl->errorDot = pimpl->controlFrame->getVelocity().getLinearVelocity();
    return pimpl->errorDot;
  }

  const MatrixXd& PointContactFeature::computeJacobian(const Feature& featureDes) const
  {
    throw std::runtime_error("[PointContactFeature::computeJacobian(const Feature&)] Desired feature are irrelevant in PointContactFeatures");
  }

  const MatrixXd& PointContactFeature::computeJacobian() const
  {
    pimpl->jacobian = pimpl->controlFrame->getJacobian().bottomRows(3);
    return pimpl->jacobian;
  }

  const Eigen::MatrixXd& PointContactFeature::computeProjectedMass(const Feature& featureDes) const
  {
    throw std::runtime_error("[PointContactFeature::computeProjectedMass(const Feature&)] Desired feature are irrelevant in PointContactFeatures");
  }

  const Eigen::MatrixXd& PointContactFeature::computeProjectedMass() const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[PositionFeature::computeProjectedMass] Feature must depend on configuration!");

    pimpl->M = computeProjectedMassInverse().inverse();

    return pimpl->M;
  }

  const Eigen::MatrixXd& PointContactFeature::computeProjectedMassInverse(const Feature& featureDes) const
  {
    throw std::runtime_error("[PointContactFeature::computeProjectedMassInverse(const Feature&)] Desired feature are irrelevant in PointContactFeatures");
  }

  const Eigen::MatrixXd& PointContactFeature::computeProjectedMassInverse() const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[PositionFeature::computeProjectedMassInverse] feature does not depend on configuration");

    const MatrixXd& J = computeJacobian();
    const MatrixXd& Minv = pimpl->controlFrame->getModel().getInertiaMatrixInverse();
    pimpl->Minv = J * Minv * J.transpose();

    return pimpl->Minv;
  }
  TaskState PointContactFeature::getState() const
  {
      TaskState state;
      state.setPosition(pimpl->controlFrame->getPosition());
      state.setVelocity(pimpl->controlFrame->getVelocity());
      state.setAcceleration(pimpl->controlFrame->getAcceleration());
      state.setWrench(pimpl->controlFrame->getWrench());

      return state;
  }

  void PointContactFeature::setState(const TaskState& newState)
  {
      try {
          TargetFrame::Ptr targetFrame = std::dynamic_pointer_cast<TargetFrame>(pimpl->controlFrame);
          if(newState.hasPosition()) {
                targetFrame->setPosition(newState.getPosition());
            }
            if(newState.hasVelocity()) {
                targetFrame->setVelocity(newState.getVelocity());
            }
            if(newState.hasAcceleration()) {
                targetFrame->setAcceleration(newState.getAcceleration());
            }
            if(newState.hasWrench()) {
                targetFrame->setWrench(newState.getWrench());
            }
      } catch (int errCode) {
          std::cout << "You cannot set the state of this feature because it is not a desired feature. It must be constructed with a TargetFrame." << errCode << std::endl;
      }
  }

  // --- ORIENTATION --------------------------------------------

  struct OrientationFeature::Pimpl
  {
    ControlFrame::Ptr controlFrame;
    MatrixXd spaceTransform;
    VectorXd error;
    VectorXd errorDot;
    VectorXd effort;
    VectorXd acceleration;
    MatrixXd jacobian;
    MatrixXd M;
    MatrixXd Minv;

    Pimpl(ControlFrame::Ptr cf)
      : controlFrame(cf)
    {
      spaceTransform = MatrixXd::Identity(3, 3);
      error = VectorXd::Zero(3);
      errorDot = VectorXd::Zero(3);
      effort = VectorXd::Zero(3);
      acceleration = VectorXd::Zero(3);
      jacobian = MatrixXd::Zero(3, cf->getJacobian().cols());
    }
  };

  OrientationFeature::OrientationFeature(const std::string& name, ControlFrame::Ptr frame)
    : Feature(name)
    , pimpl(new Pimpl(frame))
  {
  }

  const MatrixXd& OrientationFeature::getSpaceTransform() const
  {
    return pimpl->spaceTransform;
  }

  int OrientationFeature::getDimension() const
  {
    return 3;
  }

  const VectorXd& OrientationFeature::computeEffort(const Feature& featureDes) const
  {
    const OrientationFeature& sdes = dynamic_cast<const OrientationFeature&>(featureDes);

    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
    const Eigen::Displacementd::Rotation3D& Rdes = sdes.pimpl->controlFrame->getPosition().getRotation();
    const Eigen::Displacementd::Rotation3D Rdes_in_r = R.inverse() * Rdes;

    pimpl->effort = pimpl->controlFrame->getWrench().getTorque() - Rdes_in_r.adjoint() * sdes.pimpl->controlFrame->getWrench().getTorque();

    return pimpl->effort;
  }

  const VectorXd& OrientationFeature::computeEffort() const
  {
    pimpl->effort = pimpl->controlFrame->getWrench().getTorque();
    return pimpl->effort;
  }

  const VectorXd& OrientationFeature::computeAcceleration(const Feature& featureDes) const
  {
    const OrientationFeature& sdes = dynamic_cast<const OrientationFeature&>(featureDes);

    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
//    const Eigen::Displacementd::Rotation3D& Rdes = sdes.pimpl->controlFrame->getPosition().getRotation();
//    const Eigen::Displacementd::Rotation3D Rdes_in_r = R.inverse() * Rdes;

//    pimpl->acceleration = pimpl->controlFrame->getAcceleration().getAngularVelocity() - Rdes_in_r.adjoint() * sdes.pimpl->controlFrame->getAcceleration().getAngularVelocity();

    pimpl->acceleration = pimpl->controlFrame->getAcceleration().getAngularVelocity() - sdes.pimpl->controlFrame->getAcceleration().getAngularVelocity();
    return pimpl->acceleration;
  }

  const VectorXd& OrientationFeature::computeAcceleration() const
  {
    pimpl->acceleration = pimpl->controlFrame->getAcceleration().getAngularVelocity();
    return pimpl->acceleration;
  }

  const VectorXd& OrientationFeature::computeError(const Feature& featureDes) const
  {
    const OrientationFeature& sdes = dynamic_cast<const OrientationFeature&>(featureDes);

    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
    const Eigen::Displacementd::Rotation3D& Rdes = sdes.pimpl->controlFrame->getPosition().getRotation();

//    pimpl->error = (Rdes.inverse() * R).log();
    pimpl->error = Rdes.adjoint()*((Rdes.inverse() * R).log());

    return pimpl->error;
  }

  const VectorXd& OrientationFeature::computeError() const
  {
    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
    pimpl->error = R.log();
    return pimpl->error;
  }

  const VectorXd& OrientationFeature::computeErrorDot(const Feature& featureDes) const
  {
    const OrientationFeature& sdes = dynamic_cast<const OrientationFeature&>(featureDes);

    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
    const Eigen::Displacementd::Rotation3D& Rdes = sdes.pimpl->controlFrame->getPosition().getRotation();
    const Eigen::Displacementd::Rotation3D Rdes_in_r = R.inverse() * Rdes;

//    pimpl->errorDot = pimpl->controlFrame->getVelocity().getAngularVelocity() - Rdes_in_r.adjoint() * sdes.pimpl->controlFrame->getVelocity().getAngularVelocity();
    pimpl->errorDot = pimpl->controlFrame->getVelocity().getAngularVelocity() - sdes.pimpl->controlFrame->getVelocity().getAngularVelocity();

    return pimpl->errorDot;
  }

  const VectorXd& OrientationFeature::computeErrorDot() const
  {
    pimpl->errorDot = pimpl->controlFrame->getVelocity().getAngularVelocity();
    return pimpl->errorDot;
  }

  const MatrixXd& OrientationFeature::computeJacobian(const Feature& featureDes) const
  {
    const OrientationFeature& sdes = dynamic_cast<const OrientationFeature&>(featureDes);

    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
    const Eigen::Displacementd::Rotation3D& Rdes = sdes.pimpl->controlFrame->getPosition().getRotation();
    const Eigen::Displacementd::Rotation3D Rdes_in_r = R.inverse() * Rdes;

//    pimpl->jacobian = pimpl->controlFrame->getJacobian().topRows(3) - Rdes_in_r.adjoint() * sdes.pimpl->controlFrame->getJacobian().topRows(3);

    pimpl->jacobian = pimpl->controlFrame->getJacobian().topRows(3) - sdes.pimpl->controlFrame->getJacobian().topRows(3);
    return pimpl->jacobian;
  }

  const MatrixXd& OrientationFeature::computeJacobian() const
  {
    pimpl->jacobian = pimpl->controlFrame->getJacobian().topRows(3);
    return pimpl->jacobian;
  }

  const MatrixXd& OrientationFeature::computeProjectedMass(const Feature& featureDes) const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[OrientationFeature::computeProjectedMass] feature does not depend on configuration");

    pimpl->M = computeProjectedMassInverse(featureDes).inverse();

    return pimpl->M;
  }

  const MatrixXd& OrientationFeature::computeProjectedMass() const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[OrientationFeature::computeProjectedMass] feature does not depend on configuration");

    pimpl->M = computeProjectedMassInverse().inverse();

    return pimpl->M;
  }

  const MatrixXd& OrientationFeature::computeProjectedMassInverse(const Feature& featureDes) const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[OrientationFeature::computeProjectedMassInverse] feature does not depend on configuration");

    const MatrixXd& J = computeJacobian(featureDes);
    const MatrixXd& Minv = pimpl->controlFrame->getModel().getInertiaMatrixInverse();
    pimpl->Minv = J * Minv * J.transpose();

    return pimpl->Minv;
  }

  const MatrixXd& OrientationFeature::computeProjectedMassInverse() const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[OrientationFeature::computeProjectedMassInverse] feature does not depend on configuration");

    const MatrixXd& J = computeJacobian();
    const MatrixXd& Minv = pimpl->controlFrame->getModel().getInertiaMatrixInverse();
    pimpl->Minv = J * Minv * J.transpose();

    return pimpl->Minv;
  }
  TaskState OrientationFeature::getState() const
  {
      TaskState state;
      state.setPosition(pimpl->controlFrame->getPosition());
      state.setVelocity(pimpl->controlFrame->getVelocity());
      state.setAcceleration(pimpl->controlFrame->getAcceleration());
      state.setWrench(pimpl->controlFrame->getWrench());

      return state;
  }

  void OrientationFeature::setState(const TaskState& newState)
  {
      try {
          TargetFrame::Ptr targetFrame = std::dynamic_pointer_cast<TargetFrame>(pimpl->controlFrame);
          if(newState.hasPosition()) {
                targetFrame->setPosition(newState.getPosition());
            }
            if(newState.hasVelocity()) {
                targetFrame->setVelocity(newState.getVelocity());
            }
            if(newState.hasAcceleration()) {
                targetFrame->setAcceleration(newState.getAcceleration());
            }
            if(newState.hasWrench()) {
                targetFrame->setWrench(newState.getWrench());
            }
      } catch (int errCode) {
          std::cout << "You cannot set the state of this feature because it is not a desired feature. It must be constructed with a TargetFrame." << errCode << std::endl;
      }
  }

  // --- DISPLACEMENT -------------------------------------------

  struct DisplacementFeature::Pimpl
  {
    ControlFrame::Ptr controlFrame;
    ECartesianDof axes;
    int dim;
    MatrixXd u;
    MatrixXd spaceTransform;
    VectorXd error;
    VectorXd errorDot;
    VectorXd effort;
    VectorXd acceleration;
    MatrixXd jacobian;
    MatrixXd M;
    MatrixXd Minv;

    Pimpl(ControlFrame::Ptr cf, ECartesianDof a)
      : controlFrame(cf)
      , dim(3 + utils::computeDimensionFor(axes, NONE))
      , axes(a)
    {
      u.resize(3, dim-3);
      int k = 0;
      if(axes & X)
        u.col(k++) = Vector3d(1., 0., 0.);
      if(axes & Y)
        u.col(k++) = Vector3d(0., 1., 0.);
      if(axes & Z)
        u.col(k++) = Vector3d(0., 0., 1.);

      error = VectorXd::Zero(dim);
      errorDot = VectorXd::Zero(dim);
      effort = VectorXd::Zero(dim);
      acceleration = VectorXd::Zero(dim);
      jacobian = MatrixXd::Zero(dim, cf->getJacobian().cols());
      spaceTransform = MatrixXd(dim, 6);
    }
  };

  DisplacementFeature::DisplacementFeature(const std::string& name, ControlFrame::Ptr frame, ECartesianDof axes)
    : Feature(name)
    , pimpl(new Pimpl(frame, axes))
  {
  }

  const MatrixXd& DisplacementFeature::getSpaceTransform() const
  {
    pimpl->spaceTransform.topRows(3).setIdentity();

    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
    pimpl->spaceTransform.bottomRows(3) = pimpl->u.transpose() * R.adjoint();

    return pimpl->spaceTransform;
  }

  int DisplacementFeature::getDimension() const
  {
    return pimpl->dim;
  }

  const VectorXd& DisplacementFeature::computeEffort(const Feature& featureDes) const
  {
    const DisplacementFeature& sdes = dynamic_cast<const DisplacementFeature&>(featureDes);

    // Twist error in the mobile frame
    const Eigen::Displacementd Herror = sdes.pimpl->controlFrame->getPosition().inverse() * pimpl->controlFrame->getPosition();
    const Eigen::Wrenchd Werror = pimpl->controlFrame->getWrench() - Herror.adjointTr(sdes.pimpl->controlFrame->getWrench());

    // project the translational part on the controlled axes
    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
    const MatrixXd u_in_mobileFrame = R.inverse().adjoint() * pimpl->u;
    pimpl->effort.tail(pimpl->dim - 3) = u_in_mobileFrame.transpose() * Werror.getForce();

    pimpl->effort.head(3) = Werror.getTorque();

    return pimpl->effort;
  }

  const VectorXd& DisplacementFeature::computeEffort() const
  {
    // Twist error in the mobile frame
    const Eigen::Wrenchd Werror = pimpl->controlFrame->getWrench();

    // project the translational part on the controlled axes
    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
    const MatrixXd u_in_mobileFrame = R.inverse().adjoint() * pimpl->u;
    pimpl->effort.tail(pimpl->dim - 3) = u_in_mobileFrame.transpose() * Werror.getForce();
    pimpl->effort.head(3) = Werror.getTorque();

    return pimpl->effort;
  }

  const VectorXd& DisplacementFeature::computeAcceleration(const Feature& featureDes) const
  {
    const DisplacementFeature& sdes = dynamic_cast<const DisplacementFeature&>(featureDes);

//    // Twist error in the mobile frame
//    const Eigen::Displacementd Herror = pimpl->controlFrame->getPosition().inverse() * sdes.pimpl->controlFrame->getPosition();
//    const Eigen::Twistd Terror = sdes.pimpl->controlFrame->getVelocity() - Herror.inverse().adjoint() * pimpl->controlFrame->getVelocity();
//    //const Eigen::Twistd gamma_error =
//    //  pimpl->controlFrame->getAcceleration() -
//    //  Herror.adjoint() * sdes.pimpl->controlFrame->getAcceleration() -
//    //  Herror.adjoint() * Terror.bracket(sdes.pimpl->controlFrame->getAcceleration());
//    const Eigen::Twistd gamma_error =
//      pimpl->controlFrame->getAcceleration() - sdes.pimpl->controlFrame->getAcceleration();

//    // project the translational part on the controlled axes
//    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
//    const MatrixXd u_in_mobileFrame = R.inverse().adjoint() * pimpl->u;
//    pimpl->acceleration.tail(pimpl->dim - 3) = u_in_mobileFrame.transpose() * gamma_error.getLinearVelocity();

//    pimpl->acceleration.head(3) = gamma_error.getAngularVelocity();

    pimpl->acceleration = pimpl->controlFrame->getAcceleration() - sdes.pimpl->controlFrame->getAcceleration();

    return pimpl->acceleration;
  }

  const VectorXd& DisplacementFeature::computeAcceleration() const
  {
//    const VectorXd acc = pimpl->controlFrame->getAcceleration();

//    // project the translational part on the controlled axes
//    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
//    const MatrixXd u_in_mobileFrame = R.inverse().adjoint() * pimpl->u;
//    pimpl->acceleration.tail(pimpl->dim - 3) = u_in_mobileFrame.transpose() * acc.tail(3);
//    pimpl->acceleration.head(3) = acc.head(3);

    pimpl->acceleration = pimpl->controlFrame->getAcceleration();

    return pimpl->acceleration;
  }

  const VectorXd& DisplacementFeature::computeError(const Feature& featureDes) const
  {
    const DisplacementFeature& sdes = dynamic_cast<const DisplacementFeature&>(featureDes);

    // Displacement error in the mobile frame
//    const Eigen::Displacementd Herror = pimpl->controlFrame->getPosition().inverse() * sdes.pimpl->controlFrame->getPosition();

    // Project the opposite translational part on the controlled axes
    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
//    const MatrixXd u_in_mobileFrame = R.inverse().adjoint() * pimpl->u;
//    pimpl->error.tail(pimpl->dim - 3) = - u_in_mobileFrame.transpose() * Herror.getTranslation();

    const Eigen::Displacementd::Rotation3D& Rdes = sdes.pimpl->controlFrame->getPosition().getRotation();
//    pimpl->error.head(3) = (Rdes.inverse() * R).log();
    pimpl->error.head(3) = Rdes.adjoint()*((Rdes.inverse() * R).log());

    pimpl->error.tail(pimpl->dim - 3) = pimpl->controlFrame->getPosition().getTranslation() - sdes.pimpl->controlFrame->getPosition().getTranslation();
    return pimpl->error;
  }

  const VectorXd& DisplacementFeature::computeError() const
  {
    // Displacement error in the mobile frame
//    const Eigen::Displacementd Herror = pimpl->controlFrame->getPosition().inverse();

    // Project the opposite translational part on the controlled axes
    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
    const MatrixXd u_in_mobileFrame = R.inverse().adjoint() * pimpl->u;
//    pimpl->error.tail(pimpl->dim - 3) = - u_in_mobileFrame.transpose() * Herror.getTranslation();
    pimpl->error.tail(pimpl->dim - 3) = pimpl->controlFrame->getPosition().getTranslation();
    pimpl->error.head(3) = R.log();

    return pimpl->error;
  }

  const VectorXd& DisplacementFeature::computeErrorDot(const Feature& featureDes) const
  {
    const DisplacementFeature& sdes = dynamic_cast<const DisplacementFeature&>(featureDes);

//    // Twist error in the mobile frame
//    const Eigen::Displacementd Herror = pimpl->controlFrame->getPosition().inverse() * sdes.pimpl->controlFrame->getPosition();
//    const Eigen::Twistd Terror = pimpl->controlFrame->getVelocity() - Herror.adjoint() * sdes.pimpl->controlFrame->getVelocity();

//    // project the translational part on the controlled axes
//    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
//    const MatrixXd u_in_mobileFrame = R.inverse().adjoint() * pimpl->u;
//    pimpl->errorDot.tail(pimpl->dim - 3) = u_in_mobileFrame.transpose() * Terror.getLinearVelocity();
    pimpl->errorDot.tail(pimpl->dim - 3) = pimpl->controlFrame->getVelocity().getLinearVelocity() - sdes.pimpl->controlFrame->getVelocity().getLinearVelocity();

//    pimpl->errorDot.head(3) = Terror.getAngularVelocity();

    pimpl->errorDot.head(3) = pimpl->controlFrame->getVelocity().getAngularVelocity() - sdes.pimpl->controlFrame->getVelocity().getAngularVelocity();


    return pimpl->errorDot;
  }

  const VectorXd& DisplacementFeature::computeErrorDot() const
  {
//    // Twist error in the mobile frame
//    const Eigen::Twistd Terror = pimpl->controlFrame->getVelocity();

//    // project the translational part on the controlled axes
//    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
//    const MatrixXd u_in_mobileFrame = R.inverse().adjoint() * pimpl->u;
//    pimpl->errorDot.tail(pimpl->dim - 3) = u_in_mobileFrame.transpose() * Terror.getLinearVelocity();
    pimpl->errorDot.tail(pimpl->dim - 3) = pimpl->controlFrame->getVelocity().getLinearVelocity();

//    pimpl->errorDot.head(3) = Terror.getAngularVelocity();
    pimpl->errorDot.head(3) = pimpl->controlFrame->getVelocity().getAngularVelocity();

    return pimpl->errorDot;
  }

  const MatrixXd& DisplacementFeature::computeJacobian(const Feature& featureDes) const
  {
    const DisplacementFeature& sdes = dynamic_cast<const DisplacementFeature&>(featureDes);

//    // Twist error in the mobile frame
//    const Eigen::Displacementd Herror = pimpl->controlFrame->getPosition().inverse() * sdes.pimpl->controlFrame->getPosition();
//    const MatrixXd J = pimpl->controlFrame->getJacobian() - Herror.adjoint() * sdes.pimpl->controlFrame->getJacobian();

//    // project the translational part on the controlled axes
//    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
//    const MatrixXd u_in_mobileFrame = R.inverse().adjoint() * pimpl->u;
//    pimpl->jacobian.bottomRows(pimpl->dim - 3) = u_in_mobileFrame.transpose() * J.bottomRows(3);

//    pimpl->jacobian.topRows(3) = J.topRows(3);

    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
    const MatrixXd J = pimpl->controlFrame->getJacobian() - sdes.pimpl->controlFrame->getJacobian();

    return pimpl->jacobian;
  }

  const MatrixXd& DisplacementFeature::computeJacobian() const
  {
    // Twist error in the mobile frame
    const Eigen::Displacementd Herror = pimpl->controlFrame->getPosition().inverse();
//    const MatrixXd J = pimpl->controlFrame->getJacobian();
    pimpl->jacobian = pimpl->controlFrame->getJacobian();

//    // project the translational part on the controlled axes
//    const Eigen::Displacementd::Rotation3D& R = pimpl->controlFrame->getPosition().getRotation();
//    const MatrixXd u_in_mobileFrame = R.inverse().adjoint() * pimpl->u;
//    pimpl->jacobian.bottomRows(pimpl->dim - 3) = u_in_mobileFrame.transpose() * J.bottomRows(3);
//    pimpl->jacobian.topRows(3) = J.topRows(3);

    return pimpl->jacobian;
  }

  const MatrixXd& DisplacementFeature::computeProjectedMass(const Feature& featureDes) const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[DisplacementFeature::computeProjectedMass] feature does not depend on configuration");

    pimpl->M = computeProjectedMassInverse(featureDes).inverse();

    return pimpl->M;
  }

  const MatrixXd& DisplacementFeature::computeProjectedMass() const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[DisplacementFeature::computeProjectedMass] feature does not depend on configuration");

    pimpl->M = computeProjectedMassInverse().inverse();

    return pimpl->M;
  }

  const MatrixXd& DisplacementFeature::computeProjectedMassInverse(const Feature& featureDes) const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[DisplacementFeature::computeProjectedMassInverse] feature does not depend on configuration");

    const MatrixXd& J = computeJacobian(featureDes);
    const MatrixXd& Minv = pimpl->controlFrame->getModel().getInertiaMatrixInverse();
    pimpl->Minv = J * Minv * J.transpose();

    return pimpl->Minv;
  }

  const MatrixXd& DisplacementFeature::computeProjectedMassInverse() const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[DisplacementFeature::computeProjectedMassInverse] feature does not depend on configuration");

    const MatrixXd& J = computeJacobian();
    const MatrixXd& Minv = pimpl->controlFrame->getModel().getInertiaMatrix().inverse();
    pimpl->Minv = J * Minv * J.transpose();

    return pimpl->Minv;
  }
  TaskState DisplacementFeature::getState() const
  {
      TaskState state;
      state.setPosition(pimpl->controlFrame->getPosition());
      state.setVelocity(pimpl->controlFrame->getVelocity());
      state.setAcceleration(pimpl->controlFrame->getAcceleration());
      state.setWrench(pimpl->controlFrame->getWrench());

      return state;
  }

  void DisplacementFeature::setState(const TaskState& newState)
  {
      try {
          TargetFrame::Ptr targetFrame = std::dynamic_pointer_cast<TargetFrame>(pimpl->controlFrame);
          if(newState.hasPosition()) {
                targetFrame->setPosition(newState.getPosition());
            }
            if(newState.hasVelocity()) {
                targetFrame->setVelocity(newState.getVelocity());
            }
            if(newState.hasAcceleration()) {
                targetFrame->setAcceleration(newState.getAcceleration());
            }
            if(newState.hasWrench()) {
                targetFrame->setWrench(newState.getWrench());
            }
      } catch (int errCode) {
          std::cout << "You cannot set the state of this feature because it is not a desired feature. It must be constructed with a TargetFrame." << errCode << std::endl;
      }
  }

  // --- CONTACT CONSTRAINT FEATURES ----------------------------

  struct ContactConstraintFeature::Pimpl
  {
    ControlFrame::Ptr controlFrame;
    MatrixXd spaceTransform;
    VectorXd error;
    VectorXd errorDot;
    VectorXd effort;
    VectorXd acceleration;
    MatrixXd jacobian;
    MatrixXd M;
    MatrixXd Minv;

    Pimpl(ControlFrame::Ptr cf)
      : controlFrame(cf)
      , spaceTransform(MatrixXd::Identity(6, 6))
      , error(VectorXd::Zero(6))
      , errorDot(VectorXd::Zero(6))
      , effort(VectorXd::Zero(6))
      , acceleration(VectorXd::Zero(6))
      , jacobian(MatrixXd::Zero(6, cf->getJacobian().cols()))
    {
    }
  };

  ContactConstraintFeature::ContactConstraintFeature(const std::string& name, ControlFrame::Ptr frame)
    : Feature(name)
    , pimpl(new Pimpl(frame))
  {
  }

  const MatrixXd& ContactConstraintFeature::getSpaceTransform() const
  {
    return pimpl->spaceTransform;
  }

  int ContactConstraintFeature::getDimension() const
  {
    return 6;
  }

  const VectorXd& ContactConstraintFeature::computeEffort(const Feature& featureDes) const
  {
    throw std::runtime_error("[ContactConstraintFeature::computeEffort(const Feature&)] Desired feature are irrelevant in ContactConstraintFeature");
  }

  const VectorXd& ContactConstraintFeature::computeEffort() const
  {
    const Eigen::Wrenchd Werror = pimpl->controlFrame->getWrench();
    pimpl->effort.tail(3) = Werror.getForce();
    pimpl->effort.head(3) = Werror.getTorque();
    return pimpl->effort;
  }

  const VectorXd& ContactConstraintFeature::computeAcceleration(const Feature& featureDes) const
  {
    throw std::runtime_error("[ContactConstraintFeature::computeAcceleration(const Feature&)] Desired feature are irrelevant in ContactConstraintFeature");
  }

  const VectorXd& ContactConstraintFeature::computeAcceleration() const
  {
    pimpl->acceleration.head(3) = pimpl->controlFrame->getAcceleration().getAngularVelocity();
    pimpl->acceleration.tail(3) = pimpl->controlFrame->getAcceleration().getLinearVelocity();
    return pimpl->acceleration;
  }

  const VectorXd& ContactConstraintFeature::computeError(const Feature& featureDes) const
  {
    throw std::runtime_error("[ContactConstraintFeature::computeError(const Feature&)] Desired feature are irrelevant in ContactConstraintFeature");
  }

  const VectorXd& ContactConstraintFeature::computeError() const
  {
    const Eigen::Displacementd H = pimpl->controlFrame->getPosition();
    const Eigen::Displacementd::Rotation3D& R = H.getRotation();
//    pimpl->error.tail(3) = R.adjoint().transpose() * H.getTranslation();
    pimpl->error.tail(3) = H.getTranslation();
    pimpl->error.head(3) = R.log();
    return pimpl->error;
  }

  const VectorXd& ContactConstraintFeature::computeErrorDot(const Feature& featureDes) const
  {
    throw std::runtime_error("[ContactConstraintFeature::computeErrorDot(const Feature&)] Desired feature are irrelevant in ContactConstraintFeature");
  }

  const VectorXd& ContactConstraintFeature::computeErrorDot() const
  {
    const Eigen::Twistd Terror = pimpl->controlFrame->getVelocity();
    pimpl->errorDot.tail(3) = Terror.getLinearVelocity();
    pimpl->errorDot.head(3) = Terror.getAngularVelocity();
    return pimpl->errorDot;
  }

  const MatrixXd& ContactConstraintFeature::computeJacobian(const Feature& featureDes) const
  {
    throw std::runtime_error("[ContactConstraintFeature::computeJacobian(const Feature&)] Desired feature are irrelevant in ContactConstraintFeature");
  }

  const MatrixXd& ContactConstraintFeature::computeJacobian() const
  {
    pimpl->jacobian = pimpl->controlFrame->getJacobian();
    return pimpl->jacobian;
  }

  const MatrixXd& ContactConstraintFeature::computeProjectedMass(const Feature& featureDes) const
  {
    throw std::runtime_error("[ContactConstraintFeature::computeProjectedMass(const Feature&)] Desired feature are irrelevant in ContactConstraintFeature");
  }

  const MatrixXd& ContactConstraintFeature::computeProjectedMass() const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[DisplacementFeature::computeProjectedMass] feature does not depend on configuration");

    pimpl->M = computeProjectedMassInverse().inverse();

    return pimpl->M;
  }

  const MatrixXd& ContactConstraintFeature::computeProjectedMassInverse(const Feature& featureDes) const
  {
    throw std::runtime_error("[ContactConstraintFeature::computeProjectedMassInverse(const Feature&)] Desired feature are irrelevant in ContactConstraintFeature");
  }

  const MatrixXd& ContactConstraintFeature::computeProjectedMassInverse() const
  {
    if(!pimpl->controlFrame->dependsOnModelConfiguration())
      throw std::runtime_error("[DisplacementFeature::computeProjectedMassInverse] feature does not depend on configuration");

    const MatrixXd& J = computeJacobian();
    const MatrixXd& Minv = pimpl->controlFrame->getModel().getInertiaMatrix().inverse();
    pimpl->Minv = J * Minv * J.transpose();

    return pimpl->Minv;
  }
  TaskState ContactConstraintFeature::getState() const
  {
      TaskState state;
      state.setPosition(pimpl->controlFrame->getPosition());
      state.setVelocity(pimpl->controlFrame->getVelocity());
      state.setAcceleration(pimpl->controlFrame->getAcceleration());
      state.setWrench(pimpl->controlFrame->getWrench());

      return state;
  }

  void ContactConstraintFeature::setState(const TaskState& newState)
  {
      try {
          TargetFrame::Ptr targetFrame = std::dynamic_pointer_cast<TargetFrame>(pimpl->controlFrame);
          if(newState.hasPosition()) {
                targetFrame->setPosition(newState.getPosition());
            }
            if(newState.hasVelocity()) {
                targetFrame->setVelocity(newState.getVelocity());
            }
            if(newState.hasAcceleration()) {
                targetFrame->setAcceleration(newState.getAcceleration());
            }
            if(newState.hasWrench()) {
                targetFrame->setWrench(newState.getWrench());
            }
      } catch (int errCode) {
          std::cout << "You cannot set the state of this feature because it is not a desired feature. It must be constructed with a TargetFrame." << errCode << std::endl;
      }
  }




  // --- ARTICULAR ----------------------------------------------

  struct FullStateFeature::Pimpl
  {
    FullState::Ptr state;
    VectorXd error;
    VectorXd errorDot;
    VectorXd effort;
    VectorXd acceleration;
    MatrixXd J;
    MatrixXd M;
    MatrixXd Minv;
    MatrixXd spaceTransform;

    Pimpl(FullState::Ptr fs)
      : state(fs)
    {
      spaceTransform = MatrixXd::Identity(state->getSize(), state->getSize());
    }
  };

  FullStateFeature::FullStateFeature(const std::string& name, FullState::Ptr state)
    : Feature(name)
    , pimpl( new Pimpl(state) )
  {
  }

  const MatrixXd& FullStateFeature::getSpaceTransform() const
  {
    return pimpl->spaceTransform;
  }

  int FullStateFeature::getDimension() const
  {
    return pimpl->state->getSize();
  }

  const VectorXd& FullStateFeature::computeEffort(const Feature& featureDes) const
  {
    const FullStateFeature& sdes = dynamic_cast<const FullStateFeature&>(featureDes);
    pimpl->effort = pimpl->state->tau() - sdes.pimpl->state->tau();
    return pimpl->effort;
  }

  const VectorXd& FullStateFeature::computeEffort() const
  {
    pimpl->effort = pimpl->state->tau();
    return pimpl->effort;
  }

  const VectorXd& FullStateFeature::computeAcceleration(const Feature& featureDes) const
  {
    const FullStateFeature& sdes = dynamic_cast<const FullStateFeature&>(featureDes);
    pimpl->acceleration = pimpl->state->qddot() - sdes.pimpl->state->qddot();
    return pimpl->acceleration;
  }

  const VectorXd& FullStateFeature::computeAcceleration() const
  {
    pimpl->acceleration = pimpl->state->qddot();
    return pimpl->acceleration;
  }

  const VectorXd& FullStateFeature::computeError(const Feature& featureDes) const
  {
    const FullStateFeature& sdes = dynamic_cast<const FullStateFeature&>(featureDes);
    pimpl->error = pimpl->state->q() - sdes.pimpl->state->q();
    return pimpl->error;
  }

  const VectorXd& FullStateFeature::computeError() const
  {
    pimpl->error = pimpl->state->q();
    return pimpl->error;
  }

  const VectorXd& FullStateFeature::computeErrorDot(const Feature& featureDes) const
  {
    const FullStateFeature& sdes = dynamic_cast<const FullStateFeature&>(featureDes);
    pimpl->errorDot = pimpl->state->qdot() - sdes.pimpl->state->qdot();
    return pimpl->errorDot;
  }

  const VectorXd& FullStateFeature::computeErrorDot() const
  {
    pimpl->errorDot = pimpl->state->qdot();
    return pimpl->errorDot;
  }

  const MatrixXd& FullStateFeature::computeJacobian(const Feature& featureDes) const
  {
    const FullStateFeature& sdes = dynamic_cast<const FullStateFeature&>(featureDes);
    pimpl->J = pimpl->state->getJacobian();
    return pimpl->J;
  }

  const MatrixXd& FullStateFeature::computeJacobian() const
  {
    pimpl->J = pimpl->state->getJacobian();
    return pimpl->J;
  }

  const MatrixXd& FullStateFeature::computeProjectedMass(const Feature& featureDes) const
  {
    const FullStateFeature& sdes = dynamic_cast<const FullStateFeature&>(featureDes);
    pimpl->M = pimpl->state->getInertiaMatrix();
    return pimpl->M;
  }

  const MatrixXd& FullStateFeature::computeProjectedMass() const
  {
    pimpl->M = pimpl->state->getInertiaMatrix();
    return pimpl->M;
  }

  const MatrixXd& FullStateFeature::computeProjectedMassInverse(const Feature& featureDes) const
  {
    const FullStateFeature& sdes = dynamic_cast<const FullStateFeature&>(featureDes);
    pimpl->M = pimpl->state->getInertiaMatrixInverse();
    return pimpl->M;
  }

  const MatrixXd& FullStateFeature::computeProjectedMassInverse() const
  {
    pimpl->M = pimpl->state->getInertiaMatrixInverse();
    return pimpl->M;
  }

  TaskState FullStateFeature::getState() const
  {
      TaskState state;
      state.setQ(pimpl->state->q());
      state.setQd(pimpl->state->qdot());
      state.setQdd(pimpl->state->qddot());

      return state;
  }

  void FullStateFeature::setState(const TaskState& newState)
  {
      try {
          FullTargetState::Ptr targetState = std::dynamic_pointer_cast<FullTargetState>(pimpl->state);
          if(newState.hasQ()) {
                targetState->set_q(newState.getQ());
            }
            if(newState.hasQd()) {
                targetState->set_qdot(newState.getQd());
            }
            if(newState.hasQdd()) {
                targetState->set_qddot(newState.getQdd());
            }
      } catch (int errCode) {
          std::cout << "You cannot set the state of this feature because it is not a desired feature. It must be constructed with a FullTargetState." << errCode << std::endl;
      }
  }

  // --- PARTIAL - ARTICULAR ------------------------------------

  struct PartialStateFeature::Pimpl
  {
      PartialState::Ptr state;
      VectorXd error;
      VectorXd errorDot;
      VectorXd effort;
      VectorXd acceleration;
      MatrixXd J;
      MatrixXd M;
      MatrixXd Minv;
      MatrixXd spaceTransform;

      Pimpl(PartialState::Ptr ps)
          : state(ps)
      {
          spaceTransform = MatrixXd::Identity(state->getSize(), state->getSize());
      }
    };

  PartialStateFeature::PartialStateFeature(const std::string& name, PartialState::Ptr state)
      : Feature(name)
      , pimpl( new Pimpl(state) )
  {
  }

  const MatrixXd& PartialStateFeature::getSpaceTransform() const
  {
      return pimpl->spaceTransform;
  }

  int PartialStateFeature::getDimension() const
  {
      return pimpl->state->getSize();
  }

  const VectorXd& PartialStateFeature::computeEffort(const Feature& featureDes) const
  {
      const PartialStateFeature& sdes = dynamic_cast<const PartialStateFeature&>(featureDes);
      pimpl->effort = pimpl->state->tau() - sdes.pimpl->state->tau();
      return pimpl->effort;
  }

  const VectorXd& PartialStateFeature::computeEffort() const
  {
      pimpl->effort = pimpl->state->tau();
      return pimpl->effort;
  }

  const VectorXd& PartialStateFeature::computeAcceleration(const Feature& featureDes) const
  {
      const PartialStateFeature& sdes = dynamic_cast<const PartialStateFeature&>(featureDes);
      pimpl->acceleration = pimpl->state->qddot() - sdes.pimpl->state->qddot();
      return pimpl->acceleration;
  }

  const VectorXd& PartialStateFeature::computeAcceleration() const
  {
      pimpl->acceleration = pimpl->state->qddot();
      return pimpl->acceleration;
  }

  const VectorXd& PartialStateFeature::computeError(const Feature& featureDes) const
  {
      const PartialStateFeature& sdes = dynamic_cast<const PartialStateFeature&>(featureDes);
      pimpl->error = pimpl->state->q() - sdes.pimpl->state->q();
      return pimpl->error;
  }

  const VectorXd& PartialStateFeature::computeError() const
  {
      pimpl->error = pimpl->state->q();
      return pimpl->error;
  }

  const VectorXd& PartialStateFeature::computeErrorDot(const Feature& featureDes) const
  {
      const PartialStateFeature& sdes = dynamic_cast<const PartialStateFeature&>(featureDes);
      pimpl->errorDot = pimpl->state->qdot() - sdes.pimpl->state->qdot();
      return pimpl->errorDot;
  }

  const VectorXd& PartialStateFeature::computeErrorDot() const
  {
      pimpl->errorDot = pimpl->state->qdot();
      return pimpl->errorDot;
  }

  const MatrixXd& PartialStateFeature::computeJacobian(const Feature& featureDes) const
  {
      const PartialStateFeature& sdes = dynamic_cast<const PartialStateFeature&>(featureDes);
      pimpl->J = pimpl->state->getJacobian();
      return pimpl->J;
  }

  const MatrixXd& PartialStateFeature::computeJacobian() const
  {
      pimpl->J = pimpl->state->getJacobian();
      return pimpl->J;
  }

  const MatrixXd& PartialStateFeature::computeProjectedMass(const Feature& featureDes) const
  {
      const PartialStateFeature& sdes = dynamic_cast<const PartialStateFeature&>(featureDes);
      pimpl->M = pimpl->state->getInertiaMatrix();
      return pimpl->M;
  }

  const MatrixXd& PartialStateFeature::computeProjectedMass() const
  {
      pimpl->M = pimpl->state->getInertiaMatrix();
      return pimpl->M;
  }

  const MatrixXd& PartialStateFeature::computeProjectedMassInverse(const Feature& featureDes) const
  {
      const PartialStateFeature& sdes = dynamic_cast<const PartialStateFeature&>(featureDes);
      pimpl->M = pimpl->state->getInertiaMatrixInverse();
      return pimpl->M;
  }

  const MatrixXd& PartialStateFeature::computeProjectedMassInverse() const
  {
      pimpl->M = pimpl->state->getInertiaMatrixInverse();
      return pimpl->M;
  }
  TaskState PartialStateFeature::getState() const
  {
      TaskState state;
      state.setQ(pimpl->state->q());
      state.setQd(pimpl->state->qdot());
      state.setQdd(pimpl->state->qddot());

      return state;
  }

  void PartialStateFeature::setState(const TaskState& newState)
  {
      try {
          PartialTargetState::Ptr targetState = std::dynamic_pointer_cast<PartialTargetState>(pimpl->state);
          if(newState.hasQ()) {
                targetState->set_q(newState.getQ());
            }
            if(newState.hasQd()) {
                targetState->set_qdot(newState.getQd());
            }
            if(newState.hasQdd()) {
                targetState->set_qddot(newState.getQdd());
            }
      } catch (int errCode) {
          std::cout << "You cannot set the state of this feature because it is not a desired feature. It must be constructed with a PartialTargetState." << errCode << std::endl;
      }
  }

}

// cmake:sourcegroup=Api

/*!
\file Feature.h
\brief A class hierarchy to compute task errors based on control frames.

Copyright (C) 2010 CEA/DRT/LIST/DTSI/SRCI

\author Escande Adrien
\author Evrard Paul
\date 2010/10/03

File history:
*/

#ifndef _OCRA_FEATURE_H_
#define _OCRA_FEATURE_H_

#include "ocra/control/ControlEnum.h"
#include "ocra/optim/NamedInstance.h"
#include <Eigen/Lgsm>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

namespace ocra
{
  class ControlFrame;
  class FullState;
}

namespace ocra
{
  // --- ABSTRACT -----------------------------------------------

  //! Feature interface, used by tasks to compute errors and jacobians.
  /*!
  A feature is associated with a control frame, and computes errors and jacobians
  related to the frame, or to the 'difference' between the frame and a frame associated
  to another feature, denoted 'desired feature'.
  An task can be defined as the error between a feature and a desired feature, and
  features are therefore key objects to build instances of the Task class.

  See the children classes to know the different tasks that can be built.

  Concurrent call of computeXXX() and computeXXX(FeatureDes) are not thread-safe
  by default (most implementations use the same cache).
  */
  class Feature
    : public NamedInstance
    , boost::noncopyable
  {
  protected:
    Feature(const std::string& name);

  public:
    virtual ~Feature() = 0;

    virtual int getDimension() const = 0;

    virtual const Eigen::MatrixXd& getSpaceTransform() const = 0;

    virtual const Eigen::VectorXd& computeEffort(const Feature& featureDes) const = 0;
    virtual const Eigen::VectorXd& computeAcceleration(const Feature& featureDes) const = 0;
    virtual const Eigen::VectorXd& computeError(const Feature& featureDes) const = 0;
    virtual const Eigen::VectorXd& computeErrorDot(const Feature& featureDes) const = 0;
    virtual const Eigen::MatrixXd& computeJacobian(const Feature& featureDes) const = 0;
    virtual const Eigen::MatrixXd& computeProjectedMass(const Feature& featureDes) const = 0;
    virtual const Eigen::MatrixXd& computeProjectedMassInverse(const Feature& featureDes) const = 0;

    virtual const Eigen::VectorXd& computeEffort() const = 0;
    virtual const Eigen::VectorXd& computeAcceleration() const = 0;
    virtual const Eigen::VectorXd& computeError() const = 0;
    virtual const Eigen::VectorXd& computeErrorDot() const = 0;
    virtual const Eigen::MatrixXd& computeJacobian() const = 0;
    virtual const Eigen::MatrixXd& computeProjectedMass() const = 0;
    virtual const Eigen::MatrixXd& computeProjectedMassInverse() const = 0;
  };


  // --- POSITION -----------------------------------------------

  //! Used to build positioning tasks
  /*!
  The error between two PositionFeature is equal to the difference between
  the origins of their associated frames (the orientation error is neglected).
  You can specify axes at construction to consider only some axes: position along
  x, y, or z; position in the xy, xz, yz planes, or in the cartesian space (x, y, and z).
  The output space of the jacobian given by this feature correspond to the world frame.
  Use getSpaceTransform() to get the transformation from the control frame space to
  the output space.
  */
  class PositionFeature
    : public Feature
  {
  public:
    PositionFeature(const std::string& name, const ControlFrame& frame, ECartesianDof axes);

    int getDimension() const;

    const Eigen::MatrixXd& getSpaceTransform() const;

    const Eigen::VectorXd& computeEffort(const Feature& featureDes) const;
    const Eigen::VectorXd& computeAcceleration(const Feature& featureDes) const;
    const Eigen::VectorXd& computeError(const Feature& featureDes) const;
    const Eigen::VectorXd& computeErrorDot(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeJacobian(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMass(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMassInverse(const Feature& featureDes) const;

    const Eigen::VectorXd& computeEffort() const;
    const Eigen::VectorXd& computeAcceleration() const;
    const Eigen::VectorXd& computeError() const;
    const Eigen::VectorXd& computeErrorDot() const;
    const Eigen::MatrixXd& computeJacobian() const;
    const Eigen::MatrixXd& computeProjectedMass() const;
    const Eigen::MatrixXd& computeProjectedMassInverse() const;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };


  // --- POINT CONTACT ------------------------------------------

  //! Used to build point contact tasks
  /*!
  Desired features have no meaning here, so the versions of the methods
  with desired features passed as arguments will throw. These features
  are used to build point contact tasks, and will typically be associated
  with friction cones. The space transform is identity: the error is
  computed in the controlled frame.
  */
  class PointContactFeature
    : public Feature
  {
  public:
    PointContactFeature(const std::string& name, const ControlFrame& frame);

    int getDimension() const;

    const Eigen::MatrixXd& getSpaceTransform() const;

    const Eigen::VectorXd& computeEffort(const Feature& featureDes) const;
    const Eigen::VectorXd& computeAcceleration(const Feature& featureDes) const;
    const Eigen::VectorXd& computeError(const Feature& featureDes) const;
    const Eigen::VectorXd& computeErrorDot(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeJacobian(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMass(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMassInverse(const Feature& featureDes) const;

    const Eigen::VectorXd& computeEffort() const;
    const Eigen::VectorXd& computeAcceleration() const;
    const Eigen::VectorXd& computeError() const;
    const Eigen::VectorXd& computeErrorDot() const;
    const Eigen::MatrixXd& computeJacobian() const;
    const Eigen::MatrixXd& computeProjectedMass() const;
    const Eigen::MatrixXd& computeProjectedMassInverse() const;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };


  // --- ORIENTATION --------------------------------------------

  //! Used to build orientation tasks.
  /*!
  OrientationFeature instances consider only the orientation of the associated control frame.
  The error between two orientation features is the log of Rdes.inverse() * R, where R is
  the orientation of the orientation frame and Rdes is the desired orientation.
  */
  class OrientationFeature
    : public Feature
  {
  public:
    OrientationFeature(const std::string& name, const ControlFrame& frame);

    int getDimension() const;

    const Eigen::MatrixXd& getSpaceTransform() const;

    const Eigen::VectorXd& computeEffort(const Feature& featureDes) const;
    const Eigen::VectorXd& computeAcceleration(const Feature& featureDes) const;
    const Eigen::VectorXd& computeError(const Feature& featureDes) const;
    const Eigen::VectorXd& computeErrorDot(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeJacobian(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMass(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMassInverse(const Feature& featureDes) const;

    const Eigen::VectorXd& computeEffort() const;
    const Eigen::VectorXd& computeAcceleration() const;
    const Eigen::VectorXd& computeError() const;
    const Eigen::VectorXd& computeErrorDot() const;
    const Eigen::MatrixXd& computeJacobian() const;
    const Eigen::MatrixXd& computeProjectedMass() const;
    const Eigen::MatrixXd& computeProjectedMassInverse() const;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };


  // --- DISPLACEMENT -------------------------------------------

  //! Used to build position/orientation tasks.
  /*!
  These features combine position and orientation features, to control both the position
  and orientation of a control frame. See the documentation of PositionFeature and
  OrientationFeature.
  */
  class DisplacementFeature
    : public Feature
  {
  public:
    DisplacementFeature(const std::string& name, const ControlFrame& frame, ECartesianDof axes);

    int getDimension() const;

    const Eigen::MatrixXd& getSpaceTransform() const;

    const Eigen::VectorXd& computeEffort(const Feature& featureDes) const;
    const Eigen::VectorXd& computeAcceleration(const Feature& featureDes) const;
    const Eigen::VectorXd& computeError(const Feature& featureDes) const;
    const Eigen::VectorXd& computeErrorDot(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeJacobian(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMass(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMassInverse(const Feature& featureDes) const;

    const Eigen::VectorXd& computeEffort() const;
    const Eigen::VectorXd& computeAcceleration() const;
    const Eigen::VectorXd& computeError() const;
    const Eigen::VectorXd& computeErrorDot() const;
    const Eigen::MatrixXd& computeJacobian() const;
    const Eigen::MatrixXd& computeProjectedMass() const;
    const Eigen::MatrixXd& computeProjectedMassInverse() const;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };


  // --- CONTACT CONSTRAINT -------------------------------------

  //! Used to build contact constraint tasks (null acceleration).
  /*!
  These features is similar to a displacement feature, except all quantities are
  expressed in the controlled frame (space transform is identity). Desired
  features are irrelevant and computeXXX(const Feature&) will throw.
  */
  class ContactConstraintFeature
    : public Feature
  {
  public:
    ContactConstraintFeature(const std::string& name, const ControlFrame& frame);

    int getDimension() const;

    const Eigen::MatrixXd& getSpaceTransform() const;

    const Eigen::VectorXd& computeEffort(const Feature& featureDes) const;
    const Eigen::VectorXd& computeAcceleration(const Feature& featureDes) const;
    const Eigen::VectorXd& computeError(const Feature& featureDes) const;
    const Eigen::VectorXd& computeErrorDot(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeJacobian(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMass(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMassInverse(const Feature& featureDes) const;

    const Eigen::VectorXd& computeEffort() const;
    const Eigen::VectorXd& computeAcceleration() const;
    const Eigen::VectorXd& computeError() const;
    const Eigen::VectorXd& computeErrorDot() const;
    const Eigen::MatrixXd& computeJacobian() const;
    const Eigen::MatrixXd& computeProjectedMass() const;
    const Eigen::MatrixXd& computeProjectedMassInverse() const;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };


  // --- LOOK AT ------------------------------------------------

  //! Not implemented yet.
  class LookAtFeature
    : public Feature
  {
  public:
    LookAtFeature(const std::string& name, const ControlFrame& frame);

    int getDimension() const;

    const Eigen::MatrixXd& getSpaceTransform() const;

    const Eigen::VectorXd& computeEffort(const Feature& featureDes) const;
    const Eigen::VectorXd& computeAcceleration(const Feature& featureDes) const;
    const Eigen::VectorXd& computeError(const Feature& featureDes) const;
    const Eigen::VectorXd& computeErrorDot(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeJacobian(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMass(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMassInverse(const Feature& featureDes) const;

    const Eigen::VectorXd& computeEffort() const;
    const Eigen::VectorXd& computeAcceleration() const;
    const Eigen::VectorXd& computeError() const;
    const Eigen::VectorXd& computeErrorDot() const;
    const Eigen::MatrixXd& computeJacobian() const;
    const Eigen::MatrixXd& computeProjectedMass() const;
    const Eigen::MatrixXd& computeProjectedMassInverse() const;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };


  // --- ARTICULAR ----------------------------------------------

  //! Used to build tasks in the manikin configuration space.
  /*!
  Can be used e.g. for Joint PD control.
  */
  class FullStateFeature
    : public Feature
  {
  public:
    FullStateFeature(const std::string& name, const FullState& state);

    int getDimension() const;

    const Eigen::MatrixXd& getSpaceTransform() const;

    const Eigen::VectorXd& computeEffort(const Feature& featureDes) const;
    const Eigen::VectorXd& computeAcceleration(const Feature& featureDes) const;
    const Eigen::VectorXd& computeError(const Feature& featureDes) const;
    const Eigen::VectorXd& computeErrorDot(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeJacobian(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMass(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMassInverse(const Feature& featureDes) const;

    const Eigen::VectorXd& computeEffort() const;
    const Eigen::VectorXd& computeAcceleration() const;
    const Eigen::VectorXd& computeError() const;
    const Eigen::VectorXd& computeErrorDot() const;
    const Eigen::MatrixXd& computeJacobian() const;
    const Eigen::MatrixXd& computeProjectedMass() const;
    const Eigen::MatrixXd& computeProjectedMassInverse() const;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };
}

#endif

// cmake:sourcegroup=Api

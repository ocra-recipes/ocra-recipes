/*!
\file ControlFrame.h
\brief Classes that represent frames.

Copyright (C) 2010 CEA/DRT/LIST/DTSI/SRCI

\author Evrard Paul
\author Escande Adrien
\date 2010/10/03

File history:
*/

#ifndef _OCRA_CONTROL_FRAME_H_
#define _OCRA_CONTROL_FRAME_H_

#include "ocra/optim/NamedInstance.h"
#include "ocra/MathTypes.h"
#include "Eigen/Lgsm"
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

namespace ocra
{
  class Model;
}

namespace ocra
{
  // --- ABSTRACT -----------------------------------------------

  //! Generic representation of a frame: gives access to its position, velocity, jacobian...
  /*!
  A frame is always associated to a manikin's model. However, it does not always depend
  on its configuration.
  If it depends on the configuration of the manikin's model, then the method
  ControlFrame::sependsOnModelConfiguration will return true.
  Otherwise, this method returns false, and ControlFrame::getJacobian
  will return a null matrix, whose size be 6 x getModel().nbDofs().
  */
  class ControlFrame
    : public NamedInstance
    , boost::noncopyable
  {
  protected:
    ControlFrame(const std::string& name);

  public:
    virtual ~ControlFrame() = 0;

    virtual Eigen::Displacementd getPosition() const = 0;
    virtual Eigen::Twistd getVelocity() const = 0;
    virtual Eigen::Twistd getAcceleration() const = 0;
    virtual Eigen::Wrenchd getWrench() const = 0;
    virtual Eigen::Matrix<double,6,Eigen::Dynamic> getJacobian() const = 0;
    virtual bool dependsOnModelConfiguration() const = 0;
    virtual const Model& getModel() const = 0;
  };


  // --- EXTERNAL INPUT -----------------------------------------

  //! Represents a 'target', i.e. something that does not depend on a model but must match with another frame.
  /*!
  A target frame has its position and velocity manually defined via TargetFrame::setPosition and TargetFrame::setVelocity.
  It can be used to specify control objectives. Note that you always have to specify a displacement and a twist, whatever
  the control objective. If you are only interested in specifying a position, you can just fill the rotational part with
  the identity quaternion. Likewise, if only the rotational part is of interest, you can specify any position, since it will
  be ignored by the controller. The same goes for the twist.
  */
  class TargetFrame
    : public ControlFrame
  {
  public:
    TargetFrame(const std::string& name, const Model& model);

    Eigen::Displacementd getPosition() const;
    Eigen::Twistd getVelocity() const;
    Eigen::Twistd getAcceleration() const;
    Eigen::Wrenchd getWrench() const;
    Eigen::Matrix<double,6,Eigen::Dynamic> getJacobian() const;
    bool dependsOnModelConfiguration() const;
    const Model& getModel() const;

    void setPosition(const Eigen::Displacementd& H);
    void setVelocity(const Eigen::Twistd& T);
    void setAcceleration(const Eigen::Twistd& gamma);
    void setWrench(const Eigen::Wrenchd& W);

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };


  // --- ATTACHED TO A SEGMENT ----------------------------------

  //! A frame attached to a segment of a model.
  class SegmentFrame
    : public ControlFrame
  {
  public:
    SegmentFrame(const std::string& name, const Model& model, const std::string& segname);
    SegmentFrame(const std::string& name, const Model& model, const std::string& segname, const Eigen::Displacementd& H_local);
    SegmentFrame(const std::string& name, const Model& model, int segmentId);
    SegmentFrame(const std::string& name, const Model& model, int segmentId, const Eigen::Displacementd& H_local);

    Eigen::Displacementd getPosition() const;
    Eigen::Twistd getVelocity() const;
    Eigen::Twistd getAcceleration() const;
    Eigen::Wrenchd getWrench() const;
    Eigen::Matrix<double,6,Eigen::Dynamic> getJacobian() const;
    bool dependsOnModelConfiguration() const;
    const Model& getModel() const;
    int getSegmentIndex() const;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };


  // --- ATTACHED TO THE COM ------------------------------------

  //! Creates a frame whose center is at the CoM of the model and whose axes are parallel to the axes of the world frame.
  class CoMFrame
    : public ControlFrame
  {
  public:
    CoMFrame(const std::string& name, const Model& model);

    Eigen::Displacementd getPosition() const;
    Eigen::Twistd getVelocity() const;
    Eigen::Twistd getAcceleration() const;
    Eigen::Wrenchd getWrench() const;
    Eigen::Matrix<double,6,Eigen::Dynamic> getJacobian() const;
    bool dependsOnModelConfiguration() const;
    const Model& getModel() const;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };
}

#endif

// cmake:sourcegroup=Api

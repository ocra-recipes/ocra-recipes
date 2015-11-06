#ifndef _OCRA_FULLSTATE_H_
#define _OCRA_FULLSTATE_H_

#include "ocra/optim/NamedInstance.h"
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

namespace ocra
{
  class Model;
}

namespace ocra
{
  class FullState
    : public NamedInstance
  {
  public:
    enum
    {
      FULL_STATE,
      FREE_FLYER,
      INTERNAL
    };

  public:
    FullState(const std::string& name, const Model& model, int whichPart);
    virtual ~FullState() = 0;

    const Model& getModel() const;
    int getSize() const;
    const Eigen::MatrixXd& getJacobian() const;

    virtual const Eigen::MatrixXd& getInertiaMatrix() const = 0;
    virtual const Eigen::MatrixXd& getInertiaMatrixInverse() const = 0;
    virtual const Eigen::VectorXd& q() const = 0;
    virtual const Eigen::VectorXd& qdot() const = 0;
    virtual const Eigen::VectorXd& qddot() const = 0;
    virtual const Eigen::VectorXd& tau() const = 0;

  protected:
    int whichPart() const;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };


  class FullModelState
    : public FullState
  {
  public:
    FullModelState(const std::string& name, const Model& model, int whichPart);

    const Eigen::MatrixXd& getInertiaMatrix() const;
    const Eigen::MatrixXd& getInertiaMatrixInverse() const;
    const Eigen::VectorXd& q() const;
    const Eigen::VectorXd& qdot() const;
    const Eigen::VectorXd& qddot() const;
    const Eigen::VectorXd& tau() const;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };


  class FullTargetState
    : public FullState
  {
  public:
    FullTargetState(const std::string& name, const Model& model, int whichPart);

    const Eigen::MatrixXd& getInertiaMatrix() const;
    const Eigen::MatrixXd& getInertiaMatrixInverse() const;
    const Eigen::VectorXd& q() const;
    const Eigen::VectorXd& qdot() const;
    const Eigen::VectorXd& qddot() const;
    const Eigen::VectorXd& tau() const;

    void set_q(const Eigen::VectorXd& q);
    void set_qdot(const Eigen::VectorXd& qdot);
    void set_qddot(const Eigen::VectorXd& qddot);
    void set_tau(const Eigen::VectorXd& tau);

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };
}

#endif

// cmake:sourcegroup=Api

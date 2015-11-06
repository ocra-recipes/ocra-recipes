#include "FullState.h"

#include "ocra/control/Model.h"

namespace ocra
{
  struct FullState::Pimpl
  {
    const Model& model;
    int part;
    int size;
    MatrixXd J;

    Pimpl(const Model& m, int whichPart)
      : model(m)
      , part(whichPart)
      , size(0)
    {
      switch(part)
      {
      case FULL_STATE:
        size = m.nbDofs();
        J = MatrixXd::Identity(size, size);
        break;
      case FREE_FLYER:
        size = m.nbDofs() - m.nbInternalDofs();
        J = MatrixXd::Zero(size, m.nbDofs());
        J.leftCols(size).setIdentity();
        break;
      case INTERNAL:
        size = m.nbInternalDofs();
        J = MatrixXd::Zero(size, m.nbDofs());
        J.rightCols(size).setIdentity();
        break;
      default:
        throw std::runtime_error("[FullState::FullState] invalid specified part (specify FULL_STATE, FREE_FLYER or INTERNAL");
      }
    }
  };

  FullState::FullState(const std::string& name, const Model& model, int whichPart)
    : NamedInstance(name)
    , pimpl( new Pimpl(model, whichPart) )
  {
  }

  FullState::~FullState()
  {
  }

  const Model& FullState::getModel() const
  {
    return pimpl->model;
  }

  int FullState::getSize() const
  {
    return pimpl->size;
  }

  const MatrixXd& FullState::getJacobian() const
  {
    return pimpl->J;
  }

  int FullState::whichPart() const
  {
    return pimpl->part;
  }


  
  struct FullModelState::Pimpl
  {
    MatrixXd M;
    MatrixXd Minv;
    VectorXd tau;
    VectorXd qddot;
  };

  FullModelState::FullModelState(const std::string& name, const Model& model, int whichPart)
    : FullState(name, model, whichPart)
    , pimpl( new Pimpl )
  {
    pimpl->tau = VectorXd::Zero(getSize());
    pimpl->qddot = VectorXd::Zero(getSize());
  }

  const Eigen::VectorXd& FullModelState::q() const
  {
    switch(whichPart())
    {
    case FREE_FLYER:
      return getModel().getRootConfigurationVariable().getValue();
      break;
    case INTERNAL:
      return getModel().getInternalConfigurationVariable().getValue();
      break;
    default:
      return getModel().getConfigurationVariable().getValue();
    }
  }

  const Eigen::VectorXd& FullModelState::qdot() const
  {
    switch(whichPart())
    {
    case FREE_FLYER:
      return getModel().getRootVelocityVariable().getValue();
      break;
    case INTERNAL:
      return getModel().getInternalVelocityVariable().getValue();
      break;
    default:
      return getModel().getVelocityVariable().getValue();
    }
  }

  const Eigen::VectorXd& FullModelState::qddot() const
  {
    return pimpl->qddot;
  }

  const Eigen::VectorXd& FullModelState::tau() const
  {
    return pimpl->tau;
  }

  const MatrixXd& FullModelState::getInertiaMatrix() const
  {
    switch(whichPart())
    {
    case FREE_FLYER:
      pimpl->M = getModel().getInertiaMatrix().block(0, 0, getSize(), getSize());
      return pimpl->M;
      break;
    case INTERNAL:
      pimpl->M = getModel().getInertiaMatrix().block(getModel().nbDofs()-getSize(), getModel().nbDofs()-getSize(), getSize(), getSize());
      return pimpl->M;
      break;
    default:
      return getModel().getInertiaMatrix();
    }
  }

  const MatrixXd& FullModelState::getInertiaMatrixInverse() const
  {
    switch(whichPart())
    {
    case FREE_FLYER:
      pimpl->Minv = getModel().getInertiaMatrixInverse().block(0, 0, getSize(), getSize());
      return pimpl->Minv;
      break;
    case INTERNAL:
      pimpl->Minv = getModel().getInertiaMatrixInverse().block(getModel().nbDofs()-getSize(), getModel().nbDofs()-getSize(), getSize(), getSize());
      return pimpl->Minv;
      break;
    default:
      return getModel().getInertiaMatrixInverse();
    }
  }


  struct FullTargetState::Pimpl
  {
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd qddot;
    Eigen::VectorXd tau;
    MatrixXd M;

    Pimpl(const Model& m)
    {
    }
  };
  
  FullTargetState::FullTargetState(const std::string& name, const Model& model, int whichPart)
    : FullState(name, model, whichPart)
    , pimpl( new Pimpl(model) )
  {
    pimpl->q = VectorXd::Zero(getSize());
    pimpl->qdot = VectorXd::Zero(getSize());
    pimpl->qddot = VectorXd::Zero(getSize());
    pimpl->tau = VectorXd::Zero(getSize());
    pimpl->M = MatrixXd::Zero(getSize(), getSize());
  }

  const Eigen::VectorXd& FullTargetState::q() const
  {
    return pimpl->q;
  }

  const Eigen::VectorXd& FullTargetState::qdot() const
  {
    return pimpl->qdot;
  }

  const Eigen::VectorXd& FullTargetState::qddot() const
  {
    return pimpl->qddot;
  }

  const Eigen::VectorXd& FullTargetState::tau() const
  {
    return pimpl->tau;
  }

  void FullTargetState::set_q(const Eigen::VectorXd& q)
  {
    pimpl->q = q;
  }

  void FullTargetState::set_qdot(const Eigen::VectorXd& qdot)
  {
    pimpl->qdot = qdot;
  }

  void FullTargetState::set_qddot(const Eigen::VectorXd& qddot)
  {
    pimpl->qddot = qddot;
  }

  void FullTargetState::set_tau(const Eigen::VectorXd& tau)
  {
    pimpl->tau = tau;
  }

  const MatrixXd& FullTargetState::getInertiaMatrix() const
  {
    return pimpl->M;
  }

  const MatrixXd& FullTargetState::getInertiaMatrixInverse() const
  {
    return pimpl->M;
  }
}

// cmake:sourcegroup=Api

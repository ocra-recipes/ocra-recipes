/**
 * \file wOcraPartialState.cpp
 * \author Joseph Salini
 *
 * \brief Implement partial state classes that can be used to control some joints of the robot.
 */


#include "wocra/Features/wOcraPartialState.h"

#include "ocra/control/FullState.h"

#include <iostream>

namespace wocra
{

struct PartialState::Pimpl
{
    const Model& model;
    int size;
    Eigen::MatrixXd J;
    Eigen::VectorXi dofs;
    int part;

    Pimpl(const Model& m, const Eigen::VectorXi& selectedDofs, int whichPart)
      : model(m)
      , dofs(selectedDofs.size())
      , size(selectedDofs.size())
      , part(whichPart)
    {
        J = Eigen::MatrixXd::Zero(size, model.nbDofs());

        for (int i=0; i<size; i++)
        {
            if (selectedDofs(i)<0)
                throw std::runtime_error("[PartialState::PartialState] some selected dofs are negative");
        }


        switch(part)
        {
            case ocra::FullState::FULL_STATE:
            {
                for (int i=0; i<size; i++)
                {
                    dofs(i) = selectedDofs(i);
                    J(i, dofs(i)) = 1;
                }
                break;
            }
            case ocra::FullState::INTERNAL:
            {
                int decal = model.hasFixedRoot() ? 0 : 6;
                for (int i=0; i<size; i++)
                {
                    dofs(i) = decal+selectedDofs(i);
                    J(i, dofs(i)) = 1;
                }
                break;
            }
            default:
            {
                throw std::runtime_error("[PartialState::PartialState] invalid specified part (specify FULL_STATE, or INTERNAL)");
            }
        }

    }
};

PartialState::PartialState(const std::string& name, const Model& model, const Eigen::VectorXi& selectedDofs, int whichPart)
    : NamedInstance(name)
    , pimpl( new PartialState::Pimpl(model, selectedDofs, whichPart) )
{
}

PartialState::~PartialState()
{
}

const Model& PartialState::getModel() const
{
    return pimpl->model;
}

int PartialState::getSize() const
{
    return pimpl->size;
}

const MatrixXd& PartialState::getJacobian() const
{
    return pimpl->J;
}

Eigen::VectorXi& PartialState::getDofs() const
{
    return pimpl->dofs;
}




//====================================================================

struct PartialModelState::Pimpl
{
    Eigen::MatrixXd M;
    Eigen::MatrixXd Minv;
    Eigen::VectorXd tau;
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd qddot;
};



PartialModelState::PartialModelState(const std::string& name, const Model& model, const Eigen::VectorXi& selectedDofs, int whichPart)
    : PartialState(name, model, selectedDofs, whichPart)
    , pimpl( new Pimpl )
{
    pimpl->tau   = VectorXd::Zero(getSize());
    pimpl->q     = VectorXd::Zero(getSize());
    pimpl->qdot  = VectorXd::Zero(getSize());
    pimpl->qddot = VectorXd::Zero(getSize());
}

PartialModelState::~PartialModelState()
{

}

const Eigen::VectorXd& PartialModelState::q() const
{
    Eigen::VectorXd tmp = getModel().getConfigurationVariable().getValue();

    for (int i=0; i<getSize(); i++)
        pimpl->q(i) = tmp(getDofs()(i));

    return pimpl->q;
}

const Eigen::VectorXd& PartialModelState::qdot() const
{
    Eigen::VectorXd tmp = getModel().getVelocityVariable().getValue();

    for (int i=0; i<getSize(); i++)
        pimpl->qdot(i) = tmp(getDofs()(i));

    return pimpl->qdot;
}

  const Eigen::VectorXd& PartialModelState::qddot() const
  {
    return pimpl->qddot;
  }

  const Eigen::VectorXd& PartialModelState::tau() const
  {
    return pimpl->tau;
  }

const MatrixXd& PartialModelState::getInertiaMatrix() const
{
    throw std::runtime_error("[PartialModelState::getInertiaMatrix] Not Implemented");

//    switch(whichPart())
//    {
//    case FREE_FLYER:
//      pimpl->M = getModel().getInertiaMatrix().block(0, 0, getSize(), getSize());
//      return pimpl->M;
//      break;
//    case INTERNAL:
//      pimpl->M = getModel().getInertiaMatrix().block(getModel().nbDofs()-getSize(), getModel().nbDofs()-getSize(), getSize(), getSize());
//      return pimpl->M;
//      break;
//    default:
//      return getModel().getInertiaMatrix();
//    }
}

const MatrixXd& PartialModelState::getInertiaMatrixInverse() const
{
    throw std::runtime_error("[PartialModelState::getInertiaMatrixInverse] Not Implemented");

//    switch(whichPart())
//    {
//    case FREE_FLYER:
//      pimpl->Minv = getModel().getInertiaMatrixInverse().block(0, 0, getSize(), getSize());
//      return pimpl->Minv;
//      break;
//    case INTERNAL:
//      pimpl->Minv = getModel().getInertiaMatrixInverse().block(getModel().nbDofs()-getSize(), getModel().nbDofs()-getSize(), getSize(), getSize());
//      return pimpl->Minv;
//      break;
//    default:
//      return getModel().getInertiaMatrixInverse();
//    }
}


//====================================================================

struct PartialTargetState::Pimpl
{
    Eigen::MatrixXd M;
    Eigen::MatrixXd Minv;
    Eigen::VectorXd tau;
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd qddot;
};



PartialTargetState::PartialTargetState(const std::string& name, const Model& model, const Eigen::VectorXi& selectedDofs, int whichPart)
    : PartialState(name, model, selectedDofs, whichPart)
    , pimpl( new Pimpl )
{
    pimpl->tau   = VectorXd::Zero(getSize());
    pimpl->q     = VectorXd::Zero(getSize());
    pimpl->qdot  = VectorXd::Zero(getSize());
    pimpl->qddot = VectorXd::Zero(getSize());
}

PartialTargetState::~PartialTargetState()
{

}

const Eigen::VectorXd& PartialTargetState::q() const
{
    return pimpl->q;
}

const Eigen::VectorXd& PartialTargetState::qdot() const
{
    return pimpl->qdot;
}

  const Eigen::VectorXd& PartialTargetState::qddot() const
  {
    return pimpl->qddot;
  }

  const Eigen::VectorXd& PartialTargetState::tau() const
  {
    return pimpl->tau;
  }

const MatrixXd& PartialTargetState::getInertiaMatrix() const
{
    throw std::runtime_error("[PartialModelState::getInertiaMatrix] Not Implemented");
}

const MatrixXd& PartialTargetState::getInertiaMatrixInverse() const
{
    throw std::runtime_error("[PartialModelState::getInertiaMatrixInverse] Not Implemented");
}


void PartialTargetState::set_q(const Eigen::VectorXd& q)
{
    pimpl->q = q;
}

void PartialTargetState::set_qdot(const Eigen::VectorXd& qdot)
{
    pimpl->qdot = qdot;
}

void PartialTargetState::set_qddot(const Eigen::VectorXd& qddot)
{
    pimpl->qddot = qddot;
}

void PartialTargetState::set_tau(const Eigen::VectorXd& tau)
{
    pimpl->tau = tau;
}




} // end namespace wocra

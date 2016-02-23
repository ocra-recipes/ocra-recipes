/**
 * \file gOcraFeature.cpp
 * \author Joseph Salini
 *
 * \brief Implement the partial state feature that can manage partial state instances.
 */

#include "gocra/Features/gOcraFeature.h"


namespace gocra
{

// --- ARTICULAR ----------------------------------------------

struct PartialStateFeature::Pimpl
{
    const PartialState& state;
    VectorXd error;
    VectorXd errorDot;
    VectorXd effort;
    VectorXd acceleration;
    MatrixXd J;
    MatrixXd M;
    MatrixXd Minv;
    MatrixXd spaceTransform;

    Pimpl(const PartialState& ps)
        : state(ps)
    {
        spaceTransform = MatrixXd::Identity(state.getSize(), state.getSize());
    }
  };

PartialStateFeature::PartialStateFeature(const std::string& name, const PartialState& state)
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
    return pimpl->state.getSize();
}

const VectorXd& PartialStateFeature::computeEffort(const Feature& featureDes) const
{
    const PartialStateFeature& sdes = dynamic_cast<const PartialStateFeature&>(featureDes);
    pimpl->effort = pimpl->state.tau() - sdes.pimpl->state.tau();
    return pimpl->effort;
}

const VectorXd& PartialStateFeature::computeEffort() const
{
    pimpl->effort = pimpl->state.tau();
    return pimpl->effort;
}

const VectorXd& PartialStateFeature::computeAcceleration(const Feature& featureDes) const
{
    const PartialStateFeature& sdes = dynamic_cast<const PartialStateFeature&>(featureDes);
    pimpl->acceleration = pimpl->state.qddot() - sdes.pimpl->state.qddot();
    return pimpl->acceleration;
}

const VectorXd& PartialStateFeature::computeAcceleration() const
{
    pimpl->acceleration = pimpl->state.qddot();
    return pimpl->acceleration;
}

const VectorXd& PartialStateFeature::computeError(const Feature& featureDes) const
{
    const PartialStateFeature& sdes = dynamic_cast<const PartialStateFeature&>(featureDes);
    pimpl->error = pimpl->state.q() - sdes.pimpl->state.q();
    return pimpl->error;
}

const VectorXd& PartialStateFeature::computeError() const
{
    pimpl->error = pimpl->state.q();
    return pimpl->error;
}

const VectorXd& PartialStateFeature::computeErrorDot(const Feature& featureDes) const
{
    const PartialStateFeature& sdes = dynamic_cast<const PartialStateFeature&>(featureDes);
    pimpl->errorDot = pimpl->state.qdot() - sdes.pimpl->state.qdot();
    return pimpl->errorDot;
}

const VectorXd& PartialStateFeature::computeErrorDot() const
{
    pimpl->errorDot = pimpl->state.qdot();
    return pimpl->errorDot;
}

const MatrixXd& PartialStateFeature::computeJacobian(const Feature& featureDes) const
{
    const PartialStateFeature& sdes = dynamic_cast<const PartialStateFeature&>(featureDes);
    pimpl->J = pimpl->state.getJacobian();
    return pimpl->J;
}

const MatrixXd& PartialStateFeature::computeJacobian() const
{
    pimpl->J = pimpl->state.getJacobian();
    return pimpl->J;
}

const MatrixXd& PartialStateFeature::computeProjectedMass(const Feature& featureDes) const
{
    const PartialStateFeature& sdes = dynamic_cast<const PartialStateFeature&>(featureDes);
    pimpl->M = pimpl->state.getInertiaMatrix();
    return pimpl->M;
}

const MatrixXd& PartialStateFeature::computeProjectedMass() const
{
    pimpl->M = pimpl->state.getInertiaMatrix();
    return pimpl->M;
}

const MatrixXd& PartialStateFeature::computeProjectedMassInverse(const Feature& featureDes) const
{
    const PartialStateFeature& sdes = dynamic_cast<const PartialStateFeature&>(featureDes);
    pimpl->M = pimpl->state.getInertiaMatrixInverse();
    return pimpl->M;
}

const MatrixXd& PartialStateFeature::computeProjectedMassInverse() const
{
    pimpl->M = pimpl->state.getInertiaMatrixInverse();
    return pimpl->M;
}



}

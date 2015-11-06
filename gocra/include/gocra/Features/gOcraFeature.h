/**
 * \file gOcraFeature.h
 * \author Joseph Salini
 *
 * \brief Define the partial state feature that can manage partial state instances.
 */

#ifndef __GOCRAFEATURE_H__
#define __GOCRAFEATURE_H__

// OCRA INCLUDES
#include "ocra/control/Feature.h"


// GOCRA INCLUDES
#include "gocra/Features/gOcraPartialState.h"


namespace gocra
{

/** \addtogroup feature
 * \{
 */

/** \brief A partial state feature.
 *
 * This class is greatly inspired from the \b FullStateFeature class defined in the xde framework.
 */
class PartialStateFeature: public ocra::Feature
{
public:
    PartialStateFeature(const std::string& name, const PartialState& state);

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

/** \} */ // end group feature


} //end of namespace gocra

#endif

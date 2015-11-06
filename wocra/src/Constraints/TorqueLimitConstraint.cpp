/**
 * \file TorqueLimitConstraint.cpp
 * \author Joseph Salini
 *
 * \brief Implement torque limit constraint for wOcra controller.
 */

#include "wocra/Constraints/TorqueLimitConstraint.h"



namespace wocra
{



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** Initialize a torque limit inequality function.
 *
 * \param model The ocra::Model on which we will get the dynamic parameters
 *
 * It initializes the Jacobian of the linear function, \f$ \A \f$ which is constant.
 */
TorqueLimitFunction::TorqueLimitFunction(const ocra::Model& model)
    : ocra::NamedInstance("Torque Limit Equation Function")
    , ocra::AbilitySet(ocra::PARTIAL_X)
    , ocra::CoupledInputOutputSize(false)
    , _torqueDim(model.getJointTorqueVariable().getSize())
    , LinearFunction(model.getJointTorqueVariable(), 2*model.getJointTorqueVariable().getSize())
{
    //_model.connect<EVT_CHANGE_VALUE>(*this, &DynamicEquationFunction::invalidateAll); // not necessary  because A (jacobian) is constant
    //_model.connect<ocra::EVT_CHANGE_VALUE>(*this, &JointLimitFunction::invalidateb);   // b is constant, unless someone changes it by calling setTorqueLimits

    /* The function is on the form:
     *      | O -I |         |  taumax |
     *      | O  I | tau  +  | -taumax |
     *
     * where O (nx6) is defined if the model has a free-floating base (we cannot limit the range of this "joint"), else O is a void matrix (nx0)
     *
     * The associated constraint is:
     *      A ddq + b >= 0
     */

    // Building of A, the jacobian, which is constant:
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(_torqueDim, _torqueDim);

    _jacobian = Eigen::MatrixXd::Zero(2*_torqueDim, _torqueDim);
    _jacobian.topRows(_torqueDim)    = -I;
    _jacobian.bottomRows(_torqueDim) =  I;

    // Set limit from model
    setTorqueLimits( Eigen::VectorXd::Constant(_torqueDim, 1000) );
}

/** Destructor
 *
 */
TorqueLimitFunction::~TorqueLimitFunction()
{

}

/** Update the Jacobian matrix (\f$ \A \f$) in the function.
 *
 * Nothing to do.
 */
void TorqueLimitFunction::updateJacobian() const
{

}

/** Update the vector (\f$ \b \f$) in the function.
 *
 * It computes \f$ \b = \begin{bmatrix} \torque_{max} \\ \torque_{max} \end{bmatrix} \f$ .
 */
void TorqueLimitFunction::updateb() const
{
    _b.head(_torqueDim) =   _torqueLimits;
    _b.tail(_torqueDim) =   _torqueLimits;
}


/** Set the torque limit.
 *
 * \param torqueLimits The torque limit \f$ \torque_{max} \f$
 */
void  TorqueLimitFunction::setTorqueLimits(const Eigen::VectorXd& torqueLimits)
{
    _torqueLimits = torqueLimits;
    updateb();
}

/** Get the torque limit.
 *
 * \return The torque limit \f$ \torque_{max} \f$
 */
const Eigen::VectorXd& TorqueLimitFunction::getTorqueLimits() const
{
    return _torqueLimits;
}




} // end namespace wocra



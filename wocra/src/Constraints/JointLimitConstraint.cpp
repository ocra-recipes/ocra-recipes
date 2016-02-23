/**
 * \file JointLimitConstraint.cpp
 * \author Joseph Salini
 *
 * \brief Implement joint limit constraint for wOcra controller.
 */

#include "wocra/Constraints/JointLimitConstraint.h"


using namespace wocra;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** Initialize the joint limit constraint function.
 *
 * \param model  The ocra::Model on which we will get the dynamic parameters
 * \param var    The problem variable that will be used to write this constraint
 *
 * This class is a generic class to compute matrices for the joint limit function, but <b>it should not be used</b>.
 * You would rather choose one of the following derivative classes, depending on the choosen formalism,
 * either full \f$ \x = [ \ddq \; \torque \; \force_c ] \f$ or reduced \f$ \x = [ \torque \; \force_c ] \f$:
 *
 *      - wocra::FullJointLimitFunction
 *      - wocra::ReducedJointLimitFunction
 *
 * The function is on the form:
 *      | O -I |         |  ddqmax |
 *      | O  I | ddq  +  |  ddqmin |
 *
 * where O (nx6) is defined if the model has a free-floating base (we cannot limit the range of this "joint"), else O is a void matrix (nx0)
 *
 * The associated constraint is:
 *      A ddq + b >= 0
 */

JointLimitFunction::JointLimitFunction(const ocra::Model& model, ocra::Variable& var)
    : ocra::NamedInstance("Joint Limit Equation Function")
    , ocra::CoupledInputOutputSize(false)
    , LinearFunction(var, 2*model.nbInternalDofs())
    , hpos(.2)
{
    // Set limit from model
    setJointLowerLimits( model.getJointLowerLimits() );
    setJointUpperLimits( model.getJointUpperLimits() );
}

/** Destructor
 *
 */
JointLimitFunction::~JointLimitFunction()
{

}

/** Get the time horizon of prediction \f$ h \f$ for the joint limit function.
 *
 * \return The time horizon (s)
 */
double JointLimitFunction::getHorizonOfPrediction() const
{
    return hpos;
}

/** Set the time horizon of prediction \f$ h \f$ for the joint limit function.
 *
 * \param newHpos The new time horizon (s)
 */
void   JointLimitFunction::setHorizonOfPrediction(double newHpos)
{
    hpos = newHpos;
}

/** Set the joint limits \f$ \q_{min}, \q_{max} \f$.
 *
 * \param lowerLimits  The lower bound
 * \param upperLimits  The upper bound
 */
void JointLimitFunction::setJointLimits(const Eigen::VectorXd& lowerLimits, const Eigen::VectorXd& upperLimits)
{
    setJointLowerLimits(lowerLimits);
    setJointUpperLimits(upperLimits);
}

/** Set the joint limit \f$ \q_{min} \f$.
 *
 * \param newLowerLimits The lower bound
 */
void JointLimitFunction::setJointLowerLimits(const Eigen::VectorXd& newLowerLimits)
{
    jointLowerLimits = newLowerLimits;
}

/** Set the joint limits \f$ \q_{max} \f$.
 *
 * \param newUpperLimits The upper bound
 */
void JointLimitFunction::setJointUpperLimits(const Eigen::VectorXd& newUpperLimits)
{
    jointUpperLimits = newUpperLimits;
}

/** Set the joint limits for one dof \f$ \q_{min}[i], \q_{max}[i] \f$.
 *
 * \param i              The dof index whose bounds are modified
 * \param newLowerLimit  The lower bound
 * \param newUpperLimit  The upper bound
 */
void JointLimitFunction::setJointLimit(int i, double newLowerLimit, double newUpperLimit)
{
    setJointLowerLimit(i, newLowerLimit);
    setJointUpperLimit(i, newUpperLimit);
}

/** Set the joint limit for one dof \f$ \q_{min}[i] \f$.
 *
 * \param i              The dof index whose bound is modified
 * \param newLowerLimit  The lower bound
 */
void JointLimitFunction::setJointLowerLimit(int i, double newLowerLimit)
{
    jointLowerLimits(i) = newLowerLimit;
}

/** Set the joint limit for one dof \f$ \q_{max}[i] \f$.
 *
 * \param i              The dof index whose bound is modified
 * \param newUpperLimit  The upper bound
 */
void JointLimitFunction::setJointUpperLimit(int i, double newUpperLimit)
{
    jointUpperLimits(i) = newUpperLimit;
}

/** Get the joint limit \f$ \q_{min} \f$.
 *
 * \return The lower bound
 */
const Eigen::VectorXd& JointLimitFunction::getJointLowerLimits() const
{
    return jointLowerLimits;
}

/** Get the joint limit \f$ \q_{max} \f$.
 *
 * \return The upper bound
 */
const Eigen::VectorXd& JointLimitFunction::getJointUpperLimits() const
{
    return jointUpperLimits;
}

/** Get the joint limit for one dof \f$ \q_{min}[i] \f$.
 *
 * \param i The dof index
 * \return The lower bound
 */
double JointLimitFunction::getJointLowerLimit(int i) const
{
    return jointLowerLimits(i);
}

/** Get the joint limit for one dof \f$ \q_{max}[i] \f$.
 *
 * \param i The dof index
 * \return The upper bound
 */
double JointLimitFunction::getJointUpperLimit(int i) const
{
    return jointUpperLimits(i);
}

/** Compute the Jacobian matrix for the joint limits constraint.
 *
 * \param varDofs       The variable dimension
 * \param nIDofs        Number of internal dof of the robot
 * \param fullJacobian  The jacobian matrix instance where to write the jacobian in the full formalism
 *
 * This function compute matrix \f$ \A \f$ for the full formalism. When using the reduced one, this matrix is first computed,
 * and then transformed (see \ref sec_transform_formalism) to fit the new variable.
 *
 * The computation of \f$ \A \f$ is explained in wocra::JointLimitFunction
 */
void JointLimitFunction::computeFullJacobian(int varDofs, int nIDofs, Eigen::MatrixXd& fullJacobian) const
{
    Eigen::MatrixXd Icut = Eigen::MatrixXd::Identity(varDofs, varDofs).bottomRows(nIDofs);

    fullJacobian                    =  Eigen::MatrixXd::Zero(2*nIDofs, varDofs);
    fullJacobian.topRows(nIDofs)    = -Icut;
    fullJacobian.bottomRows(nIDofs) =  Icut;
}

/** Compute the vector \f$ \b \f$ for the joint limits constraint.
 *
 * \param gpos   The genralized position
 * \param gvel   The generalized velocity
 * \param fullb  The vector instance where to write the limits vector in the full formalism
 *
 * This function compute vector \f$ \b \f$ for the full formalism. When using the reduced one, this vector is first computed,
 * and then transformed (see \ref sec_transform_formalism) to fit the new variable.
 *
 * The computation of \f$ \b \f$ is explained in wocra::JointLimitFunction
 */
void JointLimitFunction::computeFullb(const Eigen::VectorXd& gpos, const Eigen::VectorXd& gvel, Eigen::VectorXd& fullb) const
{
    // We compute the min and max accelerations of q to avoid joint limits:
    // there is two min and max:
    // - the first one is computed of a horizon h. The system brakes far from the limit, but can pass inside the constraint when close to it.
    // - the second one is computed with the time of inflexion. The system brakes when close to the limits, but some singularities appears when it is on the limits.
    //
    // The two computation have their drawbacks, but together they allow limits avoidance properly.

    int nIDofs = gpos.size();

    Eigen::VectorXd maxDdqTotal = 2 * (jointUpperLimits - gpos - hpos * gvel)/(hpos*hpos);
    Eigen::VectorXd minDdqTotal = 2 * (jointLowerLimits - gpos - hpos * gvel)/(hpos*hpos);


//    tmin = -2*(gpos - qlim[:, 0]) / gvel
//    tmax = -2*(gpos - qlim[:, 1]) / gvel
//
//    tmin[tmin <=  0  ] = nan
//    tmin[tmin > hpos] = nan
//    tmin[isfinite(tmin)] = 1.
//    tmax[tmax <=  0  ] = nan
//    tmax[tmax > hpos] = nan
//    tmax[isfinite(tmax)] = 1.

//    bmin_pos_time = gvel**2/(2*(gpos - qlim[:, 0])) * tmin
//    bmax_pos_time = gvel**2/(2*(gpos - qlim[:, 1])) * tmax


    for (int i=0; i<nIDofs; i++)
    {
        double tmax = -2*(gpos(i) - jointUpperLimits(i)) / gvel(i);
        double tmin = -2*(gpos(i) - jointLowerLimits(i)) / gvel(i);

        if ((0 <= tmax) && (tmax <= hpos))
        {
            double ddqlim = (gvel(i)*gvel(i))/(2*(gpos(i) - jointUpperLimits(i)));
            if (maxDdqTotal(i) >= ddqlim)
            {
                maxDdqTotal(i) = ddqlim;
            }
        }
        if ((0 <= tmin) && (tmin <= hpos))
        {
            double ddqlim = (gvel(i)*gvel(i))/(2*(gpos(i) - jointLowerLimits(i)));
            if (minDdqTotal(i) <= ddqlim)
            {
                minDdqTotal(i) = ddqlim;
            }
        }
    }

    fullb.head(nIDofs) =   maxDdqTotal;
    fullb.tail(nIDofs) = - minDdqTotal;

}

















struct FullJointLimitFunction::Pimpl
{
    const ocra::Model&   _model;

    Pimpl(const ocra::Model& model)
        : _model(model)
    {

    }
};


/** Initialize a joint limits function designed for the full formalism.
 *
 * \param model The ocra::Model on which we will update the dynamic parameters
 *
 * It is connected with the model and invalidates \f$ \b \f$ when ocra::EVT_CHANGE_VALUE is raised.
 * Furthermore, it computes the Jacobian matrix \f$ \A \f$ which is constant.
 */
FullJointLimitFunction::FullJointLimitFunction(const ocra::Model& model)
    : ocra::NamedInstance("Full Joint Limit Equation Function")
    , ocra::AbilitySet(ocra::PARTIAL_X)
    , ocra::CoupledInputOutputSize(false)
    , JointLimitFunction(model, model.getAccelerationVariable())
    , pimpl(new Pimpl(model))
{
    //_model.connect<EVT_CHANGE_VALUE>(*this, &FullJointLimitFunction::invalidateAll); // not necessary  because A (jacobian) is constant
    pimpl->_model.connect<ocra::EVT_CHANGE_VALUE>(*this, &FullJointLimitFunction::invalidateb); //to update b at each time step

    int varDofs = model.getAccelerationVariable().getSize();

    computeFullJacobian(varDofs, pimpl->_model.nbInternalDofs(), _jacobian);

}

/** Destructor
 *
 * It is disconnected from the model.
 */
FullJointLimitFunction::~FullJointLimitFunction()
{
    //_model.disconnect<EVT_CHANGE_VALUE>(*this, &FullJointLimitFunction::invalidateAll);
    pimpl->_model.disconnect<ocra::EVT_CHANGE_VALUE>(*this, &FullJointLimitFunction::invalidateb);
}

/** Update the vector \f$ \b \f$ for the full formalism.
 *
 * It calls #computeFullb(const Eigen::VectorXd&, const Eigen::VectorXd&, Eigen::VectorXd&) const.
 */
void FullJointLimitFunction::updateb() const
{
    computeFullb(pimpl->_model.getJointPositions(), pimpl->_model.getJointVelocities(), _b);
}

















struct ReducedJointLimitFunction::Pimpl
{
    const ocra::Model&          _model;
    const wOcraDynamicFunction& _dynamicEquation;

    Eigen::MatrixXd            _fullJacobian;

    Pimpl(const ocra::Model& model, const wOcraDynamicFunction& dynamicEquation)
        : _model(model)
        , _dynamicEquation(dynamicEquation)
    {

    }
};


/** Initialize a joint limits function designed for the reduced formalism.
 *
 * \param model            The ocra::Model on which we will update the dynamic parameters
 * \param dynamicEquation  The dynamic equation of motion that can compute matrices for the reduced problem
 *
 * It is connected with the model and invalidates all when ocra::EVT_CHANGE_VALUE is raised.
 * Furthermore, it computes the Jacobian matrix \f$ \A \f$ for the full formalism which is constant. It will be transformed to fit the reduced formalism.
 */
ReducedJointLimitFunction::ReducedJointLimitFunction(const ocra::Model& model, const wOcraDynamicFunction& dynamicEquation)
    : ocra::NamedInstance("Reduced Joint Limit Equation Function")
    , ocra::AbilitySet(ocra::PARTIAL_X)
    , ocra::CoupledInputOutputSize(false)
    , JointLimitFunction(model, dynamicEquation.getActionVariable())
    , pimpl(new Pimpl(model, dynamicEquation))
{
    pimpl->_model.connect<ocra::EVT_CHANGE_VALUE>(*this, &ReducedJointLimitFunction::invalidateAll);
    pimpl->_model.connect<ocra::EVT_CHANGE_VALUE>(*this, &ReducedJointLimitFunction::invalidateb); //to update b at each time step

    computeFullJacobian(pimpl->_model.nbDofs(), pimpl->_model.nbInternalDofs(), pimpl->_fullJacobian);
}

/** Destructor
 *
 * It is disconnected from the model.
 */
ReducedJointLimitFunction::~ReducedJointLimitFunction()
{
    pimpl->_model.disconnect<ocra::EVT_CHANGE_VALUE>(*this, &ReducedJointLimitFunction::invalidateAll);
    pimpl->_model.disconnect<ocra::EVT_CHANGE_VALUE>(*this, &ReducedJointLimitFunction::invalidateb);
}



void ReducedJointLimitFunction::updateJacobian() const
{

    _jacobian = - pimpl->_fullJacobian * pimpl->_dynamicEquation.getInertiaMatrixInverseJchiT();
}


/** Update the vector \f$ \b \f$ for the reduced formalism.
 *
 * It calls #computeFullb(const Eigen::VectorXd&, const Eigen::VectorXd&, Eigen::VectorXd&) const.
 * Then the transformation is done from the full formalism to the reduced one.
 */
void ReducedJointLimitFunction::updateb() const
{
    Eigen::VectorXd _fullb(_b.size());
    computeFullb(pimpl->_model.getJointPositions(), pimpl->_model.getJointVelocities(), _fullb);

    //_jacobian =          pimpl->_fullJacobian * pimpl->_model.getInertiaMatrixInverseJchiT();
    _b        = _fullb - pimpl->_fullJacobian * pimpl->_dynamicEquation.getInertiaMatrixInverseLinNonLinGrav();
}



/** Do when input size changes, before.
 *
 * By overloading this function, it allows linear function modification when input size changes.
 * Does nothing actually.
 */
void ReducedJointLimitFunction::doUpdateInputSizeBegin()
{
    //do nothing : this overload allows to resize
}

/** Do when input size changes, after.
 *
 * By overloading this function, it allows linear function modification when input size changes.
 * Does nothing actually.
 */
void ReducedJointLimitFunction::doUpdateInputSizeEnd()
{
    computeFullJacobian(pimpl->_model.nbDofs(), pimpl->_model.nbInternalDofs(), pimpl->_fullJacobian);
}














JointLimitConstraint::JointLimitConstraint(const ocra::Model& model, double hpos)
    : _model(model)
    , _is_connected(false)
    , wOcraConstraint()
{
    setHorizonOfPrediction(hpos);
    setJointLimits(model.getJointLowerLimits(), model.getJointUpperLimits() );
}

JointLimitConstraint::JointLimitConstraint(const ocra::Model& model, const Eigen::VectorXd& lowerLimits, const Eigen::VectorXd& upperLimits, double hpos)
    : _model(model)
    , _is_connected(false)
    , wOcraConstraint()
{
    setHorizonOfPrediction(hpos);
    setJointLimits(lowerLimits, upperLimits);
}



double JointLimitConstraint::getHorizonOfPrediction() const
{
    return _hpos;
}
const Eigen::VectorXd& JointLimitConstraint::getJointLowerLimits() const
{
    return _lowerLimits;
}

const Eigen::VectorXd& JointLimitConstraint::getJointUpperLimits() const
{
    return _upperLimits;
}

double JointLimitConstraint::getJointLowerLimit(int i) const
{
    return _lowerLimits(i);
}

double JointLimitConstraint::getJointUpperLimit(int i) const
{
    return _upperLimits(i);
}

void   JointLimitConstraint::setHorizonOfPrediction(double newHpos)
{
    _hpos = newHpos;
    if (_is_connected)
        _jointLimitFunction->setHorizonOfPrediction(newHpos);
}

void  JointLimitConstraint::setJointLimits(const Eigen::VectorXd& lowerLimits, const Eigen::VectorXd& upperLimits)
{
    _lowerLimits = lowerLimits;
    _upperLimits = upperLimits;
    if (_is_connected)
        _jointLimitFunction->setJointLimits(lowerLimits, upperLimits);
}

void  JointLimitConstraint::setJointLowerLimits(const Eigen::VectorXd& newLowerLimits)
{
    _lowerLimits = newLowerLimits;
    if (_is_connected)
        _jointLimitFunction->setJointLowerLimits(newLowerLimits);
}

void  JointLimitConstraint::setJointUpperLimits(const Eigen::VectorXd& newUpperLimits)
{
    _upperLimits = newUpperLimits;
    if (_is_connected)
        _jointLimitFunction->setJointUpperLimits(newUpperLimits);
}

void  JointLimitConstraint::setJointLimit(int i, double newLowerLimit, double newUpperLimit)
{
    _lowerLimits(i) = newLowerLimit;
    _upperLimits(i) = newUpperLimit;
    if (_is_connected)
        _jointLimitFunction->setJointLimit(i, newLowerLimit, newUpperLimit);
}
void  JointLimitConstraint::setJointLowerLimit(int i, double newLowerLimit)
{
    _lowerLimits(i) = newLowerLimit;
    if (_is_connected)
        _jointLimitFunction->setJointLowerLimit(i, newLowerLimit);
}

void  JointLimitConstraint::setJointUpperLimit(int i, double newUpperLimit)
{
    _upperLimits(i) = newUpperLimit;
    if (_is_connected)
        _jointLimitFunction->setJointUpperLimit(i, newUpperLimit);
}


void JointLimitConstraint::connectToController(const wOcraDynamicFunction& dynamicEquation, bool useReducedProblem)
{
    ocra::LinearFunction* f = NULL;
    if (useReducedProblem)
        f = createReducedJointLimitFunction(_model, dynamicEquation);
    else
        f = createFullJointLimitFunction(_model);

    _constraint.reset(new ocra::LinearConstraint(f));
    _is_connected = true;

    setHorizonOfPrediction(_hpos);
    setJointLowerLimits( _lowerLimits );
    setJointUpperLimits( _upperLimits );
}

void JointLimitConstraint::disconnectFromController()
{
    _is_connected = false;
}


JointLimitFunction* JointLimitConstraint::createFullJointLimitFunction(const ocra::Model& model)
{
    _jointLimitFunction = new FullJointLimitFunction(model);
    return _jointLimitFunction;
}

JointLimitFunction* JointLimitConstraint::createReducedJointLimitFunction(const ocra::Model& model, const wOcraDynamicFunction& dynamicEquation)
{
    _jointLimitFunction = new ReducedJointLimitFunction(model, dynamicEquation);
    return _jointLimitFunction;
}



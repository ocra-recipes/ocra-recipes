/**
 * \file ContactAvoidanceConstraint.cpp
 * \author Joseph Salini
 *
 * \brief Implement contact avoidance constraint for wOcra controller.
 */

#include "wocra/Constraints/ContactAvoidanceConstraint.h"


using namespace wocra;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** Initialize the contact avoidance constraint function.
 *
 * \param model  The ocra::Model on which we will get the dynamic parameters
 * \param var    The problem variable that will be used to write this constraint
 *
 * This class is a generic class to compute matrices for the contact avoidance function, but <b>it should not be used</b>.
 * You would rather choose one of the following derivative classes, depending on the choosen formalism,
 * either full \f$ \x = [ \ddq \; \torque \; \force_c ] \f$ or reduced \f$ \x = [ \torque \; \force_c ] \f$:
 *
 *      - wocra::FullContactAvoidanceFunction
 *      - wocra::ReducedContactAvoidanceFunction
 */
ContactAvoidanceFunction::ContactAvoidanceFunction(const ocra::Model& model, ocra::Variable& var)
    : ocra::NamedInstance("wOcra Contact Avoidance Function")
    , ocra::CoupledInputOutputSize(false)
    , LinearFunction(var, 0)
//    , _model(model)
//    , _q_ddot(model.getAccelerationVariable())
    , hpos(.5)
    , margin(.05)
{
    model.connect<ocra::EVT_CHANGE_VALUE>(*this, &ContactAvoidanceFunction::invalidateAll);
    model.connect<ocra::EVT_CHANGE_VALUE>(*this, &ContactAvoidanceFunction::invalidateb);
}

/** Destructor
 *
 */
ContactAvoidanceFunction::~ContactAvoidanceFunction()
{

}

/** Get the time horizon of prediction \f$ h \f$ for the contact avoidance function.
 *
 * \return The time horizon (s)
 */
double ContactAvoidanceFunction::getHorizonOfPrediction() const
{
    return hpos;
}

/** Set the time horizon of prediction \f$ h \f$ for the contact avoidance function.
 *
 * \param newHpos The new time horizon (s)
 */
void   ContactAvoidanceFunction::setHorizonOfPrediction(double newHpos)
{
    hpos = newHpos;
}

/** Get the obstacle avoidance margin \f$ \vec{m} \f$.
 *
 * \return The margin vector
 */
double ContactAvoidanceFunction::getMargin() const
{
    return margin;
}

/** Set the obstacle avoidance margin \f$ \vec{m} \f$.
 *
 * \param newMargin The margin vector
 */
void   ContactAvoidanceFunction::setMargin(double newMargin)
{
    margin = newMargin;
}

/** Update contact information to compute obstacle avoidance function.
 *
 * \param _JObst     The Jacobian of obstacle avoidance
 * \param _dJdqObst  The derivative of the Jacobian of obstacle avoidance multiplied by generalized velocity \f$ = \dJ_{Obst} \dq  \f$
 * \param _distObst  The relative distance of obstacle avoidance
 * \param _velObst   The relative velocity of obstacle avoidance
 */
void ContactAvoidanceFunction::updateContactInformation(const Eigen::MatrixXd& _JObst, const Eigen::VectorXd& _dJdqObst, const Eigen::VectorXd& _distObst, const Eigen::VectorXd& _velObst)
{
    JObst    = _JObst;
    dJdqObst = _dJdqObst;
    distObst = _distObst;
    velObst  = _velObst;

    changeFunctionDimension(_distObst.size());
}

/** Compute the Jacobian matrix \f$ \A \f$ of the linear function for the collision avoidance constraint expressed in the full formalism.
 *
 * \param _JObst        The Jacobian of obstacle avoidance
 * \param fullJacobian  The matrix instance where to write the collision avoidance data in the full formalism
 */
void ContactAvoidanceFunction::computeFullJacobian(const Eigen::MatrixXd& _JObst, Eigen::MatrixXd& fullJacobian) const
{
    fullJacobian = - _JObst;
}

/** Compute the vector \f$ \b \f$ of the linear function for the collision avoidance constraint expressed in the full formalism.
 *
 * \param _dJdqObst  The derivative of the Jacobian of obstacle avoidance multiplied by generalized velocity \f$ = \dJ_{Obst} \dq  \f$
 * \param _distObst  The relative distance of obstacle avoidance
 * \param _velObst   The relative velocity of obstacle avoidance
 * \param fullb      The vector instance where to write the collision avoidance data in the full formalism
 */
void ContactAvoidanceFunction::computeFullb(const Eigen::VectorXd& _dJdqObst, const Eigen::VectorXd& _distObst, const Eigen::VectorXd& _velObst, Eigen::VectorXd& fullb) const
{
    fullb = - _dJdqObst + ( (_distObst - Eigen::VectorXd::Constant(_distObst.size(), margin)) - _velObst * hpos ) * 2/(hpos*hpos);


    for (int i=0; i<_distObst.size(); i++)
    {
        double tmax = 2*(_distObst(i) - margin) / _velObst(i);
        if ((0 <= tmax) && (tmax <= hpos))
        {
            double ddqlim = - ( _velObst(i)*_velObst(i) ) / ( 2.*(_distObst(i) - margin) );
            if (fullb(i) >= ddqlim)
            {
                fullb(i) = ddqlim;
            }
        }
    }
}

/** update the Jacobian matrix \f$ \A \f$.
 *
 * Does nothing.
 *
 * \todo It should be conform to the ocra framework, the computation of the jacobian should be here.
 */
void ContactAvoidanceFunction::updateJacobian() const
{
    //TODO change _jacobian
}

/** update the vector \f$ \b \f$.
 *
 * Does nothing.
 *
 * \todo It should be conform to the ocra framework, the computation of the vector should be here.
 */
void ContactAvoidanceFunction::updateb() const
{
    //TODO change _b
}

/** Do when linear function dimension changes, before.
 *
 * By overloading this function, it allows linear function modification when function size changes.
 * It does nothing actually.
 */
void ContactAvoidanceFunction::doUpdateDimensionBegin(int newDimension)
{

}

/** Do when linear function dimension changes, after.
 *
 * By overloading this function, it allows linear function modification when function size changes.
 * It does nothing actually.
 */
void ContactAvoidanceFunction::doUpdateDimensionEnd(int oldDimension)
{

}











///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** Initialize a contact avoidance function designed for the full formalism.
 *
 * \param model The ocra::Model on which we will update the dynamic parameters
 */
FullContactAvoidanceFunction::FullContactAvoidanceFunction(const ocra::Model& model)
    : ocra::NamedInstance("Full Contact Avoidance Function")
    , ocra::AbilitySet(ocra::PARTIAL_X)
    , ocra::CoupledInputOutputSize(false)
    , ContactAvoidanceFunction(model, model.getAccelerationVariable())
    //, pimpl(new Pimpl(model))
{

}

/** Destructor
 *
 */
FullContactAvoidanceFunction::~FullContactAvoidanceFunction()
{

}

/** Update the Jacobian matrix \f$ \A \f$ for the linear function.
 *
 * It calls #computeFullJacobian(const Eigen::MatrixXd&, Eigen::MatrixXd&) const.
 */
void FullContactAvoidanceFunction::updateJacobian() const
{
    computeFullJacobian(JObst, _jacobian);
}

/** Update the vector \f$ \b \f$ for the linear function.
 *
 * It calls #computeFullb(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, Eigen::VectorXd&) const.
 */
void FullContactAvoidanceFunction::updateb() const
{
    computeFullb(dJdqObst, distObst, velObst, _b);
}













/////////////////////////////////////////////////////////////////////////////////////////////////////////
struct ReducedContactAvoidanceFunction::Pimpl
{
    const ocra::Model&           _model;
    const wOcraDynamicFunction&  _dynamicEquation;

    Eigen::MatrixXd             _fullJacobian;
    Eigen::VectorXd             _fullb;

    Pimpl(const ocra::Model& model, const wOcraDynamicFunction& dynamicEquation)
        : _model(model)
        , _dynamicEquation(dynamicEquation)
    {

    }
};

/** Initialize a contact avoidance function designed for the reduced formalism.
 *
 * \param model            The ocra::Model on which we will update the dynamic parameters
 * \param dynamicEquation  The dynamic equation of motion that can compute matrices for the reduced problem
 */
ReducedContactAvoidanceFunction::ReducedContactAvoidanceFunction(const ocra::Model& model, const wOcraDynamicFunction& dynamicEquation)
    : ocra::NamedInstance("Reduced Contact Avoidance Function")
    , ocra::AbilitySet(ocra::PARTIAL_X)
    , ocra::CoupledInputOutputSize(false)
    , ContactAvoidanceFunction(model, dynamicEquation.getActionVariable())
    , pimpl(new Pimpl(model, dynamicEquation))
{
    pimpl->_fullJacobian.setZero(_jacobian.rows(), _jacobian.cols());
    pimpl->_fullb.setZero(_b.size());
}

/** Destructor
 *
 */
ReducedContactAvoidanceFunction::~ReducedContactAvoidanceFunction()
{

}


/** Update the vector \f$ \b \f$ for the reduced formalism.
 *
 * It calls #computeFullJacobian(const Eigen::MatrixXd&, Eigen::MatrixXd&) const,
 * and #computeFullb(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, Eigen::VectorXd&) const.
 * Then the transformation is done from the full formalism to the reduced one.
 */
void ReducedContactAvoidanceFunction::updateb() const
{
    computeFullJacobian(JObst, pimpl->_fullJacobian);
    computeFullb(dJdqObst, distObst, velObst, pimpl->_fullb);

    _jacobian =               - pimpl->_fullJacobian * pimpl->_dynamicEquation.getInertiaMatrixInverseJchiT();
    _b        = pimpl->_fullb - pimpl->_fullJacobian * pimpl->_dynamicEquation.getInertiaMatrixInverseLinNonLinGrav();
}


/** Do when input size changes, before.
 *
 * By overloading this function, it allows linear function modification when input size changes.
 * It does nothing actually.
 */
void ReducedContactAvoidanceFunction::doUpdateInputSizeBegin()
{
    //do nothing : this overload allows to resize
}

/** Do when input size changes, after.
 *
 * By overloading this function, it allows linear function modification when input size changes.
 */
void ReducedContactAvoidanceFunction::doUpdateInputSizeEnd()
{
    pimpl->_fullJacobian.setZero(_jacobian.rows(), _jacobian.cols());
    pimpl->_fullb.setZero(_b.size());
}







ContactAvoidanceConstraint::ContactAvoidanceConstraint(const ocra::Model& model, double hpos, double margin)
    : _model(model)
    , _is_connected(false)
    , wOcraConstraint()
{
    setHorizonOfPrediction(hpos);
    setMargin(margin);
}

double ContactAvoidanceConstraint::getHorizonOfPrediction() const
{
    return _hpos;
}

void   ContactAvoidanceConstraint::setHorizonOfPrediction(double newHpos)
{
    _hpos = newHpos;
    if (_is_connected)
        _contactAvoidanceFunction->setHorizonOfPrediction(newHpos);
}


double ContactAvoidanceConstraint::getMargin() const
{
    return _margin;
}

void   ContactAvoidanceConstraint::setMargin(double newMargin)
{
    _margin = newMargin;
    if (_is_connected)
        _contactAvoidanceFunction->setMargin(newMargin);
}


void ContactAvoidanceConstraint::updateContactInformation(const Eigen::MatrixXd& _JObst, const Eigen::VectorXd& _dJdqOst, const Eigen::VectorXd& _distObst, const Eigen::VectorXd& _velObst)
{
    if (_is_connected)
        _contactAvoidanceFunction->updateContactInformation(_JObst, _dJdqOst, _distObst, _velObst);
    else
        throw std::runtime_error("[ContactAvoidanceConstraint::updateContactInformation] Constraint is not connected to the controller!");
}


void ContactAvoidanceConstraint::connectToController(const wOcraDynamicFunction& dynamicEquation, bool useReducedProblem)
{
    ocra::LinearFunction* f = NULL;
    if (useReducedProblem)
        f = createReducedContactAvoidanceFunction(_model, dynamicEquation);
    else
        f = createFullContactAvoidanceFunction(_model);

    _constraint.reset(new ocra::LinearConstraint(f));
    _is_connected = true;

    setHorizonOfPrediction(_hpos);
    setMargin(_margin);
}

void ContactAvoidanceConstraint::disconnectFromController()
{
    _is_connected = false;
}

ContactAvoidanceFunction* ContactAvoidanceConstraint::createFullContactAvoidanceFunction(const ocra::Model& model)
{
    _contactAvoidanceFunction = new FullContactAvoidanceFunction(model);
    return _contactAvoidanceFunction;
}

ContactAvoidanceFunction* ContactAvoidanceConstraint::createReducedContactAvoidanceFunction(const ocra::Model& model, const wOcraDynamicFunction& dynamicEquation)
{
    _contactAvoidanceFunction = new ReducedContactAvoidanceFunction(model, dynamicEquation);
    return _contactAvoidanceFunction;
}




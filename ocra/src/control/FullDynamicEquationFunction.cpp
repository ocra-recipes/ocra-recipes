/**
 * \file FullDynamicEquationFunction.cpp
 * \author Joseph Salini
 *
 * \brief Implement base class that can be used as constraints in wOcra controller.
 */

#include "ocra/control/FullDynamicEquationFunction.h"


using namespace ocra;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct FullDynamicEquationFunction::Pimpl
{
    const Model&  _model;
    Variable&     _tau;
    Variable&     _fc;

    // ADD ACTION VARIABLE
    CompositeVariable _chi;
    bool _reducedProblemDataAreUpToDate;

    Eigen::MatrixXd  _InertiaMatrixInverseJchiT;
    Eigen::VectorXd  _InertiaMatrixInverseLinNonLinGrav;
    bool             _useGrav;


    Pimpl(const Model& model)
        : _model(model)
        , _tau(model.getJointTorqueVariable())
        , _fc(model.getModelContacts().getContactForcesVariable())
        , _chi("chi", model.getJointTorqueVariable(), model.getModelContacts().getContactForcesVariable())
        , _reducedProblemDataAreUpToDate(false)
        , _useGrav(true)
        , _InertiaMatrixInverseJchiT(model.nbDofs(), model.nbDofs())
        , _InertiaMatrixInverseLinNonLinGrav(model.nbDofs())
    {

    }

    void updateReduceProblemData(const Eigen::MatrixXd& _jacobian, const Eigen::VectorXd& _b)
    {
        const Eigen::MatrixXd& Minv  = _model.getInertiaMatrixInverse();
        const Eigen::MatrixXd& JchiT = _jacobian.rightCols(_chi.getSize());

        _InertiaMatrixInverseJchiT          = Minv * JchiT;
        _InertiaMatrixInverseLinNonLinGrav  = Minv * _b;

        _reducedProblemDataAreUpToDate = true;
    }
};

/** Initialize a dynamic equation of motion function.
 *
 * \param model The Model on which we will update the dynamic parameters
 *
 * It is connected with the model on "EVT_CHANGE_VALUE", meaning that a,y modification in the model will invalidate this linear function.
 */
 FullDynamicEquationFunction::FullDynamicEquationFunction(const Model& model)
    : NamedInstance("Full Dynamic Equation Function")
    , AbilitySet(PARTIAL_X)
    , CoupledInputOutputSize(false)
    , LinearFunction(createDEVariable(model), model.nbDofs())
    , pimpl( new Pimpl(model))
{
    buildA();

    pimpl->_model.connect<EVT_CHANGE_VALUE>(*this, &FullDynamicEquationFunction::invalidateAll);
    pimpl->_model.connect<EVT_CHANGE_VALUE>(*this, &FullDynamicEquationFunction::invalidateb);
    pimpl->_model.connect<EVT_CHANGE_VALUE>(*this, &FullDynamicEquationFunction::invalidateReducedProblemData);
}

/** Destructor
 *
 */
 FullDynamicEquationFunction::~FullDynamicEquationFunction()
{
    pimpl->_model.disconnect<EVT_CHANGE_VALUE>(*this, &FullDynamicEquationFunction::invalidateAll);
    pimpl->_model.disconnect<EVT_CHANGE_VALUE>(*this, &FullDynamicEquationFunction::invalidateb);
    pimpl->_model.disconnect<EVT_CHANGE_VALUE>(*this, &FullDynamicEquationFunction::invalidateReducedProblemData);
    delete &getVariable();
}


void FullDynamicEquationFunction::takeIntoAccountGravity(bool useGrav)
{
    pimpl->_useGrav = useGrav;
}



/** Update the Jacobian matrix (\f$ \A \f$) in the function.
 *
 * Actually, it computes \f$ \A = \begin{bmatrix} \M  &  -S  & \J_c\tp \end{bmatrix} \f$ by updating \f$ \M \f$ and \f$ \J_c \f$.
 * \f$ S \f$ is constant and has been initialized in the constructor which calls #buildA()
 */
void FullDynamicEquationFunction::updateJacobian() const
{
    _jacobian.leftCols(_dim)                  =  pimpl->_model.getInertiaMatrix(); // we update the M part
    _jacobian.rightCols(pimpl->_fc.getSize()) =  pimpl->_model.getModelContacts().getJct(); // we update the Jc.T part
}

/** Update the vector (\f$ \b \f$) in the function.
 *
 * It computes \f$ \b = \n + \g \f$ .
 */
void FullDynamicEquationFunction::updateb() const
{
    if (pimpl->_useGrav)
        _b = pimpl->_model.getLinearTerms() + pimpl->_model.getNonLinearTerms() + pimpl->_model.getGravityTerms();
    else
        _b = pimpl->_model.getLinearTerms() + pimpl->_model.getNonLinearTerms();
}

/** Initialize the Jacobian matrix (\f$ \A \f$) in the function.
 *
 * It sets \f$ \A \f$ to 0 and computes \f$ S \f$ that is a constant matrix, So we obtain \f$ \A = \begin{bmatrix} 0  &  -S  & 0 \end{bmatrix} \f$.
 * \f$ \M \f$ and \f$ \J_c \f$ are updated in #updateJacobian() const .
 */
void FullDynamicEquationFunction::buildA()
{
    /* the dynamic equation is of the form:
     *
     * M ddq + n + l + g = S tau - Jc.T fc
     *
     * by gathering all the variables, it leads to:
     *
     *                | ddq |
     *                | tau |
     * [ M  -S   Jc.T]| fc  | + (n + l + g) = 0
     *
     * The dimension should not change, so we initialize all the jacobian and b here;
     * S is constant so we can set here;
     */
    _jacobian.setZero();

    // Here we update the S part;
    // If the robot has not a fixed root, then we need to delete the 6 first column in the actuatedDof diag, that's why this trick
    int index = pimpl->_model.hasFixedRoot() ? 0 : 6;
    _jacobian.block(0, _dim, _dim, pimpl->_tau.getSize()).setZero();
    _jacobian.block(index, _dim, pimpl->_tau.getSize(), pimpl->_tau.getSize()).diagonal() = -pimpl->_model.getActuatedDofs();
}


/** Create the function variable \f$ \x = \begin{bmatrix}  \ddq\tp & \torque\tp & \force_c\tp\end{bmatrix}\tp \f$ .
 *
 * \param model The Model that contains the dynamic variables:
 *
 *      - \f$ \ddq \f$ -> model.getAccelerationVariable()
 *      - \f$ \torque \f$ -> model.getJointTorqueVariable()
 *      - \f$ \force_c \f$ -> model.getModelContacts().getContactForcesVariable()
 */
Variable& FullDynamicEquationFunction::createDEVariable(const Model& model)
{
    static int cpt = 0;
    std::stringstream name;
    name << "dyn_eq" << cpt++;
    CompositeVariable* var = new CompositeVariable(name.str(), model.getAccelerationVariable());
    var->add(model.getJointTorqueVariable());                       //
    var->add(model.getModelContacts().getContactForcesVariable());  // [tau, fc] should be the same order as that of _chi in pimpl
    return *var;
}


/** Do when input size changes, before.
 *
 * By overloading this function, it allows linear function modification when input size changes.
 * It does nothing actually.
 */
void FullDynamicEquationFunction::doUpdateInputSizeBegin()
{
    //do nothing : this overload allows to resize
}

/** Do when input size changes, after.
 *
 * By overloading this function, it allows linear function modification when input size changes.
 * It does nothing actually.
 */
void FullDynamicEquationFunction::doUpdateInputSizeEnd()
{
    buildA();
}

void FullDynamicEquationFunction::invalidateReducedProblemData(int timestamp)
{
    pimpl->_reducedProblemDataAreUpToDate = false;
}


const Eigen::MatrixXd& FullDynamicEquationFunction::getInertiaMatrixInverseJchiT() const
{
    if (!pimpl->_reducedProblemDataAreUpToDate)
        pimpl->updateReduceProblemData( getJacobian(), getb() );
    return pimpl->_InertiaMatrixInverseJchiT;
}

const Eigen::VectorXd& FullDynamicEquationFunction::getInertiaMatrixInverseLinNonLinGrav() const
{
    if (!pimpl->_reducedProblemDataAreUpToDate)
        pimpl->updateReduceProblemData( getJacobian(), getb() );
    return pimpl->_InertiaMatrixInverseLinNonLinGrav;
}

Variable& FullDynamicEquationFunction::getActionVariable() const
{
    return pimpl->_chi;
}

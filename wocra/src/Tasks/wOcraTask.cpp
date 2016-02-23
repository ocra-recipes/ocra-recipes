/**
 * \file wOcraTask.cpp
 * \author Joseph Salini
 *
 * \brief Implement \b task class for wOcra controller. It inherits from the task class defined in the ocra framework.
 */

#include "wocra/Tasks/wOcraTask.h"

// OCRA INCLUDES
#include "ocra/control/Model.h"
#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/LinearizedCoulombFunction.h"
#include "ocra/optim/WeightedSquareDistanceFunction.h"

// WOCRA INCLUDES
#include "wocra/wOcraDebug.h"


namespace wocra
{


class VariableChiFunction : public ocra::LinearFunction
{
public:
    VariableChiFunction(ocra::Variable& x, int dimension)
    : NamedInstance("Variable Chi Linear Function")
    , ocra::AbilitySet(ocra::PARTIAL_X)
    , CoupledInputOutputSize(false)
    , LinearFunction(x, dimension)
    {

    }

    void doUpdateInputSizeBegin() {};

    void doUpdateInputSizeEnd() {};
};


struct wOcraTask::Pimpl
{
    const Model&                innerModel;
    wOcraSolver*                 solver;
    const wOcraDynamicFunction*  dynamicEquation;
    bool                        useReducedProblem;
    ocra::BaseVariable           fcVar;

//    double                weight;
    const Feature&        feature;

    bool taskHasBeenInitialized;

    LessThanZeroConstraintPtr<LinearizedCoulombFunction>    frictionConstraint; // if contact task

    bool contactForceConstraintHasBeenSavedInSolver;
    bool contactPointHasBeenSavedInModel;
    bool frictionConstraintIsRegisteredInConstraint;
    EqualZeroConstraintPtr<LinearFunction>  ContactForceConstraint;

    bool isRegisteredAsObjective;
    bool isRegisteredAsConstraint;

    TYPETASK                                innerTaskType;

    LinearFunction*                         innerObjectiveFunction;
    Objective<SquaredLinearFunction>*       innerTaskAsObjective;
    EqualZeroConstraintPtr<LinearFunction>  innerTaskAsConstraint;


    Pimpl(const std::string& name, const Model& m, const Feature& s)
        : innerModel(m)
        , solver(0x0)
        , dynamicEquation(0x0)
        , useReducedProblem(false)
        , fcVar(name+".var", s.getDimension())
//        , weight(1.)
        , feature(s)
        , taskHasBeenInitialized(false)
        , contactForceConstraintHasBeenSavedInSolver(false)
        , contactPointHasBeenSavedInModel(false)
        , frictionConstraintIsRegisteredInConstraint(false)
        , isRegisteredAsObjective(false)
        , isRegisteredAsConstraint(false)
        , innerTaskType(UNKNOWNTASK)
        , innerObjectiveFunction(NULL)
        , innerTaskAsObjective(NULL)
    {
        innerTaskAsConstraint.set(NULL);

        if(fcVar.getSize() == 3)
        {
//            std::cout<<"CAN BE A CONTACT POINT!!! register friction and contact constraints\n";
//            registerFrictionConstraint = true;
            frictionConstraint.set(  new LinearizedCoulombFunction(fcVar, 1., 6, 0.) );
            ContactForceConstraint.set( new LinearFunction( fcVar, Eigen::MatrixXd::Identity(3,3), VectorXd::Zero(3) ) );
        }
        else
        {
//            registerFrictionConstraint = false;
            frictionConstraint.set(NULL);
            ContactForceConstraint.set(NULL);
        }
    }

    ~Pimpl()
    {
        if (innerTaskAsObjective)
        {
            delete &innerTaskAsObjective->getFunction();
            delete innerTaskAsObjective;
        }
    }

    void setAsAccelerationTask()
    {

        int featn = feature.getDimension();
        if (useReducedProblem)
        {
            innerObjectiveFunction = new VariableChiFunction(dynamicEquation->getActionVariable(), featn);
        }
        else
        {
            innerObjectiveFunction = new LinearFunction (innerModel.getAccelerationVariable(), Eigen::MatrixXd::Zero(featn, innerModel.nbDofs()), Eigen::VectorXd::Zero(featn));
        }
        connectFunctionnWithObjectiveAndConstraint();
    }

    void setAsTorqueTask()
    {
        int featn = feature.getDimension();
        innerObjectiveFunction = new LinearFunction(innerModel.getJointTorqueVariable(), Eigen::MatrixXd::Zero(featn, innerModel.nbInternalDofs()), Eigen::VectorXd::Zero(featn));
        connectFunctionnWithObjectiveAndConstraint();
    }

    void setAsForceTask()
    {
        int featn = feature.getDimension();
        innerObjectiveFunction = new LinearFunction(fcVar, Eigen::MatrixXd::Identity(featn, featn), Eigen::VectorXd::Zero(featn));
        connectFunctionnWithObjectiveAndConstraint();
    }

    void connectFunctionnWithObjectiveAndConstraint()
    {
        innerTaskAsObjective = new Objective<SquaredLinearFunction>(new SquaredLinearFunction(innerObjectiveFunction));//, weight); // Here, it will manage the SquaredLinearFunction build on a pointer of the function.
        innerTaskAsConstraint.set(innerObjectiveFunction);      // As as ConstraintPtr, it will manage the new created function innerObjectiveFunction
    }
};


/** Initialize a new wOcra Task.
 *
 * \param taskName    The name of the task
 * \param innerModel  The ocra::Model on which we will update the dynamic parameters
 * \param feature     The task feature, meaning what we want to control
 * \param featureDes  The desired task feature, meaning the goal we want to reach with the \a feature
 */
wOcraTask::wOcraTask(const std::string& taskName, const Model& innerModel, const Feature& feature, const Feature& featureDes)
    : Task(taskName, innerModel, feature, featureDes)
    , pimpl(new Pimpl(taskName, innerModel, feature))
{

}

/** Initialize a new wOcra Task.
 *
 * \param taskName    The name of the task
 * \param innerModel  The ocra::Model on which we will update the dynamic parameters
 * \param feature     The task feature, meaning what we want to control
 */
wOcraTask::wOcraTask(const std::string& taskName, const Model& innerModel, const Feature& feature)
    : Task(taskName, innerModel, feature)
    , pimpl(new Pimpl(taskName, innerModel, feature))
{

}


wOcraTask::~wOcraTask()
{
}


void wOcraTask::connectToController(wOcraSolver& solver, const wOcraDynamicFunction& dynamicEquation, bool useReducedProblem)
{
    pimpl->solver            = &solver;
    pimpl->dynamicEquation   = &dynamicEquation;
    pimpl->useReducedProblem =  useReducedProblem;

    switch(pimpl->innerTaskType)
    {

        case(ACCELERATIONTASK):
        {
            pimpl->setAsAccelerationTask();

            break;
        }
        case(TORQUETASK):
        {
            pimpl->setAsTorqueTask();
            break;
        }
        case(FORCETASK):
        {
            pimpl->setAsForceTask();
            break;
        }
        case(COMMOMENTUMTASK):
        {
            pimpl->setAsAccelerationTask();
            break;
        }
        case(UNKNOWNTASK):
        {
            std::string errmsg = std::string("[wOcraTask::connectToController]: The task type of '") + getName() + std::string("' has not been set during creation.\nCall prior that 'initAsAccelerationTask', 'initAsTorqueTask' or 'initAsForceTask'\n"); //
            throw std::runtime_error(std::string(errmsg));
            break;
        }
        default:
        {
            throw std::runtime_error(std::string("[wOcraTask::connectToController]: Unhandle case of TYPETASK for task ")+getName() );
            break;
        }
    }
}


void wOcraTask::disconnectFromController()
{
    if (pimpl->isRegisteredAsObjective)
    {
        pimpl->solver->removeObjective(*pimpl->innerTaskAsObjective);
    }
    if (pimpl->isRegisteredAsConstraint)
    {
        pimpl->solver->removeConstraint(pimpl->innerTaskAsConstraint);
    }

    if (pimpl->frictionConstraintIsRegisteredInConstraint)
    {
        pimpl->solver->removeConstraint(pimpl->frictionConstraint);
    }
    if (pimpl->contactForceConstraintHasBeenSavedInSolver)
    {
        pimpl->solver->removeConstraint(pimpl->ContactForceConstraint);
    }

}



void wOcraTask::initAsAccelerationTask()
{
    pimpl->innerTaskType = ACCELERATIONTASK;
}


void wOcraTask::initAsTorqueTask()
{
    pimpl->innerTaskType = TORQUETASK;
}


void wOcraTask::initAsForceTask()
{
    pimpl->innerTaskType = FORCETASK;
}


void wOcraTask::initAsCoMMomentumTask()
{
    pimpl->innerTaskType = COMMOMENTUMTASK;
}

wOcraTask::TYPETASK wOcraTask::getTaskType() const
{
    return pimpl->innerTaskType;
}


const Eigen::VectorXd& wOcraTask::getComputedForce() const
{
    return pimpl->fcVar.getValue();
}





//---------------------------------------------------------------------------------------------------------------//
/** I don't really know, I suppose it is for a direct output.
 *
 * \param output  The vector instance where to write the output.
 *
 * This output is set to zero.
 */
void wOcraTask::doGetOutput(Eigen::VectorXd& output) const
{
    output = Eigen::VectorXd::Zero(getDimension());
}



void wOcraTask::addContactPointInModel()
{
    //THIS SHOULD BE DONE ONLY ONCE!!!
    if ( ! pimpl->contactPointHasBeenSavedInModel )
    {
        pimpl->innerModel.getModelContacts().addContactPoint(pimpl->fcVar, getFeature());
        pimpl->contactPointHasBeenSavedInModel = true;
    }

    if ( pimpl->contactForceConstraintHasBeenSavedInSolver )
    {
        pimpl->solver->removeConstraint(pimpl->ContactForceConstraint);
        pimpl->contactForceConstraintHasBeenSavedInSolver = false;
    }
}

void wOcraTask::removeContactPointInModel()
{
    //    if ( pimpl->contactPointHasBeenSavedInModel )
//    {
//        pimpl->model.getModelContacts().removeContactPoint(pimpl->fcVar);
//        pimpl->contactPointHasBeenSavedInModel = false;
//    }
    if ( ! pimpl->contactForceConstraintHasBeenSavedInSolver )
    {
        pimpl->solver->addConstraint(pimpl->ContactForceConstraint);
        pimpl->contactForceConstraintHasBeenSavedInSolver = true;
    }
}



/** Do task activation when it is a contact task.
 *
 * When this function is called, it adds a contact point in the model of contact contained in the ocra::Model instance,
 * and it adds in the solver an inequality constraint that represents the limitation of the contact force that must remain inside the cone of friction.
 */
void wOcraTask::doActivateContactMode()
{
    checkIfConnectedToController();

    addContactPointInModel();

    // add friction cone in constraint
    pimpl->solver->addConstraint(pimpl->frictionConstraint);
    pimpl->frictionConstraintIsRegisteredInConstraint = true;
}


/** Do task deactivation when it is a contact task.
 *
 * When this function is called, it removes the contact point in the model of contact,
 * and it removes from the solver the friction cone inequality constraint.
 */
void wOcraTask::doDeactivateContactMode()
{
    checkIfConnectedToController();

    removeContactPointInModel();

    // remove friction cone from constraint set
    pimpl->solver->removeConstraint(pimpl->frictionConstraint);
    pimpl->frictionConstraintIsRegisteredInConstraint = false;
}


/** For contact task, do the setting of the coefficient of friction.
 *
 * The cone of friction constraint is modified to represent a cone with this new coefficient of friction.
 */
void wOcraTask::doSetFrictionCoeff()
{
    pimpl->frictionConstraint.getFunction().setFrictionCoeff(getFrictionCoeff());
}

/** For contact task, do the setting of the friction margin.
 *
 * The cone of friction constraint is modified to represent a friction cone with this new margin.
 */
void wOcraTask::doSetMargin()
{
    pimpl->frictionConstraint.getFunction().setMargin(getMargin());
}









/** Do activation of task as an objective.
 *
 * It means that the task is not fully completed and a little error may occur.
 */
void wOcraTask::doActivateAsObjective()
{
    checkIfConnectedToController();
    pimpl->solver->addObjective(*pimpl->innerTaskAsObjective);
    pimpl->isRegisteredAsObjective = true;

    if (pimpl->innerTaskType == FORCETASK)
    {
        addContactPointInModel();
    }
}

/** Do deactivation of task as an objective.
 *
 * objective is no more considered.
 */
void wOcraTask::doDeactivateAsObjective()
{
    checkIfConnectedToController();
    pimpl->solver->removeObjective(*pimpl->innerTaskAsObjective);
    pimpl->isRegisteredAsObjective = false;

    if (pimpl->innerTaskType == FORCETASK)
    {
        removeContactPointInModel();
    }
}

/** Do activation of task as a constraint.
 *
 * It means that the task should be full completed and no error may occur.
 * Be aware that stong constraints may lead to system instability (very "sharp" solution that requires lot of energy).
 */
void wOcraTask::doActivateAsConstraint()
{
    checkIfConnectedToController();
    pimpl->solver->addConstraint(pimpl->innerTaskAsConstraint);
    pimpl->isRegisteredAsConstraint = true;

    if (pimpl->innerTaskType == FORCETASK)
    {
        addContactPointInModel();
    }
}

/** Do deactivation of task as a constraint.
 *
 * objective is no more considered.
 */
void wOcraTask::doDeactivateAsConstraint()
{
    checkIfConnectedToController();
    pimpl->solver->removeConstraint(pimpl->innerTaskAsConstraint);
    pimpl->isRegisteredAsConstraint = false;

    if (pimpl->innerTaskType == FORCETASK)
    {
        removeContactPointInModel();
    }
}



/** Do set the weight of the task.
 *
 * The weight in the objective function is modified.
 *
 */
void wOcraTask::doSetWeight()
{
    if (pimpl->innerTaskAsObjective)
    {
         pimpl->innerTaskAsObjective->getFunction().changeWeight(getWeight());
    }


}





//--------------------------------------------------------------------------------------------------------------------//
void wOcraTask::update()
{
    switch(pimpl->innerTaskType)
    {

        case(ACCELERATIONTASK):
        {
            doUpdateAccelerationTask();
            break;
        }
        case(TORQUETASK):
        {
            doUpdateTorqueTask();
            break;
        }
        case(FORCETASK):
        {
            doUpdateForceTask();
            break;
        }
        case(COMMOMENTUMTASK):
        {
            doUpdateCoMMomentumTask();
            break;
        }
        case(UNKNOWNTASK):
        {
            throw std::runtime_error(std::string("[wOcraTask::update]: The task type has not been set during creation."));
            break;
        }
        default:
        {
            throw std::runtime_error(std::string("[wOcraTask::update]: Unhandle case of TYPETASK."));
            break;
        }
    }
}


/** Update linear function of the task for the full formalism.
 *
 * It computes a desired acceleration \f$ \vec{a}^{des} = - \left( \vec{a}_{ref} + K_p (\vec{p}^{des} - vec{p}) +  K_d (\vec{v}^{des} - vec{v}) \right) \f$ .
 * Then The linear function is set as follows:
 *
 * - if it use the reduced problem:
 *
 * \f{align*}{
 *       \A &= J_{task}  .  \left(  \M^{-1} \J_{\tav}\tp  \right)
 *     & \b &= \vec{a}^{des} - \left(  J_{task} \M^{-1} ( \g - \n)  \right)
 * \f}
 *
 * see \ref sec_transform_formalism for more information.
 *
 * - else:
 *
 * \f{align*}{
 *       \A &= J_{task}
 *     & \b &= \vec{a}^{des}
 * \f}
 */
void wOcraTask::doUpdateAccelerationTask()
{
    const MatrixXd& J  = getJacobian();
    const MatrixXd& Kp = getStiffness();
    const MatrixXd& Kd = getDamping();

    const VectorXd  accDes = - ( getErrorDdot() + Kp * getError() + Kd * getErrorDot() );

    // std::cout << "\n----\ngetError() = " << getError() << std::endl;
    // std::cout << "getErrorDot() = " << getErrorDot() << std::endl;
    // std::cout << "getErrorDdot() = " << getErrorDdot() << std::endl;

    if (pimpl->useReducedProblem)
    {
        const Eigen::MatrixXd E2 =        - J * pimpl->dynamicEquation->getInertiaMatrixInverseJchiT();
        const Eigen::VectorXd f2 = accDes + J * pimpl->dynamicEquation->getInertiaMatrixInverseLinNonLinGrav();

        pimpl->innerObjectiveFunction->changeA(E2);
        pimpl->innerObjectiveFunction->changeb(f2);
    }
    else
    {
        pimpl->innerObjectiveFunction->changeA(J);
        pimpl->innerObjectiveFunction->changeb(accDes);
    }
}




void wOcraTask::doUpdateTorqueTask()
{
    const MatrixXd& J    =   getJacobian();
    const VectorXd  eff  = - getEffort();

    pimpl->innerObjectiveFunction->changeA(J);
    pimpl->innerObjectiveFunction->changeb(eff);
}


void wOcraTask::doUpdateForceTask()
{
    //innerObjectiveFunction->changeA(); //already set in initForceTask

    const VectorXd  eff  = - getEffort();

    pimpl->innerObjectiveFunction->changeb(eff);

}

void wOcraTask::doUpdateCoMMomentumTask()
{
    const MatrixXd& J  = pimpl->innerModel.getCoMAngularJacobian();
    const MatrixXd& Kd = getDamping();

    const VectorXd  accDes = - Kd * pimpl->innerModel.getCoMAngularVelocity();


    if (pimpl->useReducedProblem)
    {
        const Eigen::MatrixXd E2 =        - J * pimpl->dynamicEquation->getInertiaMatrixInverseJchiT();
        const Eigen::VectorXd f2 = accDes + J * pimpl->dynamicEquation->getInertiaMatrixInverseLinNonLinGrav();

        pimpl->innerObjectiveFunction->changeA(E2);
        pimpl->innerObjectiveFunction->changeb(f2);
    }
    else
    {
        pimpl->innerObjectiveFunction->changeA(J);
        pimpl->innerObjectiveFunction->changeb(accDes);
    }
}


void wOcraTask::checkIfConnectedToController() const
{
    if (!pimpl->solver)
    {
        std::string errmsg = std::string("[wOcraTask::doActivateAsObjective]: task '") + getName() + std::string("' not connected to any solver; Call prior that 'wOcraController::addTask' to connect to the solver inside the controller.\n"); //
        throw std::runtime_error(std::string(errmsg));
    }
}




} // end namespace wocra

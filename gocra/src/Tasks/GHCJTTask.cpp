/**
 * \file GHCJTTask.cpp
 * \author Mingxing Liu
 *
 * \brief Implement \b task class for GHCJT controller. It inherits from the task class defined in the ocra framework.
 */

#include "gocra/Tasks/GHCJTTask.h"

// OCRA INCLUDES
#include "ocra/control/Model.h"
#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/LinearFunction.h"
#include "ocra/optim/LinearizedCoulombFunction.h"
#include "ocra/optim/WeightedSquareDistanceFunction.h"
#include "ocra/optim/SumOfLinearFunctions.h"

// GOCRA INCLUDES
#include "gocra/gOcraDebug.h"


namespace gocra
{


struct GHCJTTask::Pimpl
{
    const std::string&          name;
    const Model&                innerModel;
    bool                        isFreeFloating;
    gOcraSolver*                 solver;
    SumOfLinearFunctions*  seConstraint;
    boost::shared_ptr<LinearFunction> taskSEConstraintFunction;
    bool                        useGSHC;
    ocra::BaseVariable           fcVar;


    double                weight;
    MatrixXd              alpha;
    MatrixXd              projector;
    MatrixXd              taskiProjector;
    const Feature&        feature;
    int                   indexBegin;
    int                   indexEnd;

    bool taskHasBeenInitialized;
    boost::shared_ptr<LinearizedCoulombFunction> frictionFunction;
    LessThanZeroConstraintPtr<LinearizedCoulombFunction>    frictionConstraint; // if contact task

    bool contactForceConstraintHasBeenSavedInSolver;
    bool contactPointHasBeenSavedInModel;
    bool frictionConstraintIsRegisteredInConstraint;


    bool isRegisteredAsObjective;
    bool isRegisteredAsConstraint;
    bool isPointContactTask;


    LinearFunction*                         innerObjectiveFunction;
    Objective<SquaredLinearFunction>*       innerTaskAsObjective;
    EqualZeroConstraintPtr<LinearFunction>  innerTaskAsConstraint;

    LinearFunction*                         regulationObjectiveFunction; //objective function for GHC
    Objective<SquaredLinearFunction>*       regulationObjective;

    Pimpl(const std::string& name, const Model& m, const Feature& s)
        : name(name)
        , innerModel(m)
        , isFreeFloating(m.nbDofs() - m.nbInternalDofs()>0)
        , solver(0x0)
        , useGSHC(false)
        , fcVar(name+".var", s.getDimension())
        , taskSEConstraintFunction()
        , frictionConstraint(NULL)
        , frictionFunction()
        , weight(1.)
        , alpha(MatrixXd::Zero(s.getDimension(),s.getDimension()))
        , projector(MatrixXd::Identity(m.nbDofs(),m.nbDofs()))
        , taskiProjector(MatrixXd::Identity(m.nbDofs(),m.nbDofs()))
        , feature(s)
        , taskHasBeenInitialized(false)
        , contactForceConstraintHasBeenSavedInSolver(false)
        , contactPointHasBeenSavedInModel(false)
        , frictionConstraintIsRegisteredInConstraint(false)
        , isRegisteredAsObjective(false)
        , isRegisteredAsConstraint(false)
        , isPointContactTask(false)
        , innerObjectiveFunction(NULL)
        , innerTaskAsObjective(NULL)
        , regulationObjectiveFunction(NULL)
        , regulationObjective(NULL)
    {
        innerTaskAsConstraint.set(NULL);

        if(fcVar.getSize() == 3)// candidate for contact mode
        {
            frictionFunction.reset( new LinearizedCoulombFunction(fcVar, 1., 6, 0.) );
            const int outDim = frictionFunction->getDimension();
            const int inDim = 3;
            frictionConstraint.set( new LinearFunction(fcVar, MatrixXd::Identity(outDim, inDim), VectorXd::Zero(outDim)) );
        }
        if (isFreeFloating)
        {
            taskSEConstraintFunction.reset(new LinearFunction( fcVar, MatrixXd::Zero(m.nbDofs() - m.nbInternalDofs(), fcVar.getSize()), VectorXd::Zero(m.nbDofs() - m.nbInternalDofs()) ));
        }

    }



    ~Pimpl()
    {
        if (innerTaskAsObjective)
        {
            delete &innerTaskAsObjective->getFunction();
            delete innerTaskAsObjective;
        }
        if (regulationObjective)
        {
            std::cout<<"----"<<std::endl;

            delete &regulationObjective->getFunction();
            delete regulationObjective;
        }
    }

    void connectFunctionnWithObjectiveAndConstraint()
    {
        innerTaskAsObjective = new Objective<SquaredLinearFunction>(new SquaredLinearFunction(innerObjectiveFunction), weight); // Here, it will manage the SquaredLinearFunction build on a pointer of the function.
        innerTaskAsConstraint.set(innerObjectiveFunction);      // As as ConstraintPtr, it will manage the new created function innerObjectiveFunction
        regulationObjective = new Objective<SquaredLinearFunction>(new SquaredLinearFunction(regulationObjectiveFunction), weight*0.01);
    }
};


/** Initialize a new GHCJT Task.
 *
 * \param name The name of the task
 * \param model The ocr::Model on which we will update the dynamic parameters
 * \param feature The task feature, meaning what we want to control
 * \param featureDes The desired task feature, meaning the goal we want to reach with the \a feature
 */
GHCJTTask::GHCJTTask(const std::string& taskName, const Model& innerModel, const Feature& feature, const Feature& featureDes)
    : Task(taskName, innerModel, feature, featureDes)
    , pimpl(new Pimpl(taskName, innerModel, feature))
{

}

/** Initialize a new GHCJT Task.
 *
 * \param name The name of the task
 * \param model The ocra::Model on which we will update the dynamic parameters
 * \param feature The task feature, meaning what we want to control
 */
GHCJTTask::GHCJTTask(const std::string& taskName, const Model& innerModel, const Feature& feature)
    : Task(taskName, innerModel, feature)
    , pimpl(new Pimpl(taskName, innerModel, feature))
{

}


GHCJTTask::~GHCJTTask()
{

}


void GHCJTTask::connectToController(gOcraSolver& solver, SumOfLinearFunctions& seConstraint)
{
    pimpl->solver            = &solver;
    pimpl->seConstraint = &seConstraint;


    int featn = pimpl->feature.getDimension();
    pimpl->innerObjectiveFunction = new LinearFunction(pimpl->fcVar, Eigen::MatrixXd::Identity(featn, featn), Eigen::VectorXd::Zero(featn));
    pimpl->regulationObjectiveFunction = new LinearFunction(pimpl->fcVar, Eigen::MatrixXd::Identity(featn, featn),Eigen::VectorXd::Zero(featn));

    pimpl->connectFunctionnWithObjectiveAndConstraint();
}


void GHCJTTask::disconnectFromController()
{
    if (pimpl->isRegisteredAsObjective)
    {
        pimpl->solver->removeObjective(*pimpl->innerTaskAsObjective);

        if (!isPointContactTask())
        {
            pimpl->solver->removeObjective(*pimpl->regulationObjective);
        }
    }
    if (pimpl->isRegisteredAsConstraint)
    {
        pimpl->solver->removeConstraint(pimpl->innerTaskAsConstraint);
    }

    if (pimpl->frictionConstraintIsRegisteredInConstraint)
    {
        pimpl->solver->removeConstraint(pimpl->frictionConstraint.getConstraint());
    }


}


const Eigen::VectorXd& GHCJTTask::getComputedForce() const
{
    return pimpl->fcVar.getValue();
}


void GHCJTTask::doGetOutput(Eigen::VectorXd& output) const
{
    output = pimpl->fcVar.getValue();
}



void GHCJTTask::addContactPointInModel()
{
    //THIS SHOULD BE DONE ONLY ONCE!!!
    if ( ! pimpl->contactPointHasBeenSavedInModel )
    {
        pimpl->innerModel.getModelContacts().addContactPoint(pimpl->fcVar, getFeature());
        pimpl->contactPointHasBeenSavedInModel = true;
    }


}

void GHCJTTask::removeContactPointInModel()
{
    //    if ( pimpl->contactPointHasBeenSavedInModel )
//    {
//        pimpl->model.getModelContacts().removeContactPoint(pimpl->fcVar);
//        pimpl->contactPointHasBeenSavedInModel = false;
//    }
//    if ( ! pimpl->contactForceConstraintHasBeenSavedInSolver )
//    {
//        pimpl->solver->addConstraint(pimpl->ContactForceConstraint);
//        pimpl->contactForceConstraintHasBeenSavedInSolver = true;
//    }
}



/** Do task activation when it is a contact task.
 *
 * When this function is called, it adds a contact point in the model of contact contained in the xde Model instance,
 * and it adds in the solver an inequality constraint that represents the limitation of the contact force that must remain inside the cone of friction.
 */
void GHCJTTask::doActivateContactMode()
{
    checkIfConnectedToController();

    addContactPointInModel();

    // add friction cone in constraint
    pimpl->solver->addConstraint(pimpl->frictionConstraint.getConstraint());

    pimpl->frictionConstraintIsRegisteredInConstraint = true;
}


/** Do task deactivation when it is a contact task.
 *
 * When this function is called, it removes the contact point in the model of contact,
 * and it removes from the solver the friction cone inequality constraint.
 */
void GHCJTTask::doDeactivateContactMode()
{
    checkIfConnectedToController();

    removeContactPointInModel();

    // remove friction cone from constraint set
    pimpl->solver->removeConstraint(pimpl->frictionConstraint.getConstraint());
    pimpl->frictionConstraintIsRegisteredInConstraint = false;
}


/** For contact task, do the setting of the coefficient of friction.
 *
 * The cone of friction constraint is modified to represent a cone with this new coefficient of friction.
 */
void GHCJTTask::doSetFrictionCoeff()
{
    pimpl->frictionFunction->setFrictionCoeff(getFrictionCoeff());
}

/** For contact task, do the setting of the friction margin.
 *
 * The cone of friction constraint is modified to represent a friction cone with this new margin.
 */
void GHCJTTask::doSetMargin()
{
    pimpl->frictionFunction->setMargin(getMargin());
}


/** Do activation of task as an objective.
 *
 * It means that the task is not fully completed and a little error may occur.
 */
void GHCJTTask::doActivateAsObjective()
{
    checkIfConnectedToController();
    pimpl->solver->addObjective(*pimpl->innerTaskAsObjective);
    pimpl->solver->addObjective(*pimpl->regulationObjective);
    if (pimpl->isFreeFloating)
        pimpl->seConstraint->addFunction(*pimpl->taskSEConstraintFunction);

    pimpl->isRegisteredAsObjective = true;



}

/** Do deactivation of task as an objective.
 *
 * objective is no more considered.
 */
void GHCJTTask::doDeactivateAsObjective()
{
    checkIfConnectedToController();
    pimpl->solver->removeObjective(*pimpl->innerTaskAsObjective);
    pimpl->solver->removeObjective(*pimpl->regulationObjective);
    if (pimpl->isFreeFloating)
        pimpl->seConstraint->removeFunction(*pimpl->taskSEConstraintFunction);
    pimpl->isRegisteredAsObjective = false;



}

/** Do activation of task as a constraint.
 *
 * It means that the task should be full completed and no error may occur.
 * Be aware that stong constraints may lead to system instability (very "sharp" solution that requires lot of energy).
 */
void GHCJTTask::doActivateAsConstraint()
{
    checkIfConnectedToController();
    pimpl->solver->addConstraint(pimpl->innerTaskAsConstraint);
    pimpl->solver->addObjective(*pimpl->regulationObjective);
    if (pimpl->isFreeFloating)
        pimpl->seConstraint->addFunction(*pimpl->taskSEConstraintFunction);

    pimpl->isRegisteredAsConstraint = true;


}

/** Do deactivation of task as a constraint.
 *
 * objective is no more considered.
 */
void GHCJTTask::doDeactivateAsConstraint()
{
    checkIfConnectedToController();
    pimpl->solver->removeObjective(*pimpl->regulationObjective);
    pimpl->solver->removeConstraint(pimpl->innerTaskAsConstraint);
    if (pimpl->isFreeFloating)
        pimpl->seConstraint->removeFunction(*pimpl->taskSEConstraintFunction);
    pimpl->isRegisteredAsConstraint = false;


}



/** Do set the weight of the task.
 *
 * The weight in the objective function is modified.
 *
 * \todo the \b getWeight() method of the task returns a matrix, but the \b setWeight() method of the objective requires a double value.
 * I don't really know how to cope with this problem. For now the first value of the matrix is used.
 */
void GHCJTTask::doSetWeight()
{

    if (pimpl->innerTaskAsObjective)
    {

        pimpl->innerTaskAsObjective->getFunction().changeWeight(getWeight());
    }

}


//--------------------------------------------------------------------------------------------------------------------//
void GHCJTTask::update()
{

    if(isBodyContactConstraint())
      return;

    if (pimpl->isFreeFloating)
    {
        const MatrixXd Jt = getJacobian().transpose();
        const MatrixXd PJt = getProjector()*Jt;
        if (isPointContactTask())
            pimpl->taskSEConstraintFunction->changeA( Jt.topRows(pimpl->taskSEConstraintFunction->getDimension()) );
        else
            pimpl->taskSEConstraintFunction->changeA( PJt.topRows(pimpl->taskSEConstraintFunction->getDimension()) );
    }

    const MatrixXd& Kp = getStiffness();
    const MatrixXd& Kd = getDamping();
//    const VectorXd e = pimpl->feature.computeError(*getFeatureDes());
//    const VectorXd edot = pimpl->feature.computeErrorDot(*getFeatureDes());
//    const VectorXd& e = getFeatureDes() ? getFeature().computeError(*getFeatureDes()) : getFeature().computeError();
//    const VectorXd& edot = getFeatureDes() ? getFeature().computeErrorDot(*getFeatureDes()) : getFeature().computeErrorDot();
//    const VectorXd& fd = getFeatureDes() ? getFeature().computeEffort(*getFeatureDes()) : getFeature().computeEffort();
//    const VectorXd f = -Kp * e - Kd * edot - fd;

    const VectorXd f =  -Kp * getError() - Kd * getErrorDot() - getEffort();

//    std::cout<<"f="<<f.transpose()<<std::endl;
//    const VectorXd f = getEffort();

//    pimpl->innerObjectiveFunction->changeReference(f);
    pimpl->innerObjectiveFunction->changeb(f);

    if(isPointContactTask())
    {
        setWeight(0.01);
        const MatrixXd& A = pimpl->frictionFunction->getA();
        const VectorXd b = pimpl->frictionFunction->getb() + A * getFrictionConstraintOffset();
        pimpl->frictionConstraint.getFunction().changeA(A);
        pimpl->frictionConstraint.getFunction().changeb(b);
    }
}



void GHCJTTask::checkIfConnectedToController() const
{
    if (!pimpl->solver)
    {
        std::string errmsg = std::string("[GHCJTTask::doActivateAsObjective]: task '") + getName() + std::string("' not connected to any solver; Call prior that 'GHCJTController::addTask' to connect to the solver inside the controller.\n"); //
        throw std::runtime_error(std::string(errmsg));
    }
}


const MatrixXd& GHCJTTask::getPriority() const
{
  return pimpl->alpha;

}

void GHCJTTask::setPriority(Eigen::MatrixXd& alpha)
{
    pimpl->alpha = alpha;

}

int GHCJTTask::getTaskDimension() const
{
    return pimpl->feature.getDimension();
}

const std::string& GHCJTTask::getTaskName() const
{
    return pimpl->name;
}

void GHCJTTask::setIndexBegin(int index)
{
    pimpl->indexBegin = index;
}

int GHCJTTask::getIndexBegin() const
{
    return pimpl->indexBegin;
}

void GHCJTTask::setIndexEnd(int index)
{
    pimpl->indexEnd = index;
}

int GHCJTTask::getIndexEnd() const
{
    return pimpl->indexEnd;
}

void GHCJTTask::setProjector(MatrixXd& proj)
{
    pimpl->projector = proj;
}

const MatrixXd& GHCJTTask::getProjector() const
{
    return pimpl->projector;
}

void GHCJTTask::setTaskiProjector(MatrixXd& proj)
{
    pimpl->taskiProjector = proj;
}

const MatrixXd& GHCJTTask::getTaskiProjector() const
{
    return pimpl->taskiProjector;
}

LinearFunction* GHCJTTask::getInnerObjectiveFunction() const
{
    return pimpl->innerObjectiveFunction;
}

const Variable& GHCJTTask::getVariable() const
{
  return pimpl->fcVar;
}

} // end namespace gocra

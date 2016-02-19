/**

 * \file OneLevelTask.h

 * \author Joseph Salini

 *

 * \brief Define \b task class for wOcra controller. It inherits from the task class defined in the xde framework.
 *
 * The class defined here is an abstract classes. It is overriden to fit full problem or reduced problem.

 */

#ifndef _ONE_LEVEL_TASK_H_

#define _ONE_LEVEL_TASK_H_


// OCRA INCLUDES
#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/SquaredLinearFunction.h"
#include "ocra/optim/WeightedSquareDistanceFunction.h"
#include "ocra/control/Task.h"
#include "ocra/control/Feature.h"
#include "ocra/control/ControlFrame.h"
#include "ocra/optim/OneLevelSolver.h"
#include "ocra/control/ControlConstraint.h"








namespace ocra

{

/** \addtogroup task
 * \{
 */

/** \brief A generic abstract task for the wOcra controller.
 *
 * The main difference with the Task class defined in the xde framework is the addition of a level parameter. Hence, a hierarchical set of tasks can be solved.
 *
 * \note This level information may have been added to the controller or the solver, not directly added to the task, and the Task class of the xde framework may have been used instead of this new class.
 *       But I think the level is the same concept as the weight (an importance), so I add it direcly in the task class, like the weight.
 *       Furthermore, writting this class helps me to better understand the xde framework.
 */

class OneLevelTask: public Task

{

public:

    // enum TYPETASK { UNKNOWNTASK, ACCELERATIONTASK, TORQUETASK, FORCETASK, COMMOMENTUMTASK };


    OneLevelTask(const std::string& taskName, const Model& innerModel, const Feature& feature, const Feature& featureDes);

    OneLevelTask(const std::string& taskName, const Model& innerModel, const Feature& feature);
    virtual ~OneLevelTask();

    // void initAsAccelerationTask();
    // void initAsTorqueTask();
    // void initAsForceTask();
    // void initAsCoMMomentumTask();
    //
    // TYPETASK getTaskType() const;

    const Eigen::VectorXd& getComputedForce() const;

    void disconnectFromController();


    //------------------------ friendship ------------------------//
    // friend class wOcraController;    //Only the wOcraController should know about the following functions
    void connectToController(OneLevelSolver& _solver, const FullDynamicEquationFunction& dynamicEquation, bool useReducedProblem);
    // void disconnectFromController();
    // void update();



protected:
    void addContactPointInModel();
    void removeContactPointInModel();

    void doUpdateAccelerationTask();
    void doUpdateTorqueTask();
    void doUpdateForceTask();
    void doUpdateCoMMomentumTask();

    void checkIfConnectedToController() const;

  protected:
    virtual void doActivateContactMode();
    virtual void doActivateAsConstraint();
    virtual void doDeactivateAsObjective();
    virtual void doDeactivateAsConstraint();
    virtual void doDeactivateContactMode();
    virtual void doSetFrictionCoeff();
    virtual void doSetMargin();
    virtual void doSetWeight();
    virtual void doActivateAsObjective();
    virtual void doGetOutput(Eigen::VectorXd& output) const;
    virtual void doUpdate();

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;

  };
}

#endif
/** \} */ // end group task

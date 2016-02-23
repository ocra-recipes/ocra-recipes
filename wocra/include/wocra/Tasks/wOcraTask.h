/**

 * \file wOcraTask.h

 * \author Joseph Salini

 *

 * \brief Define \b task class for wOcra controller. It inherits from the task class defined in the xde framework.
 *
 * The class defined here is an abstract classes. It is overriden to fit full problem or reduced problem.

 */

#ifndef __wOcraTASK_H__

#define __wOcraTASK_H__


// OCRA INCLUDES
#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/SquaredLinearFunction.h"
#include "ocra/optim/WeightedSquareDistanceFunction.h"



#include "ocra/control/Task.h"

#include "ocra/control/Feature.h"
#include "ocra/control/ControlFrame.h"


#include "wocra/Solvers/wOcraSolver.h"

#include "wocra/Constraints/wOcraConstraint.h"





using namespace ocra;



namespace wocra

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

class wOcraTask: public Task

{

public:

    enum TYPETASK { UNKNOWNTASK, ACCELERATIONTASK, TORQUETASK, FORCETASK, COMMOMENTUMTASK };


    wOcraTask(const std::string& taskName, const Model& innerModel, const Feature& feature, const Feature& featureDes);

    wOcraTask(const std::string& taskName, const Model& innerModel, const Feature& feature);
    virtual ~wOcraTask();

    void initAsAccelerationTask();
    void initAsTorqueTask();
    void initAsForceTask();
    void initAsCoMMomentumTask();

    TYPETASK getTaskType() const;

    const Eigen::VectorXd& getComputedForce() const;

    void disconnectFromController();


    //------------------------ friendship ------------------------//
protected:
    friend class wOcraController;    //Only the wOcraController should know about the following functions
    void connectToController(wOcraSolver& _solver, const wOcraDynamicFunction& dynamicEquation, bool useReducedProblem);
    // void disconnectFromController();
    void update();



protected:
    virtual void doGetOutput(Eigen::VectorXd& output) const;

    void addContactPointInModel();
    void removeContactPointInModel();

    virtual void doActivateContactMode();

    virtual void doDeactivateContactMode();
    virtual void doSetFrictionCoeff();

    virtual void doSetMargin();

    virtual void doSetWeight();

    virtual void doActivateAsObjective();
    virtual void doDeactivateAsObjective();

    virtual void doActivateAsConstraint();

    virtual void doDeactivateAsConstraint();

    void doUpdateAccelerationTask();
    void doUpdateTorqueTask();
    void doUpdateForceTask();
    void doUpdateCoMMomentumTask();

    void checkIfConnectedToController() const;

private:

    struct Pimpl;

    boost::shared_ptr<Pimpl> pimpl;

};

/** \} */ // end group task

}



#endif

/**
 * \file GHCJTTask.h
 * \author Mingxing Liu
 *
 * \brief Define \b task class for GHCJT controller. It inherits from the task class defined in the ocra framework.
 *
 */

#ifndef __GHCJTTask_H__
#define __GHCJTTask_H__

// OCRA INCLUDES
#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/SquaredLinearFunction.h"
#include "ocra/optim/WeightedSquareDistanceFunction.h"


#include "ocra/control/Task.h"
#include "ocra/control/Feature.h"
#include "ocra/control/ControlFrame.h"

#include "gocra/Solvers/gOcraSolver.h"
//#include "gocra/Constraints/gOcraConstraint.h"


using namespace ocra;
namespace ocra
{
  class SumOfLinearFunctions;
}
namespace gocra
{

/** \addtogroup task
 * \{
 */

/** \brief A generic abstract task for the GHCJT controller.
 *
 */
class GHCJTTask: public Task
{
public:


    GHCJTTask(const std::string& taskName, const Model& innerModel, const Feature& feature, const Feature& featureDes);
    GHCJTTask(const std::string& taskName, const Model& innerModel, const Feature& feature);
    virtual ~GHCJTTask();


    const Eigen::VectorXd& getComputedForce() const;

    const Eigen::MatrixXd& getPriority() const;
    void   setPriority(Eigen::MatrixXd& alpha);
    int getTaskDimension() const;
    const std::string& getTaskName() const;
    void setIndexBegin(int index);
    int getIndexBegin() const;
    void setIndexEnd(int index);
    int getIndexEnd() const;
    void setProjector(Eigen::MatrixXd& proj);
    void setTaskiProjector(Eigen::MatrixXd& proj);
    const Eigen::MatrixXd& getProjector() const;
    const Eigen::MatrixXd& getTaskiProjector() const;
    LinearFunction* getInnerObjectiveFunction() const;
    const Variable& getVariable() const;


    //------------------------ friendship ------------------------//
protected:
    friend class GHCJTController;    //Only the GHCJTController should know about the following functions
    void connectToController(gOcraSolver& _solver, SumOfLinearFunctions& seConstraint);
    void disconnectFromController();
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

    void checkIfConnectedToController() const;

private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;

};


/** \} */ // end group task

}

#endif

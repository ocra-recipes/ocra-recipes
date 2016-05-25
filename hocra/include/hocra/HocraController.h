#ifndef __HOCRACTRL_H__

#define __HOCRACTRL_H__


#include <string>
#include <vector>
#include <iostream>
#include <memory>

#include <ocra/optim/CascadeQPSolver.h>
#include <ocra/control/Model.h>
#include <ocra/control/Controller.h>
#include <ocra/control/Tasks/Task.h>

namespace hocra

{
using namespace ocra;

/** \addtogroup core

 * \{

 */



/** \brief Hocra Controller based on LQP solver for the ocra framework.

 *

 */

class HocraController: public Controller

{

public:

    HocraController(const std::string& ctrlName, Model::Ptr innerModel, OneLevelSolver::Ptr innerSolver, bool useReducedProblem);
    virtual void doAddContactSet(const ContactSet& contacts);
    virtual void doAddTask(Task::Ptr task);
    virtual void doComputeOutput(VectorXd& tau);
    Task::Ptr doCreateContactTask(const std::string& name, const ocra::PointContactFeature& feature, double mu, double margin) const;
    Task::Ptr doCreateTask(const std::string& name, const ocra::Feature& feature) const;
    Task::Ptr doCreateTask(const std::string& name, const ocra::Feature& feature, const ocra::Feature& featureDes) const;
    virtual ~HocraController(){};

private:
    CascadeQPSolver::Ptr cascadeQPSolver;
    Model::Ptr innerModel;
};



/** \} */ // end group core



} //end namespace hocra







#endif

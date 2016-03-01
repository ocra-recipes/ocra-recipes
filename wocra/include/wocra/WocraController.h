/**
 * \file WocraController.h
 * \author Joseph Salini
 *
 * \brief Define the LQP-based controller developped during my PhD thesis with ocra framework.
 *
 * The LQP-base controller is explained in my thesis available here: http://hal.archives-ouvertes.fr/tel-00710013/ .
 * Based on a dynamic model of the robot, we have to define tasks and constraints to properly control the system.
 */

#ifndef __WOCRACTRL_H__
#define __WOCRACTRL_H__

#include <string>#include <vector>
#include <iostream>
#include <memory>
// OCRA INCLUDES#include "ocra/control/Controller.h"
#include "ocra/control/Model.h"
#include "ocra/optim/OneLevelSolver.h"
// WOCRA INCLUDES
#include "ocra/control/Tasks/OneLevelTask.h"#include "ocra/control/ControlConstraint.h"

using namespace ocra;


namespace wocra
{

/** \addtogroup core
 * \{
 */

/** \brief Wocra Controller based on LQP solver for the ocra framework.
 *
 */
class WocraController: public Controller
{
public:

    WocraController(const std::string& ctrlName, Model& innerModel, ocra::OneLevelSolver& innerSolver, bool useReducedProblem);
    virtual ~WocraController();

    //getter
    Model&                  getModel();
    ocra::OneLevelSolver&   getSolver();
    bool                    isUsingReducedProblem();

    //setter
    void setVariableMinimizationWeights(double w_ddq, double w_tau, double w_fc);
    void takeIntoAccountGravity(bool useGrav);

    void writePerformanceInStream(std::ostream& myOstream, bool addCommaAtEnd) const;
    std::string getPerformances() const;

    void addConstraint(ocra::LinearConstraint& constraint) const;
    void removeConstraint(ocra::LinearConstraint& constraint) const;
    void addConstraint(ocra::ControlConstraint& constraint) const;
    void removeConstraint(ocra::ControlConstraint& constraint) const;

    // WocraTask& createWocraTask(const std::string& name, const Feature& feature, const Feature& featureDes) const;
    // WocraTask& createWocraTask(const std::string& name, const Feature& feature) const;
    // WocraTask& createWocraContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const;

protected:
    virtual void doComputeOutput(Eigen::VectorXd& tau);
    virtual void doAddTask(std::shared_ptr<Task> task);
    virtual void doAddContactSet(const ContactSet& contacts);

protected: // factory
    virtual std::shared_ptr<Task> doCreateTask(const std::string& name, const Feature& feature, const Feature& featureDes) const;
    virtual std::shared_ptr<Task> doCreateTask(const std::string& name, const Feature& feature) const;
    virtual std::shared_ptr<Task> doCreateContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const;


private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;

};

/** \} */ // end group core

} //end namespace wocra



#endif
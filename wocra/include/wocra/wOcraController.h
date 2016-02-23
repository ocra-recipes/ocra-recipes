/**
 * \file wOcraController.h
 * \author Joseph Salini
 *
 * \brief Define the LQP-based controller developped during my PhD thesis with ocra framework.
 *
 * The LQP-base controller is explained in my thesis available here: http://hal.archives-ouvertes.fr/tel-00710013/ .
 * Based on a dynamic model of the robot, we have to define tasks and constraints to properly control the system.
 */

#ifndef __wOcraCTRL_H__
#define __wOcraCTRL_H__

#include <string>
#include <vector>
#include <iostream>

// OCRA INCLUDES
#include "ocra/control/Controller.h"
#include "ocra/control/Model.h"


// WOCRA INCLUDES
#include "wocra/Tasks/wOcraTask.h"
#include "wocra/Solvers/wOcraSolver.h"
#include "wocra/Constraints/wOcraConstraint.h"

using namespace ocra;


namespace wocra
{

/** \addtogroup core
 * \{
 */

/** \brief wOcra Controller based on LQP solver for the ocra framework.
 *
 */
class wOcraController: public Controller
{
public:

    wOcraController(const std::string& ctrlName, Model& innerModel, wOcraSolver& innerSolver, bool useReducedProblem);
    virtual ~wOcraController();

    //getter
    Model&      getModel();
    wOcraSolver& getSolver();
    bool        isUsingReducedProblem();

    //setter
    void setVariableMinimizationWeights(double w_ddq, double w_tau, double w_fc);
    void takeIntoAccountGravity(bool useGrav);

    void writePerformanceInStream(std::ostream& myOstream, bool addCommaAtEnd) const;
    std::string getPerformances() const;

    void addConstraint(ocra::LinearConstraint& constraint) const;
    void removeConstraint(ocra::LinearConstraint& constraint) const;
    void addConstraint(wOcraConstraint& constraint) const;
    void removeConstraint(wOcraConstraint& constraint) const;

    wOcraTask& createwOcraTask(const std::string& name, const Feature& feature, const Feature& featureDes) const;
    wOcraTask& createwOcraTask(const std::string& name, const Feature& feature) const;
    wOcraTask& createwOcraContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const;

protected:
    virtual void doComputeOutput(Eigen::VectorXd& tau);
    virtual void doAddTask(Task& task);
    virtual void doAddContactSet(const ContactSet& contacts);

protected: // factory
    virtual Task* doCreateTask(const std::string& name, const Feature& feature, const Feature& featureDes) const;
    virtual Task* doCreateTask(const std::string& name, const Feature& feature) const;
    virtual Task* doCreateContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const;


private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;

};

/** \} */ // end group core

} //end namespace wocra



#endif

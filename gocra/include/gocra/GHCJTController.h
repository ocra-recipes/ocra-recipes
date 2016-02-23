/**
 * \file GHCJTController.h
 * \author Mingxing Liu
 *
 * \brief Define the Generalized Hierarchical Controller based on Jacobian transpose (GHCJT) in quasi-static case.
 *
 * The GHCJT controller is based on the generalized projector explained in the paper available here: http://link.springer.com/article/10.1007/s10514-015-9436-1 .
 */

#ifndef __GHCJTCTRL_H__
#define __GHCJTCTRL_H__

#include <string>
#include <vector>
#include <iostream>

// OCRA INCLUDES
#include "ocra/control/Controller.h"
#include "ocra/control/Model.h"
#include "ocra/optim/OneLevelSolver.h"


// GOCRA INCLUDES
#include "gocra/Tasks/GHCJTTask.h"
//#include "gocra/Constraints/gOcraConstraint.h"
//#include "gocra/Constraints/GHCAccelerationConstraint.h"
using namespace ocra;


namespace gocra
{

/** \addtogroup core
 * \{
 */

/** \brief gOcra Controller based on LQP solver for the ocra framework.
 *
 */
class GHCJTController: public Controller
{
public:

    GHCJTController(const std::string& ctrlName, Model& innerModel, ocra::OneLevelSolver& innerSolver, bool useGrav);
    virtual ~GHCJTController();

    //getter
    Model&      getModel();
    ocra::OneLevelSolver& getSolver();

    //setter
    void takeIntoAccountGravity(bool useGrav);

    void writePerformanceInStream(std::ostream& myOstream, bool addCommaAtEnd) const;
    std::string getPerformances() const;

    void addConstraint(ocra::LinearConstraint& constraint) const;
    void removeConstraint(ocra::LinearConstraint& constraint) const;
    //void addConstraint(gOcraConstraint& constraint) const;
    //void removeConstraint(gOcraConstraint& constraint) const;

    GHCJTTask& createGHCJTTask(const std::string& name, const Feature& feature, const Feature& featureDes) const;
    GHCJTTask& createGHCJTTask(const std::string& name, const Feature& feature) const;
    GHCJTTask& createGHCJTContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const;

    void setActiveTaskVector();
    void doUpdateAugmentedJacobian();
    void doUpdateProjector();
    std::vector< GHCJTTask* >& getActiveTask();
    int getNbActiveTask() const;
    int getTotalActiveTaskDimensions() const;

    void initPriorityMatrix();

    std::pair<Eigen::VectorXd, Eigen::MatrixXd> sortRows(const Eigen::MatrixXd &C, const Eigen::MatrixXd &J);
    void computeProjector(const Eigen::MatrixXd &C, const Eigen::MatrixXd &J, Eigen::MatrixXd& projector);
    void computeTaskiProjector(const Eigen::MatrixXd &J, Eigen::MatrixXd& projector);
    void setTaskProjectors(Eigen::MatrixXd& param);






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

} //end namespace gocra



#endif

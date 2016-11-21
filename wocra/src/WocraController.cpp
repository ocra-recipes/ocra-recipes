/**
 * \file WocraController.cpp
 * \author Joseph Salini
 *
 * \brief Implement the LQP-based controller developped during my PhD thesis with ocra framework.
 */

#include "wocra/WocraController.h"

#include <iostream>

#include "ocra/optim/QuadraticFunction.h"
#include "wocra/Performances.h"

/** \brief Contains all the abstract & concrete classes for robotic control with optimization, based on the ocra framework.
 *
 */
namespace wocra
{
    
    struct WocraController::Pimpl
    {
        std::shared_ptr<Model>          innerModel;
        std::shared_ptr<OneLevelSolver> innerSolver;
        bool                            reducedProblem;
        
        
        // EQUALITY CONSTRAINT OF THE DYNAMIC EQUATION
        ocra::EqualZeroConstraintPtr< ocra::FullDynamicEquationFunction >      dynamicEquation;
        
        // MINIMIZATION TASK FOR WHOLE VARIABLE MINIMIZATION
        ocra::QuadraticFunction*     minDdqFunction;
        ocra::QuadraticFunction*     minTauFunction;
        ocra::FcQuadraticFunction*        minFcFunction;
        
        ObjectivePtr<ocra::QuadraticFunction>     minDdqObjective;
        ObjectivePtr<ocra::QuadraticFunction>     minTauObjective;
        ObjectivePtr<ocra::QuadraticFunction>     minFcObjective;
        
        
        // PERFORMANCE RECORDERS
        PerformanceRecorder updateTasksRecorder;
        
        PerformanceRecorder solveProblemRecorder;
        
        
        Pimpl(std::shared_ptr<Model> m, std::shared_ptr<OneLevelSolver> s, bool useReducedProblem)
        
        : innerModel(m)
        
        , innerSolver(s)
        
        , reducedProblem(useReducedProblem)
        , dynamicEquation( new ocra::FullDynamicEquationFunction(*m) )
        
        , minDdqFunction(  new ocra::QuadraticFunction(m->getAccelerationVariable(), Eigen::MatrixXd::Identity(m->nbDofs(), m->nbDofs()), Eigen::VectorXd::Zero(m->nbDofs()), 0) )
        
        , minTauFunction(  new ocra::QuadraticFunction(m->getJointTorqueVariable(), Eigen::MatrixXd::Identity(m->getJointTorqueVariable().getSize(), m->getJointTorqueVariable().getSize()), Eigen::VectorXd::Zero(m->getJointTorqueVariable().getSize()), 0) )
        
        , minFcFunction(   new FcQuadraticFunction(m->getModelContacts().getContactForcesVariable()) )
        
        {
            minDdqObjective.set(minDdqFunction);
            minTauObjective.set(minTauFunction);
            minFcObjective.set(minFcFunction);
            
        }
        
        ~Pimpl()
        {
        }
        
    };
    
    
    
    // ############# WocraController Class ###############################################################
    
    
    WocraController::WocraController(const std::string& ctrlName, std::shared_ptr<Model> innerModel, std::shared_ptr<OneLevelSolver> innerSolver, bool useReducedProblem)
    
    : Controller(ctrlName, *innerModel)
    
    , pimpl( new Pimpl(innerModel, innerSolver, useReducedProblem) )
    
    {
        
        if (!pimpl->reducedProblem)
            
        {
            
            pimpl->innerSolver->addConstraint(pimpl->dynamicEquation.getConstraint());
            pimpl->innerSolver->addObjective(pimpl->minDdqObjective);
            pimpl->innerSolver->addObjective(pimpl->minTauObjective);
            pimpl->innerSolver->addObjective(pimpl->minFcObjective);
            
            // double predictionHorizon = 0.2;
            // jointLimitConstraint = std::make_shared<ocra::JointLimitConstraint>(*pimpl->innerModel);//, predictionHorizon);
            // addConstraint(*jointLimitConstraint);
            // Eigen::VectorXd torqueLims  = Eigen::VectorXd::Constant(pimpl->innerModel->nbInternalDofs(), 2.0);
            // torqueLimitConstraint = std::make_shared<ocra::TorqueLimitConstraint>(*pimpl->innerModel, torqueLims);//, predictionHorizon);
            // addConstraint(*torqueLimitConstraint);
            
            
        }
        else
        {
            pimpl->innerSolver->addObjective(pimpl->minTauObjective);
            pimpl->innerSolver->addObjective(pimpl->minFcObjective);
        }
        setVariableMinimizationWeights(1e-7, 1e-8, 1e-9);
        takeIntoAccountGravity(true);
        
    };
    
    WocraController::~WocraController()
    
    {
        const std::map<std::string, std::shared_ptr<Task>>& taskMap = getTasks();
        for (std::map<std::string, std::shared_ptr<Task>>::const_iterator it = taskMap.begin(); it != taskMap.end(); ++it)
        {
            if(it->second)
                it->second->disconnectFromController();
        }
        
        
        if (!pimpl->reducedProblem)
            
        {
            
            pimpl->innerSolver->removeConstraint(pimpl->dynamicEquation.getConstraint());
            pimpl->innerSolver->removeObjective(pimpl->minDdqObjective);
            pimpl->innerSolver->removeObjective(pimpl->minTauObjective);
            pimpl->innerSolver->removeObjective(pimpl->minFcObjective);
            
        }
        else
        {
            pimpl->innerSolver->removeObjective(pimpl->minTauObjective);
            pimpl->innerSolver->removeObjective(pimpl->minFcObjective);
        }
        
    };
    
    std::shared_ptr<Model> WocraController::getModel()
    
    {
        
        return pimpl->innerModel;
        
    }
    
    std::shared_ptr<OneLevelSolver> WocraController::getSolver()
    
    {
        
        return pimpl->innerSolver;
        
    }
    
    bool WocraController::isUsingReducedProblem()
    
    {
        
        return pimpl->reducedProblem;
        
    }
    
    void WocraController::setVariableMinimizationWeights(double w_ddq, double w_tau, double w_fc)
    {
        pimpl->minDdqObjective.getObjective().setWeight(w_ddq);
        pimpl->minTauObjective.getObjective().setWeight(w_tau);
        pimpl->minFcObjective.getObjective().setWeight(w_fc);
    }
    
    void WocraController::takeIntoAccountGravity(bool useGrav)
    {
        pimpl->dynamicEquation.getFunction().takeIntoAccountGravity(useGrav);
    }
    
    void WocraController::addConstraint(ocra::LinearConstraint& constraint) const
    {
        pimpl->innerSolver->addConstraint(constraint);
    }
    
    void WocraController::removeConstraint(ocra::LinearConstraint& constraint) const
    {
        pimpl->innerSolver->removeConstraint(constraint);
    }
    
    void WocraController::addConstraint(ocra::ControlConstraint& constraint) const
    {
        constraint.connectToController(pimpl->dynamicEquation, pimpl->reducedProblem);
        pimpl->innerSolver->addConstraint(constraint.getConstraint());
    }
    
    void WocraController::removeConstraint(ocra::ControlConstraint& constraint) const
    {
        pimpl->innerSolver->removeConstraint(constraint.getConstraint());
        constraint.disconnectFromController();
    }
    
    void WocraController::doAddTask(std::shared_ptr<Task> task)
    {
        try {
            task->connectToController(pimpl->innerSolver, pimpl->dynamicEquation, pimpl->reducedProblem);
        }
        catch(const std::exception & e) {
            std::cerr << e.what() ;
            throw std::runtime_error("[WocraController::doAddTask] cannot add task to controller (wrong type)");
        }
    };
    
    void WocraController::doAddContactSet(const ContactSet& contacts)
    
    {
        
        throw std::runtime_error("[WocraController::doAddTask] not implemented");
        
    };
    
    
    
    
    //
    //
    // /** Create an WocraTask.
    //  *
    //  * \return an WocraTask instance that is a dynamic_cast of the task returned by ocra::Controller::createTask(const std::string& name, Feature::Ptr feature, Feature::Ptr featureDes) const
    //  */
    // WocraTask& WocraController::createWocraTask(const std::string& name, Feature::Ptr feature, Feature::Ptr featureDes) const
    // {
    //     return dynamic_cast<WocraTask&>(createTask(name, feature, featureDes));
    // }
    //
    // /** Create an WocraTask.
    //  *
    //  * \return an WocraTask instance that is a dynamic_cast of the task returned by ocra::Controller::createTask(const std::string& name, Feature::Ptr feature) const
    //  */
    // WocraTask& WocraController::createWocraTask(const std::string& name, Feature::Ptr feature) const
    // {
    //     return dynamic_cast<WocraTask&>(createTask(name, feature));
    // }
    //
    // /** Create an WocraContactTask.
    //  *
    //  * \return an WocraTask instance that is a dynamic_cast of the task returned by ocra::Controller::createContactTask(const std::string& name, PointContactFeature::Ptr feature, double mu, double margin) const
    //  */
    // WocraTask& WocraController::createWocraContactTask(const std::string& name, PointContactFeature::Ptr feature, double mu, double margin) const
    // {
    //     return dynamic_cast<WocraTask&>(createContactTask(name, feature, mu, margin));
    // }
    
    
    
    std::shared_ptr<Task> WocraController::doCreateTask(const std::string& name, Feature::Ptr feature, Feature::Ptr featureDes) const
    {
        return std::make_shared<ocra::Task>(name, pimpl->innerModel, feature, featureDes);
    };
    
    std::shared_ptr<Task> WocraController::doCreateTask(const std::string& name, Feature::Ptr feature) const
    {
        return std::make_shared<ocra::Task>(name, pimpl->innerModel, feature);
    };
    
    std::shared_ptr<Task> WocraController::doCreateContactTask(const std::string& name, PointContactFeature::Ptr feature, double mu, double margin) const
    {
        return std::make_shared<ocra::Task>(name, pimpl->innerModel, feature);
    };
    
    /**
     *  Computes the output of the controller.
     *
     *  Here, the controller solves the optimization problem depending on the tasks and constraints, and the result is set in the variable of the problem, either \f$ x = [ \ddq \; \tau \; \force_c ] \f$ or \f$ x = [ \tau \; \force_c ] \f$. The torque variable \f$ \tau \f$ is finally applied to the robot.
     *
     *  @param tau The torque variable, which is the output of our problem.
     */
    void WocraController::doComputeOutput(Eigen::VectorXd& tau)
    
    {
        
        pimpl->updateTasksRecorder.initializeTime();
        
        const std::vector<std::shared_ptr<Task>>& tasks = getActiveTasks();
        
        for(auto task : tasks)
        {
            task->update();
        }
        
        pimpl->updateTasksRecorder.saveRelativeTime();
        
        
        pimpl->solveProblemRecorder.initializeTime();
        if(!pimpl->innerSolver->solve().info)
        {
            tau = pimpl->innerModel->getJointTorqueVariable().getValue();
        }
        
        else
        {
            setErrorMessage("solver error");
            setErrorFlag(OTHER | CRITICAL_ERROR);
        }
        pimpl->solveProblemRecorder.saveRelativeTime();
        
    }
    
    void WocraController::writePerformanceInStream(std::ostream& outstream, bool addCommaAtEnd) const
    
    {
        
        pimpl->updateTasksRecorder.writeInStream("controller_update_tasks", outstream, true);
        
        pimpl->solveProblemRecorder.writeInStream("controller_solve_problem", outstream, true);
        
        pimpl->innerSolver->writePerformanceInStream(outstream, addCommaAtEnd);
        
    }
    
    std::string WocraController::getPerformances() const
    {
        std::ostringstream osstream;
        // Write it in a json style
        osstream <<"{\n";
        writePerformanceInStream(osstream, false);
        osstream <<"}";
        return osstream.str();
    }
    
    
    
    
    
    
} // namespace wocra

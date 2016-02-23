/**
 * \file wOcraController.cpp
 * \author Joseph Salini
 *
 * \brief Implement the LQP-based controller developped during my PhD thesis with ocra framework.
 */

#include "wocra/wOcraController.h"

#include <iostream>

#include "ocra/optim/QuadraticFunction.h"

#include "wocra/Solvers/wOcraSolver.h"
#include "wocra/Tasks/wOcraTask.h"

#include "wocra/Performances.h"


/** \brief Contains all the abstract & concrete classes for robotic control with optimization, based on the ocra framework.
 *
 */
namespace wocra
{


/** \brief QuadraticFunction dedicated to the minimization of the force of contact (fc) variable.
 *
 */
class FcQuadraticFunction : public ocra::QuadraticFunction
{
public:

    FcQuadraticFunction(ocra::Variable& x)
    : NamedInstance("Variable Fc Quadratic Function")
    , ocra::AbilitySet(ocra::PARTIAL_X, ocra::PARTIAL_XX)
    , CoupledInputOutputSize(false)
    , QuadraticFunction(x)
    {

    }

    virtual ~FcQuadraticFunction() {};

    void doUpdateInputSizeBegin() {};

    void updateHessian() const
    {
        IFunction<ocra::PARTIAL_XX>::_val[0]->setIdentity(x.getSize(), x.getSize());
    }

    void updateq() const
    {
        _q[0]->setZero(x.getSize());
    }

    void updater() const
    {
        _r[0] = 0;
    }

};


struct wOcraController::Pimpl
{
    Model&       innerModel;
    wOcraSolver&  innerSolver;
    bool         reducedProblem;

    // EQUALITY CONSTRAINT OF THE DYNAMIC EQUATION
    ocra::EqualZeroConstraintPtr< wOcraDynamicFunction >      dynamicEquation;

    // MINIMIZATION TASK FOR WHOLE VARIABLE MINIMIZATION
    ocra::QuadraticFunction*     minDdqFunction;
    ocra::QuadraticFunction*     minTauFunction;
    FcQuadraticFunction*        minFcFunction;

    ObjectivePtr<ocra::QuadraticFunction>     minDdqObjective;
    ObjectivePtr<ocra::QuadraticFunction>     minTauObjective;
    ObjectivePtr<ocra::QuadraticFunction>     minFcObjective;

    std::vector< wOcraTask* >                    createdTask;

    // PERFORMANCE RECORDERS
    PerformanceRecorder updateTasksRecorder;
    PerformanceRecorder solveProblemRecorder;

    Pimpl(Model& m, wOcraSolver&  s, bool useReducedProblem)
        : innerModel(m)
        , innerSolver(s)
        , reducedProblem(useReducedProblem)
        , dynamicEquation( new wOcraDynamicFunction(m) )
        , minDdqFunction(  new ocra::QuadraticFunction(m.getAccelerationVariable(), Eigen::MatrixXd::Identity(m.nbDofs(), m.nbDofs()), Eigen::VectorXd::Zero(m.nbDofs()), 0) )
        , minTauFunction(  new ocra::QuadraticFunction(m.getJointTorqueVariable(), Eigen::MatrixXd::Identity(m.getJointTorqueVariable().getSize(),m.getJointTorqueVariable().getSize()), Eigen::VectorXd::Zero(m.getJointTorqueVariable().getSize()), 0) )
        , minFcFunction(   new FcQuadraticFunction(m.getModelContacts().getContactForcesVariable()) )
    {
        minDdqObjective.set(minDdqFunction);
        minTauObjective.set(minTauFunction);
        minFcObjective.set(minFcFunction);
    }

    ~Pimpl()
    {
    }
};



/** Initialize wOcra controller.
 *
 * \param ctrlName           The name of the controller
 * \param innerModel         The internal model of the robot one wants to control
 * \param innerSolver        The internal solver one wants to use to make the quadratic optimization
 * \param useReducedProblem  Tell if the redundant problem is considered (unknown variable is \f$ [ \ddq \; \torque \; \force_c ] \f$),
 *                           or is the reduced problem (non-redundant) is considred (unknown variable is \f$ [ \torque \; \force_c ] \f$)
 */
wOcraController::wOcraController(const std::string& ctrlName, Model& innerModel, wOcraSolver& innerSolver, bool useReducedProblem)
    : Controller(ctrlName, innerModel)
    , pimpl( new Pimpl(innerModel, innerSolver, useReducedProblem) )
{
    if (!pimpl->reducedProblem)
    {
        pimpl->innerSolver.addConstraint(pimpl->dynamicEquation.getConstraint());
        pimpl->innerSolver.addObjective(pimpl->minDdqObjective);
        pimpl->innerSolver.addObjective(pimpl->minTauObjective);
        pimpl->innerSolver.addObjective(pimpl->minFcObjective);
    }
    else
    {
        pimpl->innerSolver.addObjective(pimpl->minTauObjective);
        pimpl->innerSolver.addObjective(pimpl->minFcObjective);
    }
    setVariableMinimizationWeights(1e-7, 1e-8, 1e-9);
    takeIntoAccountGravity(true);
};

/** Destructor of wOcra controller.
 *
 * It disconnects all the tasks connected to the controller, then it disconnects the inner objectives
 * that minimize the problem variables, and finally it disconnects the dynamic equation constraint (if needed).
 */
wOcraController::~wOcraController()
{
    for (int i=0; i<pimpl->createdTask.size(); ++i)
    {
        pimpl->createdTask[i]->disconnectFromController();
        delete pimpl->createdTask[i];
    }

    if (!pimpl->reducedProblem)
    {
        pimpl->innerSolver.removeConstraint(pimpl->dynamicEquation.getConstraint());
        pimpl->innerSolver.removeObjective(pimpl->minDdqObjective);
        pimpl->innerSolver.removeObjective(pimpl->minTauObjective);
        pimpl->innerSolver.removeObjective(pimpl->minFcObjective);
    }
    else
    {
        pimpl->innerSolver.removeObjective(pimpl->minTauObjective);
        pimpl->innerSolver.removeObjective(pimpl->minFcObjective);
    }
};


/** return the inner model
 * \return the inner model used to construct this controller instance
 */
Model& wOcraController::getModel()
{
    return pimpl->innerModel;
}

/** return the inner solver
 * \return the inner solver used to construct this controller instance
 */
wOcraSolver& wOcraController::getSolver()
{
    return pimpl->innerSolver;
}


/**
 * \return \c true if variable of reduced problem (\f$ [ \torque \; \force_c ] \f$) is considered, or \c false if it uses the variable of the full one (\f$ [ \ddq \; \torque \; \force_c ] \f$).
 */
bool wOcraController::isUsingReducedProblem()
{
    return pimpl->reducedProblem;
}


/** Set weights for the objectives that minimize the norm of the problem variables: \f$ [ \ddq \; \torque \; \force_c ] \f$
 *
 * \param w_ddq weight for the minimization of \f$ \ddq \f$
 * \param w_tau weight for the minimization of \f$ \torque \f$
 * \param w_fc  weight for the minimization of \f$ \force_c \f$
 */
void wOcraController::setVariableMinimizationWeights(double w_ddq, double w_tau, double w_fc)
{
    pimpl->minDdqObjective.getObjective().setWeight(w_ddq);
    pimpl->minTauObjective.getObjective().setWeight(w_tau);
    pimpl->minFcObjective.getObjective().setWeight(w_fc);
}


/** Whether to take into account gavity in the dynamic equation of motion
 *
 * \param useGrav \c true if gravity is enable, \c false otherwise
 */
void wOcraController::takeIntoAccountGravity(bool useGrav)
{
    pimpl->dynamicEquation.getFunction().takeIntoAccountGravity(useGrav);
}


/** add a Linear constraint that is equivalent for the full & reduced problems.
 *
 */
void wOcraController::addConstraint(ocra::LinearConstraint& constraint) const
{
    pimpl->innerSolver.addConstraint(constraint);
}

/** remove a Linear constraint that is equivalent for the full & reduced problems.
 *
 */
void wOcraController::removeConstraint(ocra::LinearConstraint& constraint) const
{
    pimpl->innerSolver.removeConstraint(constraint);
}

/** add a Linear constraint that has different expressions, depending on the problem type.
 *
 * In this case, the constraint needs to be connected to the controller to get the matrices for the problem reduction.
 * See wocra::wOcraConstraint for more info.
 */
void wOcraController::addConstraint(wOcraConstraint& constraint) const
{
    constraint.connectToController(pimpl->dynamicEquation, pimpl->reducedProblem);
    pimpl->innerSolver.addConstraint(constraint.getConstraint());
}

/** add a Linear constraint that has different expressions, depending on the problem type.
 *
 * In this case, the constraint needs to be disconnected from the controller.
 * See wocra::wOcraConstraint for more info.
 */
void wOcraController::removeConstraint(wOcraConstraint& constraint) const
{
    pimpl->innerSolver.removeConstraint(constraint.getConstraint());
    constraint.disconnectFromController();
}



/** Internal implementation inside the addTask method.
 *
 * \param task The task to add in the controller
 */
void wOcraController::doAddTask(Task& task)
{
    try {
        wOcraTask& ctask = dynamic_cast<wOcraTask&>(task);
        ctask.connectToController(pimpl->innerSolver, pimpl->dynamicEquation, pimpl->reducedProblem);
    }
    catch(const std::exception & e) {
        std::cerr << e.what() ;
        throw std::runtime_error("[wOcraController::doAddTask] cannot add task to controller (wrong type)");
    }
};

/** Internal implementation inside the addContactSet method.
 *
 * \param contacts The contact set to add in the controller
 * \todo this method has not been implemented!!!
 */
void wOcraController::doAddContactSet(const ContactSet& contacts)
{
    throw std::runtime_error("[wOcraController::doAddTask] not implemented");
};



/** Create an wOcraTask.
 *
 * \return an wOcraTask instance that is a dynamic_cast of the task returned by ocra::Controller::createTask(const std::string& name, const Feature& feature, const Feature& featureDes) const
 */
wOcraTask& wOcraController::createwOcraTask(const std::string& name, const Feature& feature, const Feature& featureDes) const
{
    return dynamic_cast<wOcraTask&>(createTask(name, feature, featureDes));
}

/** Create an wOcraTask.
 *
 * \return an wOcraTask instance that is a dynamic_cast of the task returned by ocra::Controller::createTask(const std::string& name, const Feature& feature) const
 */
wOcraTask& wOcraController::createwOcraTask(const std::string& name, const Feature& feature) const
{
    return dynamic_cast<wOcraTask&>(createTask(name, feature));
}

/** Create an wOcraContactTask.
 *
 * \return an wOcraTask instance that is a dynamic_cast of the task returned by ocra::Controller::createContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const
 */
wOcraTask& wOcraController::createwOcraContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const
{
    return dynamic_cast<wOcraTask&>(createContactTask(name, feature, mu, margin));
}





/** Internal implementation inside the createTask method.
 *
 * \param name The task name, a unique identifier
 * \param feature The part of the robot one wants to control (full state, frame, CoM,...)
 * \param featureDes The desired state one wants to reach, depends on the \a feature argument
 * \return The pointer to the new created task
 *
 * This method is called by the higher level methods #createwOcraTask(const std::string&, const Feature&, const Feature&) const
 * and is the concrete implementation required by the ocra Controller class.
 */
Task* wOcraController::doCreateTask(const std::string& name, const Feature& feature, const Feature& featureDes) const
{
    wOcraTask* nTask = new wOcraTask(name, pimpl->innerModel, feature, featureDes);
    pimpl->createdTask.push_back(nTask);
    return nTask;
};

/** Internal implementation inside the createTask method.
 *
 * \param name The task name, a unique identifier
 * \param feature The part of the robot one wants to control (full state, frame, CoM,...)
 * \return The pointer to the new created task
 *
 * This method is called by the higher level methods #createwOcraTask(const std::string&, const Feature&) const
 * and is the concrete implementation required by the ocra Controller class.
 */
Task* wOcraController::doCreateTask(const std::string& name, const Feature& feature) const
{
    wOcraTask* nTask = new wOcraTask(name, pimpl->innerModel, feature);
    pimpl->createdTask.push_back(nTask);
    return nTask;
};

/** Internal implementation inside the createContactTask method.
 *
 * \param name     The task name, a unique identifier
 * \param feature  The contact point feature of the robot one wants to control
 * \param mu       The friction cone coefficient \f$ \mu \f$ such as \f$ \Vert \force_t \Vert < \mu \force_n \f$
 * \param margin   The margin inside the friction cone
 * \return The pointer to the new created contact task
 *
 * This method is called by the higher level methods #createwOcraContactTask(const std::string&, const PointContactFeature&, , double, double) const
 * and is the concrete implementation required by the ocra::Controller class.
 */
Task* wOcraController::doCreateContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const
{
    wOcraTask* nTask = new wOcraTask(name, pimpl->innerModel, feature);
    pimpl->createdTask.push_back(nTask);
    return nTask;
};




//MOST IMPORTANT FUNCTION: COMPUTE OUTPUT TORQUE
/** Compute the output of the controller.
 *
 * \param tau The torque variable, which is the output of our problem
 *
 * Here, the controller solves the optimization problem depending on the tasks and constraints, and the result is set in the variable of the problem,
 * either \f$ x = [ \ddq \; \tau \; \force_c ] \f$ or \f$ x = [ \tau \; \force_c ] \f$. The torque variable \f$ \tau \f$ is finally applied to the robot.
 */
void wOcraController::doComputeOutput(Eigen::VectorXd& tau)
{
    pimpl->updateTasksRecorder.initializeTime();
    const std::vector<Task*>& tasks = getActiveTasks();
    for(int i=0; i< tasks.size(); i++)
    {
        wOcraTask* cTask = static_cast<wOcraTask*>(tasks[i]); // addTask throws if this cast is not possible
        cTask->update();
    }
    pimpl->updateTasksRecorder.saveRelativeTime();

    pimpl->solveProblemRecorder.initializeTime();
    if(!pimpl->innerSolver.solve().info)
    {
        tau = pimpl->innerModel.getJointTorqueVariable().getValue();
    }
    else
    {
        setErrorMessage("solver error");
        setErrorFlag(OTHER | CRITICAL_ERROR);
    }
    pimpl->solveProblemRecorder.saveRelativeTime();
}

/** Write information about controller performances in a string stream.
 *
 * \param outstream the output stream where to write the performances information
 * \param addCommaAtEnd If true, add a comma at the end of the stream. If false, it means that this is the end of the json file, nothing will be added after that.
 *
 * See wocra::wOcraController::getPerformances() to know more. Here it saves:
 *
 *  - controller_update_tasks
 *  - controller_solve_problem
 *  - solver_prepare
 *  - solver_solve
 */
void wOcraController::writePerformanceInStream(std::ostream& outstream, bool addCommaAtEnd) const
{
    pimpl->updateTasksRecorder.writeInStream("controller_update_tasks", outstream, true);
    pimpl->solveProblemRecorder.writeInStream("controller_solve_problem", outstream, true);
    pimpl->innerSolver.writePerformanceInStream(outstream, addCommaAtEnd);
}



/** Get information about performances through a string.
 *
 * Information are saved in a JSON way (http://www.json.org/). It returns a dictionnary of the form:
 *
 * \code
 * {
 *    "performance_info_1": [0.01, 0.02, 0.01, ... , 0.03, 0.01],
 *    ...
 *    "performance_info_n": [0.01, 0.02, 0.01, ... , 0.03, 0.01]
 * }
 * \endcode
 *
 * where performance_info are:
 *
 *  - controller_update_tasks
 *  - controller_solve_problem
 *  - solver_prepare
 *  - solver_solve
 *
 * See wocra::wOcraController::writePerformanceInStream(std::ostream&, bool) const and wocra::wOcraSolver::writePerformanceInStream(std::ostream&, bool).
 */
std::string wOcraController::getPerformances() const
{
    std::ostringstream osstream;
    // Write it in a json style
    osstream <<"{\n";
    writePerformanceInStream(osstream, false);
    osstream <<"}";
    return osstream.str();
}




} // namespace wocra

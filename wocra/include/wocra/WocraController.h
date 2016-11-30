/**
 * @file WocraController.h
 * @author Joseph Salini
 *
 * @brief Define the LQP-based controller developped during my PhD thesis with ocra framework.
 *
 * The LQP-base controller is explained in my thesis \cite{salini2012Thesis}.
 * Based on a dynamic model of the robot, we have to define tasks and constraints to properly control the system.
 */



#ifndef __WOCRACTRL_H__

#define __WOCRACTRL_H__


#include <string>
#include <vector>
#include <iostream>
#include <memory>

// OCRA INCLUDES
#include "ocra/control/Controller.h"
#include "ocra/control/Model.h"
#include "ocra/optim/OneLevelSolver.h"

// WOCRA INCLUDES
#include "ocra/control/Task.h"
#include "ocra/control/ControlConstraint.h"
#include "ocra/control/JointLimitConstraint.h"
#include "ocra/control/TorqueLimitConstraint.h"


using namespace ocra;




namespace wocra

{
    
    
    
    /** \addtogroup core
     
     * \{
     
     */
    
    
    
    /** @brief Wocra Controller based on LQP solver for the ocra framework.
     *
     *  @detail This controller is a generic optimization-based whole-body dynamic one, which can deal with tasks transitions. The multi-task management and their sequencing is done with a weighting strategy. A mechanical system presents different control issues regarding its limitations and physical constraints which are passed to the optimization problem as equality and inequality constraints. The controller will correct the error between actual and desired states by, for example, minimizing an error resulting from the quadratic norm of their difference. This error will be minimized through a Linear Quadratic Program (LQP), which is composed by a quadratic cost function with linear constraints. 
     *
     *   The following are types of internal and external physical constraints of the system, describing a robot and its interaction with the environment:
     *
     *    - Equations of motion (with interactions) (See ocra::FullDynamicEquationFunction).
     *    - Actuation limits.
     *      - Torque limits.
     *      - Acceleration limits.
     *      - Velocity limits.
     *      - Joint limits.
     *    - Obstacle avoidance. 
     *    - Interaction with the environment:
     *      - Bilateral closure.
     *      - Unilateral frictional contact.
     *
     *   All the previous constraints are linear except for the unilateral frictional contacts, which represent the constraint of the contact forces lying in the Coulomb friction cone and is nonlinear, but at least quadratic. To deal with these kinds of constraints the WOCRA controller will rely on the LQP formulation, since it is simpler and more suitable for the real-time dynamic control of humanoid robots.
     *   
     *   Wocra will address the low-level control of the robot through the concept of tasks, i.e. the control of one or more DoF of the system towards objectives which are generally the minimization of tracking errors. Tasks are composed of two main elements, the subset of the controlled DoF and their desired goals (For more details on tasks, see \Tasks()). An example of a task function is the norm of an error.
     *
     *   Different tasks can be mixed in wocra to achieve a desired high-level motion. These tasks will be subject to the aforementioned equality and inequality constraints, which motivate the use of an LQP, which solves the following problem:
     *
     *      \f{align*}{
     *          \underset{\x}{\text{min}}  &\;  \frac{1}{2} \left( \x\tp P \x + \q\tp\x + r \right) \\
     *                               s.t. &:\;  \G\x \leq h \\
     *                                     &\;  A\x = \b
     *      \f}
     *
     *   Where \f$ x \f$ is the vector to optimize, \f$ P \f$, \f$ q \f$ and \f$ r \f$ represent the quadratic cost function, \f$ G \f$, \f$ h \f$ define the inequality constraints and \f$ A \f$, \f$ b \f$ define the equality constraints, which are the concatenation of the different constraints previously mentioned. The controller thus builds these matrices according to the tasks and the constraints acting on the robot and, when the solution is found, extracts the input torque vector \f$ \tau \f$ to actuate the system. \f$ x \f$ can be replaced by \f$ \left[ \ddq \tp \; f_c \tp \; \tau \tp \right] \tp \f$ which we also denote by the **dynamic variable** \f$ \X = [\ddq\tp \av] \f$, where \f$ \av \f$ is further referred to as the **action variable**.
     * 
     *   But the story does not end here. In particular, Wocra will use a weighting strategy which associates each task with a coefficient that sets its importance with respect to the others (a task with a higher weights gets a higher priority). The first point is to set these coefficients as real values. As a consequence, priorities are not strict and all tasks are achieved according to the trade-off defined by the weights. Given a set of \f$ n \f$ tasks and their related weights \f$ (T_i(\q, \dq, \X), \omega_i) \; i \in [1...n] \f$ one solves their weighted sum subject to the concatenation of the constraints. The task \f$ T_0 \f$ is dedicated to the minimization of the whole optimization variable. This is required when the control problem has many solutions, in order to ensure the uniqueness and more specifically to provide a solution that minimizes the input torque vector \f$ \tau \f$. \f$ T_0 \f$ has a very small weight with respect to the others \f$ 0 < \omega_0 << 1 \f$ to limit the induced error. This program is solved only one time per call.
     *
     *   The algorithm consists then in finding  \f$\X_i^*\f$, the solution of the problem:
     *  
     *   \f{align*}{
                \underset{\X}{\text{min}} &\; \frac{1}{2} \left( \sum_{i=1}^{n} (\omega_i^2 T_i (\q, \dq, \X)) + \omega_0^2 T_0(\q, \dq, \X) \right) \\
                   s.t.: &\; G \X \leq \h \\
                         &\; \A \X = \b
         \f}
     *
     *   @cite salini2012Thesis.
     *
     *   @note Put proper link to the Tasks class and document it well based on Salini's thesis.
     *
     */
    
    class WocraController: public Controller
    
    {
        
    public:
        
        
        /** Initializes Wocra controller.
         * 
         * During the instantiation of this class a WocraController::Pimpl object is created. This object, in turn, will specify the constraints (robot dynamics) and objectives (\f$ \ddq, \torque, \force_c \f$) for the solver, as well as its relative minimization weights and will take into account gravity.
         *
         * @param ctrlName           The name of the controller
         * @param innerModel         The internal model of the robot one wants to control
         * @param innerSolver        The internal solver one wants to use to make the quadratic optimization
         * @param useReducedProblem  Tell if the redundant problem is considered (unknown variable is \f$ [ \ddq, \torque, \force_c ] \f$), or is the reduced problem (non-redundant) is considred (unknown variable is \f$ [ \torque, \force_c ] \f$)
         */
        WocraController(const std::string& ctrlName, std::shared_ptr<Model> innerModel, std::shared_ptr<OneLevelSolver> innerSolver, bool useReducedProblem);
        
        /**
         *  @brief Destructor of Wocra controller.
         *  @details It disconnects all the tasks connected to the controller, then it disconnects the inner objectives
         *  that minimize the problem variables, and finally it disconnects the dynamic equation constraint (if needed).
         *
         */
        virtual ~WocraController();
        
        
        /**
         *  Returns the inner model.
         *
         *  @return inner model used to construct this controller instance
         */
        std::shared_ptr<Model>                  getModel();
        
        /**
         *  Returns the inner solver.
         *
         *  @return Inner solver used to construct this controller instance.
         */
        std::shared_ptr<OneLevelSolver>   getSolver();
        
        /**
         *   @return \c true if variable of reduced problem (\f$ [ \torque \; \force_c ] \f$) is considered, or \c false if it uses the variable of the full one (\f$ [ \ddq \; \torque \; \force_c ] \f$).
         
         */
        bool                    isUsingReducedProblem();
        
        /**
         *  Sets weights for the objectives that minimize the norm of the problem variables: \f$ [ \ddq \; \torque \; \force_c ] \f$.
         *
         *  @param w_ddq weight for the minimization of \f$ \ddq \f$.
         *  @param w_tau weight for the minimization of \f$ \torque \f$.
         *  @param w_fc  weight for the minimization of \f$ \force_c \f$.
         */
        void setVariableMinimizationWeights(double w_ddq, double w_tau, double w_fc);
        
        /**
         *  Whether to take into account gavity in the dynamic equation of motion.
         *
         *  @param useGrav \c true if gravity is enable, \c false otherwise.
         */
        void takeIntoAccountGravity(bool useGrav);
        
        /** Writes information about controller performances in a string stream.
         *
         * @param outstream the output stream where to write the performances information
         * @param addCommaAtEnd If true, add a comma at the end of the stream. If false, it means that this is the end of the json file, nothing will be added after that.
         *
         * See wocra::WocraController::getPerformances() to know more. Here it saves:
         *
         *  - controller_update_tasks
         *  - controller_solve_problem
         *  - solver_prepare
         *  - solver_solve
         */
        void writePerformanceInStream(std::ostream& myOstream, bool addCommaAtEnd) const;
        
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
         * See wocra::WocraController::writePerformanceInStream(std::ostream&, bool) const and wocra::OneLevelSolver::writePerformanceInStream(std::ostream&, bool).
         */
        std::string getPerformances() const;
        
        /**
         *  Adds a Linear constraint that is equivalent for the full & reduced problems.
         *
         *  @param Linear constraint.
         */
        void addConstraint(ocra::LinearConstraint& constraint) const;
        
        /**
         *  Removes a Linear constraint that is equivalent for the full & reduced problems.
         *
         *  @param Linear Constraint to remove.
         */
        void removeConstraint(ocra::LinearConstraint& constraint) const;
        
        /**
         *  Adds a Linear constraint that has different expressions, depending on the problem type. In this case, the constraint needs to be connected to the controller to get the matrices for the problem reduction.
         * See wocra::ControlConstraint for more info.
         *
         *  @param constraint Constraint.
         */
        void addConstraint(ocra::ControlConstraint& constraint) const;
        
        /**
         *  Removes a Linear constraint that has different expressions, depending on the problem type. In this case, the constraint needs to be disconnected from the controller.
         *
         *  @param constraint Constraint.
         */
        void removeConstraint(ocra::ControlConstraint& constraint) const;
        
        // WocraTask& createWocraTask(const std::string& name, Feature::Ptr feature, Feature::Ptr featureDes) const;
        // WocraTask& createWocraTask(const std::string& name, Feature::Ptr feature) const;
        // WocraTask& createWocraContactTask(const std::string& name, PointContactFeature::Ptr feature, double mu, double margin) const;
        
        
    protected:
        
        virtual void doComputeOutput(Eigen::VectorXd& tau);
        
        /**
         *  Internal implementation inside the addTask method.
         *
         *  @param task The task to add in the controller.
         */
        virtual void doAddTask(std::shared_ptr<Task> task);
        
        /**
         * Internal implementation inside the addContactSet method.
         *
         * @param contacts The contact set to add in the controller.
         * @todo this method has not been implemented!!!
         */
        virtual void doAddContactSet(const ContactSet& contacts);
        
        
        
    protected: // factory
        /** Internal implementation inside the createTask method.
         *
         * @param name The task name, a unique identifier
         * @param feature The part of the robot one wants to control (full state, frame, CoM,...)
         * @param featureDes The desired state one wants to reach, depends on the \a feature argument
         * @return The pointer to the new created task
         *
         * This method is called by the higher level methods createWocraTask(const std::string&, const Feature&, const Feature&) const
         * and is the concrete implementation required by the ocra Controller class.
         */
        virtual std::shared_ptr<Task> doCreateTask(const std::string& name, Feature::Ptr feature, Feature::Ptr featureDes) const;
        
        /** Internal implementation inside the createTask method.
         *
         * @param name The task name, a unique identifier
         * @param feature The part of the robot one wants to control (full state, frame, CoM,...)
         * @return The pointer to the new created task
         *
         * This method is called by the higher level methods #createWocraTask(const std::string&, const Feature&) const
         * and is the concrete implementation required by the ocra Controller class.
         */
        virtual std::shared_ptr<Task> doCreateTask(const std::string& name, Feature::Ptr feature) const;
        
        /** Internal implementation inside the createContactTask method.
         *
         * @param name     The task name, a unique identifier
         * @param feature  The contact point feature of the robot one wants to control
         * @param mu       The friction cone coefficient \f$ \mu \f$ such as \f$ \Vert \force_t \Vert < \mu \force_n \f$
         * @param margin   The margin inside the friction cone
         * @return The pointer to the new created contact task
         *
         * This method is called by the higher level methods createWocraContactTask(const std::string&, PointContactFeature::Ptr, , double, double) const
         * and is the concrete implementation required by the ocra::Controller class.
         */
        virtual std::shared_ptr<Task> doCreateContactTask(const std::string& name, PointContactFeature::Ptr feature, double mu, double margin) const;
        
        
    private:
        
        /**
         *  Structure creating, initializing and storing the main objects of the controller, i.e.:
         *   - Robot model.
         *   - Solver to be used by the Wocra Controller.
         *   - Full dynamics equation which is used as the equality constraint of the problem.
         *   - Minimization tasks for whole variable minimization (joints accelerations, joint torques and contact forces).
         */
        struct Pimpl;
        
        boost::shared_ptr<Pimpl> pimpl;
        
        std::shared_ptr<ocra::JointLimitConstraint> jointLimitConstraint;
        std::shared_ptr<ocra::TorqueLimitConstraint> torqueLimitConstraint;
        
    };
    
    
    
    /** \} */ // end group core
    
    
    
} //end namespace wocra







#endif

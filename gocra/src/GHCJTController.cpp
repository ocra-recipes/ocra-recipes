/**
 * \file GHCJTController.cpp
 * \author Mingxing Liu
 *
 * \brief Implement the quasi-static Generalized Hierarchical Control based on Jacobian Transpose (GHCJT).
 *
 */

#include "gocra/GHCJTController.h"

#include <iostream>

#include "ocra/optim/OneLevelSolver.h"
#include "ocra/optim/QuadraticFunction.h"
#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/SumOfLinearFunctions.h"
#include "ocra/control/FullState.h"
#include "ocra/control/Feature.h"
#include "ocra/control/Model.h"
#include "ocra/control/ContactSet.h"

#include "gocra/Tasks/GHCJTTask.h"
#include "gocra/Performances.h"
#include "gocra/OrthonormalFamily.h"
//#include <QDebug>

namespace gocra
{




struct GHCJTController::Pimpl
{
    Model&       innerModel;
    ocra::OneLevelSolver&  innerSolver;
    bool         isFreeFloating;
    bool         useGrav;

    // STATIC EQUILIBRIUM EQUATION
    EqualZeroConstraintPtr<SumOfLinearFunctions> seConstraint;

    std::vector< GHCJTTask* >                 createdTask;
    std::vector< GHCJTTask* >                 activeTask;
    int                                      nbActiveTask;
    int                                      totalActiveTaskDim;
    MatrixXd                                 augmentedJacobian;

    // PERFORMANCE RECORDERS
    PerformanceRecorder updateTasksRecorder;
    PerformanceRecorder solveProblemRecorder;

    Pimpl(const std::string& name, Model& m, ocra::OneLevelSolver&  s, bool useGrav)
        : innerModel(m)
        , innerSolver(s)
        , isFreeFloating(m.nbDofs() - m.nbInternalDofs()>0)
        , useGrav(useGrav)
        , seConstraint( new SumOfLinearFunctions(m.nbDofs() - m.nbInternalDofs()) )
        , createdTask()
        , activeTask()
        , nbActiveTask(0)
        , totalActiveTaskDim(0)
        , augmentedJacobian(MatrixXd::Zero(1,m.nbDofs()))
    {

//        createdTask.reserve(20);
//        activeTask.reserve(20);
    }

    ~Pimpl()
    {

    }
};



/** Initialize GHCJT controller.
 *
 * \param ctrlName The name of the controller
 * \param innerModel The internal model of the robot one wants to control
 * \param innerSolver The internal solver one wants to use to make the quadratic optimization
 * \param useGrav Whether to activate gravity compensation or not
 */
GHCJTController::GHCJTController(const std::string& ctrlName, Model& innerModel, ocra::OneLevelSolver& innerSolver, bool useGrav)
    : Controller(ctrlName, innerModel)
    , pimpl( new Pimpl(ctrlName, innerModel, innerSolver, useGrav) )
{
    if (pimpl->isFreeFloating)
        pimpl->innerSolver.addConstraint(pimpl->seConstraint.getConstraint());
}

/** Destructor
 */
GHCJTController::~GHCJTController()
{

    for (size_t i=0; i<pimpl->createdTask.size(); ++i)
    {
        pimpl->createdTask[i]->disconnectFromController();
        delete pimpl->createdTask[i];
    }

    if (pimpl->isFreeFloating)
        pimpl->innerSolver.removeConstraint(pimpl->seConstraint.getConstraint());

}


/**
 * \return the inner model used to construct this controller instance
 */
Model& GHCJTController::getModel()
{
    return pimpl->innerModel;
}

/**
 * \return the inner solver used to construct this controller instance
 */
ocra::OneLevelSolver& GHCJTController::getSolver()
{
    return pimpl->innerSolver;
}


void GHCJTController::takeIntoAccountGravity(bool useGrav)
{

}



void GHCJTController::addConstraint(ocra::LinearConstraint& constraint) const
{
    pimpl->innerSolver.addConstraint(constraint);
}


void GHCJTController::removeConstraint(ocra::LinearConstraint& constraint) const
{
    pimpl->innerSolver.removeConstraint(constraint);
}

/*
void GHCJTController::addConstraint(gOcraConstraint& constraint) const
{
    pimpl->innerSolver.addConstraint(constraint.getConstraint());
}


void GHCJTController::removeConstraint(gOcraConstraint& constraint) const
{
    pimpl->innerSolver.removeConstraint(constraint.getConstraint());
}
*/


/** Internal implementation inside the addTask method.
 *
 * \param task The task to add in the controller
 */
void GHCJTController::doAddTask(Task& task)
{
    try {
        GHCJTTask& ctask = dynamic_cast<GHCJTTask&>(task);
        ctask.connectToController(pimpl->innerSolver, pimpl->seConstraint.getFunction());
    }
    catch(const std::exception & e) {
        std::cerr << e.what() ;
        throw std::runtime_error("[GHCJTController::doAddTask] cannot add task to controller (wrong type)");
    }
}

/** Internal implementation inside the addContactSet method.
 *
 * \param contacts The contact set to add in the controller
 * \todo this method has not been implemented!!!
 */
void GHCJTController::doAddContactSet(const ContactSet& contacts)
{
    throw std::runtime_error("[GHCJTController::doAddTask] not implemented");
}




GHCJTTask& GHCJTController::createGHCJTTask(const std::string& name, const Feature& feature, const Feature& featureDes) const
{
    return dynamic_cast<GHCJTTask&>(createTask(name, feature, featureDes));
}

GHCJTTask& GHCJTController::createGHCJTTask(const std::string& name, const Feature& feature) const
{
    return dynamic_cast<GHCJTTask&>(createTask(name, feature));
}

GHCJTTask& GHCJTController::createGHCJTContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const
{
    return dynamic_cast<GHCJTTask&>(createContactTask(name, feature, mu, margin));
}





/** Internal implementation inside the createTask method.
 *
 * \param name The task name, a unique identifier
 * \param feature The part of the robot one wants to control (full state, frame, CoM,...)
 * \param featureDes The desired state one wants to reach, depends on the \a feature argument
 * \return The pointer to the new created task
 *
 * This method is called by the higher level methods #createGHCJTTask(const std::string&, const Feature&, const Feature&, int, double) const
 * and #createGHCJTTask(const std::string&, const Feature&, int, double) const and is the concrete implementation required by the xde Controller class.
 */
Task* GHCJTController::doCreateTask(const std::string& name, const Feature& feature, const Feature& featureDes) const
{
    GHCJTTask* nTask = new GHCJTTask(name, pimpl->innerModel, feature, featureDes);
    pimpl->createdTask.push_back(nTask);
    return nTask;
}

/** Internal implementation inside the createTask method.
 *
 * \param name The task name, a unique identifier
 * \param feature The part of the robot one wants to control (full state, frame, CoM,...)
 * \return The pointer to the new created task
 *
 * This method is called by the higher level methods #createGHCJTTask(const std::string&, const Feature&, const Feature&, int, double) const
 * and #createGHCJTTask(const std::string&, const Feature&, int, double) const and is the concrete implementation required by the xde Controller class.
 */
Task* GHCJTController::doCreateTask(const std::string& name, const Feature& feature) const
{
    GHCJTTask* nTask = new GHCJTTask(name, pimpl->innerModel, feature);
    pimpl->createdTask.push_back(nTask);
    return nTask;
}

/** Internal implementation inside the createContactTask method.
 *
 * \param name The task name, a unique identifier
 * \param feature The contact point feature of the robot one wants to control
 * \param mu The friction cone coefficient \f$ \mu \f$ such as \f$ \Vert \force_t \Vert < \mu \force_n \f$
 * \param margin The margin inside the friction cone
 * \return The pointer to the new created contact task
 *
 * This method is called by the higher level methods #createGHCJTContactTask(const std::string&, const PointContactFeature&, , double, double, int, double) const
 * and is the concrete implementation required by the xde Controller class.
 */
Task* GHCJTController::doCreateContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const
{
    GHCJTTask* nTask = new GHCJTTask(name, pimpl->innerModel, feature);
    pimpl->createdTask.push_back(nTask);
    return nTask;
}




//MOST IMPORTANT FUNCTION: COMPUTE OUTPUT TORQUE
/** Compute the output of the controller.
 *
 * \param tau The joint torques, which is the output of our problem
 *
 * Here, the controller solves the optimization problem depending on the tasks and constraints, and the result is set in the variable of the problem
 * \f$ \force_c \f$. The torque \f$ \tau \f$ is computed using \f$ \force_c \f$, and it is finally applied to the robot.
 */
void GHCJTController::doComputeOutput(Eigen::VectorXd& tau)
{
    pimpl->updateTasksRecorder.initializeTime();

    const std::vector<Task*>& tasks = getActiveTasks();

    if(tasks.size()==0)
    {
      tau.resize(pimpl->innerModel.nbInternalDofs());
      tau.setZero();
      return;
    }
    if (pimpl->useGrav && pimpl->isFreeFloating)
    {
        // linear function : Ax + b = 0
        VectorXd b = pimpl->innerModel.getGravityTerms().head(pimpl->innerModel.nbDofs() - pimpl->innerModel.nbInternalDofs());

        pimpl->seConstraint.getFunction().changeb(b);

    }
    for(size_t i=0; i< tasks.size(); ++i)
    {
        GHCJTTask* cTask = static_cast<GHCJTTask*>(tasks[i]); // addTask throws if this cast is not possible
        cTask->update();

    }


    pimpl->updateTasksRecorder.saveRelativeTime();
    pimpl->solveProblemRecorder.initializeTime();
    if(!pimpl->innerSolver.solve().info)
    {
        if(pimpl->useGrav)
            tau = pimpl->innerModel.getGravityTerms().tail(pimpl->innerModel.nbInternalDofs());

        for(int i = 0; i < tasks.size(); ++i)
        {
          GHCJTTask& currentTask = static_cast<GHCJTTask&>(*tasks[i]);

          if(!currentTask.isBodyContactConstraint())
          {

              if(currentTask.isPointContactTask())
                  tau += currentTask.getJacobian().transpose().bottomRows(pimpl->innerModel.nbInternalDofs()) * currentTask.getVariable().getValue();
              else
                  tau += (currentTask.getProjector()*currentTask.getJacobian().transpose()).bottomRows(pimpl->innerModel.nbInternalDofs()) * currentTask.getVariable().getValue();

          }

        }
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
 * \param addCommaAtEnd If true, add a comma at the end of the stream. If false, it means that this is the end of the json file, nothing will be added after that, no comma is added.
 *
 * See gocra::GHCJTController::getPerformances() to know more. Here it saves:
 *
 *  - controller_update_tasks
 *  - controller_solve_problem
 */
void GHCJTController::writePerformanceInStream(std::ostream& outstream, bool addCommaAtEnd) const
{
    pimpl->updateTasksRecorder.writeInStream("controller_update_tasks", outstream, true);
    pimpl->solveProblemRecorder.writeInStream("controller_solve_problem", outstream, true);
    pimpl->innerSolver.writePerformanceInStream(outstream, addCommaAtEnd);
}



/** Get information about performances through a string.
 *
 * Information are saved in a JSON way (http://www.json.org/). It returns a of dictionnary on the form:
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
 * See gocra::GHCJTController::writePerformanceInStream(std::ostream&, bool) and gocra::ocra::OneLevelSolver::writePerformanceInStream(std::ostream&, bool).
 */
std::string GHCJTController::getPerformances() const
{
    std::ostringstream osstream;
    // Write it in a json style
    osstream <<"{\n";
    writePerformanceInStream(osstream, false);
    osstream <<"}";
    return osstream.str();
}

void GHCJTController::setActiveTaskVector()
{
    pimpl->activeTask.clear();
    int totalTaskDim = 0;
    for (int i=0; i<pimpl->createdTask.size(); ++i)
    {
        if (pimpl->createdTask[i]->isActiveAsObjective())
        {
            if(!pimpl->createdTask[i]->isPointContactTask())
            {
                pimpl->activeTask.push_back(pimpl->createdTask[i]);
                totalTaskDim += pimpl->createdTask[i]->getTaskDimension();
                pimpl->createdTask[i]->setWeight(1);
            }
        }
    }
    pimpl->nbActiveTask = pimpl->activeTask.size();
    pimpl->totalActiveTaskDim = totalTaskDim;

    doUpdateAugmentedJacobian();
    initPriorityMatrix();

}

void GHCJTController::doUpdateAugmentedJacobian()
{
     int nDof = pimpl->innerModel.nbDofs();
     MatrixXd jacobian=MatrixXd::Zero(pimpl->totalActiveTaskDim, nDof);
     int rowindex = 0;
     int taskdim;
//     int index = pimpl->innerModel.hasFixedRoot() ? 0 : 6;

     for (int i=0; i<pimpl->nbActiveTask; ++i)
     {
         taskdim = pimpl->activeTask[i]->getDimension();
         jacobian.block(rowindex,0, taskdim, nDof)=pimpl->activeTask[i]->getJacobian();
         rowindex += taskdim;
     }
     pimpl->augmentedJacobian = jacobian;
}

void GHCJTController::doUpdateProjector()
{
     doUpdateAugmentedJacobian();
     const int nDof = pimpl->innerModel.nbDofs();
     MatrixXd proj(nDof,nDof);
     MatrixXd taskiProj(nDof,nDof);
     for (int i=0; i<pimpl->nbActiveTask; ++i)
     {
//         std::cout<<"tp"<<i<<"="<<pimpl->activeTask[i]->getPriority()<<std::endl;
         computeProjector(pimpl->activeTask[i]->getPriority(), pimpl->augmentedJacobian, proj);
         pimpl->activeTask[i]->setProjector(proj);
         computeTaskiProjector(pimpl->activeTask[i]->getJacobian(), taskiProj);
         pimpl->activeTask[i]->setTaskiProjector(taskiProj);
     }
}
std::vector< GHCJTTask* >& GHCJTController::getActiveTask()
{
    return pimpl->activeTask;
}

int GHCJTController::getNbActiveTask() const
{
    return pimpl->nbActiveTask;
}

int GHCJTController::getTotalActiveTaskDimensions() const
{
    return pimpl->totalActiveTaskDim;
}

void GHCJTController::initPriorityMatrix()
{
    int dim;
    int dim_alpha = 0;
    for (int i=0; i<pimpl->activeTask.size(); ++i)
    {
        dim = pimpl->activeTask[i]->getTaskDimension();
        pimpl->activeTask[i]->setIndexBegin(dim_alpha);
        pimpl->activeTask[i]->setIndexEnd(dim_alpha + dim);
        dim_alpha += dim;
    }
//    MatrixXd m_alpha = MatrixXd::Zero(dim_alpha,dim_alpha);

}

std::pair<VectorXd, MatrixXd> GHCJTController::sortRows(const MatrixXd &C, const MatrixXd &J)
{
    int totalTaskDim = J.rows();
    int nDof = J.cols();
    MatrixXd Cii_J = MatrixXd::Zero(totalTaskDim, 1+nDof);

    Cii_J.col(0) = C.diagonal();
    Cii_J.block(0,1,totalTaskDim,nDof) = J;

    int rdim = Cii_J.rows();
    VectorXd tmp= VectorXd::Zero(1+nDof);
    for (int rmin=0; rmin<rdim-1;++rmin)
    {
        for(int i=rdim-1; i>rmin; --i)
        {
            if (Cii_J(i,0)>Cii_J(i-1,0))
            {
                tmp = Cii_J.row(i-1);
                Cii_J.row(i-1) = Cii_J.row(i);
                Cii_J.row(i) = tmp;
            }
        }
    }

    return std::pair<VectorXd, MatrixXd>(Cii_J.col(0),Cii_J.block(0,1,totalTaskDim,nDof));
}

void GHCJTController::computeProjector(const MatrixXd &C, const MatrixXd &J, MatrixXd& projector)
{
    int totalTaskDim = J.rows();
    const int nDof = pimpl->innerModel.nbDofs();
    VectorXd Cs(nDof);
    MatrixXd Js(totalTaskDim, nDof);

    std::pair<VectorXd, MatrixXd> sortedMatrix = sortRows(C,J);
    Cs = sortedMatrix.first;
    Js = sortedMatrix.second;
    gocra::OrthonormalFamily onfamily(Js, 1e-9);
    onfamily.computeOrthonormalFamily();
    MatrixXd onb_Js = onfamily.getOnf();
    VectorXi origin = onfamily.getOrigin();
//    std::cout<<"origin"<<std::endl;
//    std::cout<<origin<<std::endl;
    int k = onfamily.getIndex();
    VectorXd Chat(k);

    for (int j=0; j<k; ++j)
    {
        Chat(j)=Cs(origin(j));
    }


    MatrixXd alpha = MatrixXd(Chat.asDiagonal());
//    std::cout<<"Cs"<<std::endl;
//    std::cout<<Cs<<std::endl;
//    std::cout<<"Chat"<<std::endl;
//    std::cout<<Chat<<std::endl;
//    std::cout<<"onb_js"<<std::endl;
//    std::cout<<onb_Js<<std::endl;
//    std::cout<<"alpha"<<std::endl;
//    std::cout<<alpha<<std::endl;
//    projector = MatrixXd::Zero(nDof,nDof);
    projector = MatrixXd::Identity(nDof,nDof) - onb_Js.transpose()*alpha*onb_Js;
}

void GHCJTController::computeTaskiProjector(const MatrixXd &J, MatrixXd& projector)
{

    gocra::OrthonormalFamily onfamily(J, 1e-9);
    onfamily.computeOrthonormalFamily();
    MatrixXd onb_Js = onfamily.getOnf();
    const int nDof = pimpl->innerModel.nbDofs();

    projector = MatrixXd::Identity(nDof,nDof) - onb_Js.transpose()*onb_Js;
}


void GHCJTController::setTaskProjectors(MatrixXd& param)
{
//    initPriorityMatrix(param);

    VectorXd vec_ij;
    int dim_j;
    MatrixXd m_priority(pimpl->totalActiveTaskDim,pimpl->totalActiveTaskDim);
    m_priority.setZero();

    int nt = pimpl->nbActiveTask;
    for (int i=0; i<nt; ++i)
    {
        for (int j=0; j<nt; ++j)
        {
            dim_j = pimpl->activeTask[j]->getTaskDimension();
            vec_ij = VectorXd::Zero(dim_j);
            vec_ij = vec_ij.setConstant(param(i,j));
            m_priority.block(pimpl->activeTask[j]->getIndexBegin(), pimpl->activeTask[j]->getIndexBegin(),dim_j,dim_j) = MatrixXd(vec_ij.asDiagonal());
        }

        pimpl->activeTask[i]->setPriority(m_priority);
//        std::cout<<"mp="<<m_priority<<std::endl;
    }

    const int nDof = pimpl->innerModel.nbDofs();
    MatrixXd proj(nDof,nDof);
    MatrixXd taskiProj(nDof,nDof);

    for (int i=0; i<pimpl->nbActiveTask; ++i)
    {
        computeProjector(pimpl->activeTask[i]->getPriority(), pimpl->augmentedJacobian, proj);
        pimpl->activeTask[i]->setProjector(proj);
        computeTaskiProjector(pimpl->activeTask[i]->getJacobian(), taskiProj);
        pimpl->activeTask[i]->setTaskiProjector(taskiProj);
    }

}


} // namespace gocra

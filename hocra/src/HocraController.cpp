#include "hocra/HocraController.h"


namespace hocra {
using namespace ocra;

struct HocraController::Pimpl
{
    Model::Ptr          innerModel;
    OneLevelSolver::Ptr levelSolver;
    bool                            reducedProblem;
    CascadeQPSolver cascadeQPSolver;

    // EQUALITY CONSTRAINT OF THE DYNAMIC EQUATION
    ocra::EqualZeroConstraintPtr< ocra::FullDynamicEquationFunction >      dynamicEquation;

    // MINIMIZATION TASK FOR WHOLE VARIABLE MINIMIZATION
    ocra::QuadraticFunction*     minDdqFunction;
    ocra::QuadraticFunction*     minTauFunction;
//     FcQuadraticFunction*        minFcFunction;

    ObjectivePtr<ocra::QuadraticFunction>     minDdqObjective;
    ObjectivePtr<ocra::QuadraticFunction>     minTauObjective;
    ObjectivePtr<ocra::QuadraticFunction>     minFcObjective;


    Pimpl(Model::Ptr m, OneLevelSolver::Ptr s, bool useReducedProblem)

        : innerModel(m)
        , levelSolver(s)
        , reducedProblem(useReducedProblem)
        , dynamicEquation( new ocra::FullDynamicEquationFunction(*m) )

        , minDdqFunction(  new ocra::QuadraticFunction(m->getAccelerationVariable(), Eigen::MatrixXd::Identity(m->nbDofs(), m->nbDofs()), Eigen::VectorXd::Zero(m->nbDofs()), 0) )

        , minTauFunction(  new ocra::QuadraticFunction(m->getJointTorqueVariable(), Eigen::MatrixXd::Identity(m->getJointTorqueVariable().getSize(), m->getJointTorqueVariable().getSize()), Eigen::VectorXd::Zero(m->getJointTorqueVariable().getSize()), 0) )

//         , minFcFunction(   new FcQuadraticFunction(m->getModelContacts().getContactForcesVariable()) )

    {
        minDdqObjective.set(minDdqFunction);
        minTauObjective.set(minTauFunction);
//         minFcObjective.set(minFcFunction);
        std::cout << "Pimpl Constructor" << std::endl;

    }

    ~Pimpl()
    {
    }

};

HocraController::HocraController(const std::string& ctrlName,
                                 Model::Ptr innerModel,
                                 OneLevelSolver::Ptr levelSolver,
                                 bool useReducedProblem):
Controller(ctrlName, *innerModel),
pimpl( new Pimpl(innerModel, levelSolver, useReducedProblem) )
{
    std::cout <<"Constructing Hocra Controller"<<std::endl; 
}
void HocraController::doAddContactSet(const ContactSet& contacts)
{
    std::cout << "HocraController::doAddContactSet" << std::endl;
}
void HocraController::doAddTask(Task::Ptr task)
{
    std::cout << "Adding task "<<task->getName()<<" at level "<<task->getHierarchyLevel() << std::endl;
    pimpl->cascadeQPSolver.addTask(task);
    pimpl->cascadeQPSolver.addSolver(pimpl->levelSolver,task->getHierarchyLevel());
}
void HocraController::doComputeOutput(VectorXd& tau)
{
    std::cout << "HocraController::doComputeOutput" << std::endl;
    throw std::runtime_error("[HocraController::doComputeOutput] not implemented");
}
Task::Ptr HocraController::doCreateContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const
{
    std::cout << "HocraController::doCreateContactTask" << std::endl;
    throw std::runtime_error("[HocraController::doCreateContactTask] not implemented");

}
Task::Ptr HocraController::doCreateTask(const std::string& name, const Feature& feature, const Feature& featureDes) const
{
    std::cout << "HocraController::doCreateTask des" << std::endl;
    return std::make_shared<ocra::OneLevelTask>(name, pimpl->innerModel, feature, featureDes);

}
Task::Ptr HocraController::doCreateTask(const std::string& name, const Feature& feature) const
{
    std::cout << "HocraController::doCreateTask" << std::endl;
    return  std::make_shared<ocra::OneLevelTask>(name, pimpl->innerModel, feature);
}

}

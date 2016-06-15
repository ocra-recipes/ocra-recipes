#include "hocra/HocraController.h"


namespace hocra {
using namespace ocra;



HocraController::HocraController(const std::string& _ctrlName,
                                 Model::Ptr _innerModel,
                                 OneLevelSolver::Ptr _levelSolver,
                                 bool _useReducedProblem):
    Controller(_ctrlName, *_innerModel),
    innerModel(_innerModel),
    cascadeQPSolver(new CascadeQPSolver(_ctrlName,_innerModel,_levelSolver,_useReducedProblem))
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
    cascadeQPSolver->addTask(task);
}

void HocraController::doComputeOutput(VectorXd& tau)
{
    auto tasks = getActiveTasks();

    for(auto task : tasks)
    {
        task->update();
    }
    try {
        if(!cascadeQPSolver->solve().info)
        {
             tau = innerModel->getJointTorqueVariable().getValue();
        }
    }catch(const std::exception & e) {
        std::cerr << e.what() ;
        throw std::runtime_error("[HocraController::doComputeOutput] Error while computing output");
    }
}
Task::Ptr HocraController::doCreateContactTask(const std::string& name, PointContactFeature::Ptr feature, double mu, double margin) const
{
    std::cout << "HocraController::doCreateContactTask" << std::endl;
    return std::make_shared<ocra::Task>(name, innerModel, feature);

}
Task::Ptr HocraController::doCreateTask(const std::string& name, Feature::Ptr feature, Feature::Ptr featureDes) const
{
    std::cout << "HocraController::doCreateTask des" << std::endl;
    return std::make_shared<ocra::Task>(name, innerModel, feature, featureDes);

}
Task::Ptr HocraController::doCreateTask(const std::string& name, Feature::Ptr feature) const
{
    std::cout << "HocraController::doCreateTask" << std::endl;
    return  std::make_shared<ocra::Task>(name, innerModel, feature);
}

}

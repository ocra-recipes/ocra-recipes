#include "hocra/HocraController.h"


namespace hocra {
using namespace ocra;

struct HocraController::Pimpl
{
    std::shared_ptr<Model>          innerModel;
    std::vector<std::shared_ptr<OneLevelSolver> > innerSolver;
    bool                            reducedProblem;


    // EQUALITY CONSTRAINT OF THE DYNAMIC EQUATION
    ocra::EqualZeroConstraintPtr< ocra::FullDynamicEquationFunction >      dynamicEquation;

    // MINIMIZATION TASK FOR WHOLE VARIABLE MINIMIZATION
    ocra::QuadraticFunction*     minDdqFunction;
    ocra::QuadraticFunction*     minTauFunction;
//     FcQuadraticFunction*        minFcFunction;

    ObjectivePtr<ocra::QuadraticFunction>     minDdqObjective;
    ObjectivePtr<ocra::QuadraticFunction>     minTauObjective;
    ObjectivePtr<ocra::QuadraticFunction>     minFcObjective;


    Pimpl(std::shared_ptr<Model> m, std::shared_ptr<OneLevelSolver> s, bool useReducedProblem)

        : innerModel(m)

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
                                 std::shared_ptr< Model > innerModel, 
                                 std::shared_ptr< OneLevelSolver > innerSolver, 
                                 bool useReducedProblem): 
Controller(ctrlName, *innerModel), 
pimpl( new Pimpl(innerModel, innerSolver, useReducedProblem) )
{
    //pimpl->solver.push_back(std::shared_ptr<OneLevelSolver>(innerModel));
}
void HocraController::doAddContactSet(const ContactSet& contacts)
{
std::cout << "HocraController::doAddContactSet" << std::endl;
}
void HocraController::doAddTask(std::shared_ptr< Task > task)
{
std::cout << "HocraController::doAddTask" << std::endl;

}
void HocraController::doComputeOutput(VectorXd& tau)
{
std::cout << "HocraController::doComputeOutput" << std::endl;

}
std::shared_ptr< Task > HocraController::doCreateContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const
{
std::cout << "HocraController::doCreateContactTask" << std::endl;

}
std::shared_ptr< Task > HocraController::doCreateTask(const std::string& name, const Feature& feature, const Feature& featureDes) const
{
std::cout << "HocraController::doCreateTask des" << std::endl;
//     return std::make_shared<ocra::OneLevelTask>(name, pimpl->innerModel, feature, featureDes);
}
std::shared_ptr< Task > HocraController::doCreateTask(const std::string& name, const Feature& feature) const
{
    std::cout << "HocraController::doCreateTask" << std::endl;
//     return std::make_shared<ocra::OneLevelTask>(name, pimpl->innerModel, feature);
}

}
#include "ocra/control/TaskManagers/CoMMomentumTaskManager.h"

namespace ocra
{

/** Base constructor
 *
 * \param _ctrl                 ocra::Controller to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 */
CoMMomentumTaskManager::CoMMomentumTaskManager(ocra::Controller& _ctrl, const ocra::Model& _model, const std::string& _taskName, ocra::ECartesianDof _axes, double _damping, double _weight, bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), axes(_axes)
{
    _init(_damping, _weight);
}

CoMMomentumTaskManager::CoMMomentumTaskManager(ocra::Controller& _ctrl, const ocra::Model& _model, const std::string& _taskName, ocra::ECartesianDof _axes, double _damping, const Eigen::VectorXd& _weight, bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), axes(_axes)
{
    _init(_damping, _weight);
}




CoMMomentumTaskManager::~CoMMomentumTaskManager()
{

}

/** Initializer function for the CoMTaskManager constructor, sets up the frames, parameters, controller and task
 *
 */
void CoMMomentumTaskManager::_init(double damping, double weight)
{
    comFeatFrame = new ocra::CoMFrame(name + ".CoMFrame", model);
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::PositionFeature(name + ".PositionFeature", *comFeatFrame, axes);
    featDes = new ocra::PositionFeature(name + ".PositionFeature_Des", *featDesFrame, axes);

    task = ctrl.createTask(name, *feat, *featDes);
    task->setTaskType(ocra::Task::COMMOMENTUMTASK);
    ctrl.addTask(task);

//    task->setStiffness(stiffness);
    task->setDamping(damping);
    task->setWeight(weight);
    task->activateAsObjective();

//    setStateDimension(9, 3); //3 dof for pos vel and acc

//    // Set the desired state to the current position of the segment with 0 vel or acc
//    setState(model.getCoMPosition());
}

void CoMMomentumTaskManager::_init(double damping, const Eigen::VectorXd& weight)
{
    comFeatFrame = new ocra::CoMFrame(name + ".CoMFrame", model);
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::PositionFeature(name + ".PositionFeature", *comFeatFrame, axes);
    featDes = new ocra::PositionFeature(name + ".PositionFeature_Des", *featDesFrame, axes);

    task = ctrl.createTask(name, *feat, *featDes);
    task->setTaskType(ocra::Task::COMMOMENTUMTASK);
    ctrl.addTask(task);

//    task->setStiffness(stiffness);
    task->setDamping(damping);
    task->setWeight(weight);
    task->activateAsObjective();

//    setStateDimension(9, 3); //3 dof for pos vel and acc

//    // Set the desired state to the current position of the segment with 0 vel or acc
//    setState(model.getCoMPosition());
}

///** Sets the position for the task, only the translational position
// *
// * \param position                  Vector for desired position
// */
//void CoMMomentumTaskManager::setState(const Eigen::Vector3d& position)
//{
//    setState(position, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
//}

///** Sets the position, linear velocity and linear acceleration for the task
// *
// * \param position                  Vector for desired position
// * \param velocity                  Vector for desired linear velocity
// * \param acceleration              Vector for desired linear acceleration
// */
//void CoMMomentumTaskManager::setState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration)
//{
//    featDesFrame->setPosition(Eigen::Displacementd(position));
//    featDesFrame->setVelocity(Eigen::Twistd(0.0, 0.0, 0.0, velocity(0), velocity(1), velocity(2)) );
//    featDesFrame->setAcceleration(Eigen::Twistd(0.0, 0.0, 0.0, acceleration(0), acceleration(1), acceleration(2)) );

//    eigenDesiredStateVector << position, velocity, acceleration;
//    updateDesiredStateVector(eigenDesiredStateVector.data());
//}


//void CoMMomentumTaskManager::setDesiredState()
//{
//    double * vectorStart = &newDesiredStateVector.front();
//    int dof = 3;
//    Eigen::VectorXd newPosition = Eigen::VectorXd::Map(vectorStart, dof);
//    Eigen::VectorXd newVelocity = Eigen::VectorXd::Map(vectorStart + dof, dof);
//    Eigen::VectorXd newAcceleration = Eigen::VectorXd::Map(vectorStart + (2*dof), dof);
//    setState(newPosition, newVelocity, newAcceleration);
//}


const double* CoMMomentumTaskManager::getCurrentState()
{
    eigenCurrentStateVector << comFeatFrame->getPosition().getTranslation(), comFeatFrame->getVelocity().getLinearVelocity(), comFeatFrame->getAcceleration().getLinearVelocity();
    return eigenCurrentStateVector.data();
}


std::string CoMMomentumTaskManager::getTaskManagerType()
{
    return "CoMMomentumTaskManager";
}



}

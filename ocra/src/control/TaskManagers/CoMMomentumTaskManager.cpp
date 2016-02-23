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
    featFrame = new ocra::CoMFrame(name + ".CoMFrame", model);
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::PositionFeature(name + ".PositionFeature", *featFrame, axes);
    featDes = new ocra::PositionFeature(name + ".PositionFeature_Des", *featDesFrame, axes);

    task = &ctrl.createTask(name, *feat, *featDes);
    task->setTaskType(ocra::Task::COMMOMENTUMTASK);
    ctrl.addTask(*task);

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
    featFrame = new ocra::CoMFrame(name + ".CoMFrame", model);
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::PositionFeature(name + ".PositionFeature", *featFrame, axes);
    featDes = new ocra::PositionFeature(name + ".PositionFeature_Des", *featDesFrame, axes);

    task = &ctrl.createTask(name, *feat, *featDes);
    task->setTaskType(ocra::Task::COMMOMENTUMTASK);
    ctrl.addTask(*task);

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

/** Sets the weight constant for this task
 *
 *  \param weight               Desired weight value
 */
void CoMMomentumTaskManager::setWeight(double weight)
{
    task->setWeight(weight);
}

void CoMMomentumTaskManager::setWeight(const Eigen::VectorXd& weight)
{
    task->setWeight(weight);
}

/** Gets the weight constant for this task
 *
 *  \return                     The weight for this task
 */
Eigen::VectorXd CoMMomentumTaskManager::getWeight()
{
    return task->getWeight();
}

///** Sets the stiffness for this task
// *
// * \param stiffness             Desired stiffness
// */
//void CoMMomentumTaskManager::setStiffness(double stiffness)
//{
//    task->setStiffness(stiffness);
//}

///** Gets the stiffness constant for this task
// *
// *  \return                     The stiffness for this task
// */
//double CoMMomentumTaskManager::getStiffness()
//{
//    Eigen::MatrixXd K = task->getStiffness();
//    return K(0, 0);
//}

/** Sets the damping for this task
 *
 * \param damping               Desired damping
 */
void CoMMomentumTaskManager::setDamping(double damping)
{
    task->setDamping(damping);
}

/** Gets the damping constant for this task
 *
 *  \return                     The damping for this task
 */
double CoMMomentumTaskManager::getDamping()
{
    Eigen::MatrixXd C = task->getDamping();
    return C(0, 0);
}

/** Activates the task
 *
 */
void CoMMomentumTaskManager::activate()
{
    task->activateAsObjective();
}

/** Deactivates the task
 *
 */
void CoMMomentumTaskManager::deactivate()
{
    task->deactivate();
}

/** Gets the error for this task (COM position error)
 *
 */
//Eigen::VectorXd CoMMomentumTaskManager::getTaskError()
//{
//    return task->getError();
//}

const double* CoMMomentumTaskManager::getCurrentState()
{
    eigenCurrentStateVector << featFrame->getPosition().getTranslation(), featFrame->getVelocity().getLinearVelocity(), featFrame->getAcceleration().getLinearVelocity();
    return eigenCurrentStateVector.data();
}


std::string CoMMomentumTaskManager::getTaskManagerType()
{
    return "CoMMomentumTaskManager";
}

bool CoMMomentumTaskManager::checkIfActivated()
{
    return task->isActiveAsObjective();
}


}

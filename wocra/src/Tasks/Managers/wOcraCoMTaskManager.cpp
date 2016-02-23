#include "wocra/Tasks/Managers/wOcraCoMTaskManager.h"

namespace wocra
{

/** Base constructor
 *
 * \param _ctrl                 wOcraController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 */
wOcraCoMTaskManager::wOcraCoMTaskManager(wOcraController& _ctrl, const wOcraModel& _model, const std::string& _taskName, ocra::ECartesianDof _axes, double _stiffness, double _damping, double _weight, bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts), axes(_axes)
{
    _init(_stiffness, _damping, _weight);
}

wOcraCoMTaskManager::wOcraCoMTaskManager(wOcraController& _ctrl, const wOcraModel& _model, const std::string& _taskName, ocra::ECartesianDof _axes, double _stiffness, double _damping, const Eigen::VectorXd& _weight, bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts), axes(_axes)
{
    _init(_stiffness, _damping, _weight);
}


/** Constructor with initial desired position (in 3D cartesian space)
 *
 * \param _ctrl                 wOcraController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 * \param _posDes               Vector for desired position
 */
wOcraCoMTaskManager::wOcraCoMTaskManager(wOcraController& _ctrl, const wOcraModel& _model, const std::string& _taskName, ocra::ECartesianDof _axes, double _stiffness, double _damping, double _weight, Eigen::Vector3d _posDes, bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts), axes(_axes)
{
    _init(_stiffness, _damping, _weight);
    setState(_posDes);
}

wOcraCoMTaskManager::wOcraCoMTaskManager(wOcraController& _ctrl, const wOcraModel& _model, const std::string& _taskName, ocra::ECartesianDof _axes, double _stiffness, double _damping, const Eigen::VectorXd& _weight, Eigen::Vector3d _posDes, bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts), axes(_axes)
{
    _init(_stiffness, _damping, _weight);
    setState(_posDes);
}

/** Constructor with initial desired position, velocity and acceleration
 *
 * \param ctrl                  wOcraController to connect to
 * \param model                 ocra model to setup the task
 * \param taskName              Name of the task
 * \param axes                 The axes used for the task
 * \param stiffness             Stiffness constant for task
 * \param damping               Damping constant for task
 * \param weight                Weight constant for task
 * \param posDes                Vector for desired position
 * \param velDes                Vector for desired velocity
 * \param accDes                Vector for desired acceleration
 */
/*
wOcraCoMTaskManager::wOcraCoMTaskManager(wOcraController& ctrl, const Model& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, double weight, Eigen::Vector3d posDes, Eigen::Vector3d velDes, Eigen::Vector3d accDes)
    : _ctrl(ctrl), _model(model), _name(taskName)
{
    _init(stiffness, damping, weight);
}
*/

wOcraCoMTaskManager::~wOcraCoMTaskManager()
{

}

/** Initializer function for the wOcraCoMTaskManager constructor, sets up the frames, parameters, controller and task
 *
 */
void wOcraCoMTaskManager::_init(double stiffness, double damping, double weight)
{
    featFrame = new ocra::CoMFrame(name + ".CoMFrame", model);
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::PositionFeature(name + ".PositionFeature", *featFrame, axes);
    featDes = new ocra::PositionFeature(name + ".PositionFeature_Des", *featDesFrame, axes);

    task = &(ctrl.createwOcraTask(name, *feat, *featDes));
    task->initAsAccelerationTask();
    ctrl.addTask(*task);

    task->setStiffness(stiffness);
    task->setDamping(damping);
    task->setWeight(weight);
    task->activateAsObjective();

    setStateDimension(9, 3); //3 dof for pos vel and acc

    // Set the desired state to the current position of the segment with 0 vel or acc
    setState(model.getCoMPosition());
}

void wOcraCoMTaskManager::_init(double stiffness, double damping, const Eigen::VectorXd& weight)
{
    featFrame = new ocra::CoMFrame(name + ".CoMFrame", model);
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::PositionFeature(name + ".PositionFeature", *featFrame, axes);
    featDes = new ocra::PositionFeature(name + ".PositionFeature_Des", *featDesFrame, axes);

    task = &(ctrl.createwOcraTask(name, *feat, *featDes));
    task->initAsAccelerationTask();
    ctrl.addTask(*task);

    task->setStiffness(stiffness);
    task->setDamping(damping);
    task->setWeight(weight);
    task->activateAsObjective();

    setStateDimension(9, 3); //3 dof for pos vel and acc

    // Set the desired state to the current position of the segment with 0 vel or acc
    setState(model.getCoMPosition());
}

/** Sets the position for the task, only the translational position
 *
 * \param position                  Vector for desired position
 */
void wOcraCoMTaskManager::setState(const Eigen::Vector3d& position)
{
    setState(position, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

/** Sets the position, linear velocity and linear acceleration for the task
 *
 * \param position                  Vector for desired position
 * \param velocity                  Vector for desired linear velocity
 * \param acceleration              Vector for desired linear acceleration
 */
void wOcraCoMTaskManager::setState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration)
{
    featDesFrame->setPosition(Eigen::Displacementd(position));
    featDesFrame->setVelocity(Eigen::Twistd(0.0, 0.0, 0.0, velocity(0), velocity(1), velocity(2)) );
    featDesFrame->setAcceleration(Eigen::Twistd(0.0, 0.0, 0.0, acceleration(0), acceleration(1), acceleration(2)) );

    eigenDesiredStateVector << position, velocity, acceleration;
    updateDesiredStateVector(eigenDesiredStateVector.data());
}


void wOcraCoMTaskManager::setDesiredState()
{
    double * vectorStart = &newDesiredStateVector.front();
    int dof = 3;
    Eigen::VectorXd newPosition = Eigen::VectorXd::Map(vectorStart, dof);
    Eigen::VectorXd newVelocity = Eigen::VectorXd::Map(vectorStart + dof, dof);
    Eigen::VectorXd newAcceleration = Eigen::VectorXd::Map(vectorStart + (2*dof), dof);
    setState(newPosition, newVelocity, newAcceleration);
}

/** Sets the weight constant for this task
 *
 *  \param weight               Desired weight value
 */
void wOcraCoMTaskManager::setWeight(double weight)
{
    task->setWeight(weight);
}

void wOcraCoMTaskManager::setWeight(const Eigen::VectorXd& weight)
{
    task->setWeight(weight);
}

/** Gets the weight constant for this task
 *
 *  \return                     The weight for this task
 */
Eigen::VectorXd wOcraCoMTaskManager::getWeight()
{
    return task->getWeight();
}

/** Sets the stiffness for this task
 *
 * \param stiffness             Desired stiffness
 */
void wOcraCoMTaskManager::setStiffness(double stiffness)
{
    task->setStiffness(stiffness);
}

/** Gets the stiffness constant for this task
 *
 *  \return                     The stiffness for this task
 */
double wOcraCoMTaskManager::getStiffness()
{
    Eigen::MatrixXd K = task->getStiffness();
    return K(0, 0);
}

/** Sets the damping for this task
 *
 * \param damping               Desired damping
 */
void wOcraCoMTaskManager::setDamping(double damping)
{
    task->setDamping(damping);
}

/** Gets the damping constant for this task
 *
 *  \return                     The damping for this task
 */
double wOcraCoMTaskManager::getDamping()
{
    Eigen::MatrixXd C = task->getDamping();
    return C(0, 0);
}

/** Activates the task
 *
 */
void wOcraCoMTaskManager::activate()
{
    task->activateAsObjective();
}

/** Deactivates the task
 *
 */
void wOcraCoMTaskManager::deactivate()
{
    task->deactivate();
}

/** Gets the error for this task (COM position error)
 *
 */
Eigen::VectorXd wOcraCoMTaskManager::getTaskError()
{
    return task->getError();
}

const double* wOcraCoMTaskManager::getCurrentState()
{
    eigenCurrentStateVector << featFrame->getPosition().getTranslation(), featFrame->getVelocity().getLinearVelocity(), featFrame->getAcceleration().getLinearVelocity();
    return eigenCurrentStateVector.data();
}


std::string wOcraCoMTaskManager::getTaskManagerType()
{
    return "wOcraCoMTaskManager";
}

bool wOcraCoMTaskManager::checkIfActivated()
{
    return task->isActiveAsObjective();
}


}

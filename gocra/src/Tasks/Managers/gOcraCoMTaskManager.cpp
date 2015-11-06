#include "gocra/Tasks/Managers/gOcraCoMTaskManager.h"

namespace gocra
{

/** Base constructor
 *
 * \param _ctrl                 GHCJTController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 */
gOcraCoMTaskManager::gOcraCoMTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, ocra::ECartesianDof _axes, double _stiffness, double _damping)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName), axes(_axes)
{
    _init(_stiffness, _damping);
}

/** Constructor with initial desired position (in 3D cartesian space)
 *
 * \param _ctrl                 GHCJTController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _posDes               Vector for desired position
 */
gOcraCoMTaskManager::gOcraCoMTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, ocra::ECartesianDof _axes, double _stiffness, double _damping, Eigen::Vector3d _posDes)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName), axes(_axes)
{
    _init(_stiffness, _damping);
    setState(_posDes);
}

/** Constructor with initial desired position, velocity and acceleration
 *
 * \param ctrl                  GHCJTController to connect to
 * \param model                 ocra model to setup the task
 * \param taskName              Name of the task
 * \param stiffness             Stiffness constant for task
 * \param damping               Damping constant for task
 * \param posDes                Vector for desired position
 * \param velDes                Vector for desired velocity
 * \param accDes                Vector for desired acceleration
 */
/*
gOcraCoMTaskManager::gOcraCoMTaskManager(GHCJTController& ctrl, const Model& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, double weight, Eigen::Vector3d posDes, Eigen::Vector3d velDes, Eigen::Vector3d accDes)
    : _ctrl(ctrl), _model(model), _name(taskName), axes(_axes)
{
    _init(stiffness, damping, weight);
}
*/

/** Initializer function for the gOcraCoMTaskManager constructor, sets up the frames, parameters, controller and task
 *
 */
void gOcraCoMTaskManager::_init(double stiffness, double damping)
{
    featFrame = new ocra::CoMFrame(name + ".CoMFrame", model);
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::PositionFeature(name + ".PositionFeature", *featFrame, axes);
    featDes = new ocra::PositionFeature(name + ".PositionFeature_Des", *featDesFrame, axes);
    task = &(ctrl.createGHCJTTask(name, *feat, *featDes));
    ctrl.addTask(*task);
    task->setStiffness(stiffness);
    task->setDamping(damping);
    task->setWeight(1);
    task->activateAsObjective();

    // Set the desired state to the current position of the segment with 0 vel or acc
    setState(model.getCoMPosition());
}

/** Sets the position for the task, only the translational position
 *
 * \param position                  Vector for desired position
 */
void gOcraCoMTaskManager::setState(const Eigen::Vector3d& position)
{
    setState(position, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

/** Sets the position, linear velocity and linear acceleration for the task
 *
 * \param position                  Vector for desired position
 * \param velocity                  Vector for desired linear velocity
 * \param acceleration              Vector for desired linear acceleration
 */
void gOcraCoMTaskManager::setState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration)
{
    featDesFrame->setPosition(Eigen::Displacementd(position));
    featDesFrame->setVelocity(Eigen::Twistd(0.0, 0.0, 0.0, velocity(0), velocity(1), velocity(2)) );
    featDesFrame->setAcceleration(Eigen::Twistd(0.0, 0.0, 0.0, acceleration(0), acceleration(1), acceleration(2)) );
}



/** Sets the stiffness for this task 
 * 
 * \param stiffness             Desired stiffness
 */
void gOcraCoMTaskManager::setStiffness(double stiffness)
{
    task->setStiffness(stiffness);
}

/** Gets the stiffness constant for this task
 *
 *  \return                     The stiffness for this task 
 */
double gOcraCoMTaskManager::getStiffness()
{
    Eigen::MatrixXd K = task->getStiffness();
    return K(0, 0);
}

/** Sets the damping for this task 
 * 
 * \param damping               Desired damping
 */
void gOcraCoMTaskManager::setDamping(double damping)
{
    task->setDamping(damping);
}

/** Gets the damping constant for this task
 *
 *  \return                     The damping for this task 
 */
double gOcraCoMTaskManager::getDamping()
{
    Eigen::MatrixXd C = task->getDamping();
    return C(0, 0);
}

/** Activates the task
 *
 */
void gOcraCoMTaskManager::activate()
{
    task->activateAsObjective();
}

/** Deactivates the task
 *
 */
void gOcraCoMTaskManager::deactivate()
{
    task->deactivate();
}

/** Gets the error for this task (COM position error)
 * 
 */
Eigen::VectorXd gOcraCoMTaskManager::getTaskError()
{
    return task->getError();
}

}

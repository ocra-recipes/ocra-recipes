#include "gocra/Tasks/Managers/gOcraSegCartesianTaskManager.h"

namespace gocra
{

/** Base constructor
 *
 * \param _ctrl                 GHCJTController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 */
gOcraSegCartesianTaskManager::gOcraSegCartesianTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, const std::string& _segmentName, ocra::ECartesianDof _axes, double _stiffness, double _damping)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName), segmentName(_segmentName), axes(_axes)
{
    _init(Eigen::Vector3d::Zero(), _stiffness, _damping);
}

/** Constructor with the specifying the point of reference on the segment to track
 *
 * \param _ctrl                 GHCJTController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _segPoint_Local       The point to track the task in local frame
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 */
gOcraSegCartesianTaskManager::gOcraSegCartesianTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, const std::string& _segmentName, const Eigen::Vector3d& _segPoint_Local, ocra::ECartesianDof _axes, double _stiffness, double _damping)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName), segmentName(_segmentName), axes(_axes)
{
    _init(_segPoint_Local, _stiffness, _damping);
}

/** Constructor with desired pose
 *
 * \param _ctrl                 GHCJTController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _poseDes              Initial pose (cartesian) for task
 */
gOcraSegCartesianTaskManager::gOcraSegCartesianTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, const std::string& _segmentName, ocra::ECartesianDof _axes, double _stiffness, double _damping, const Eigen::Vector3d& _poseDes)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName), segmentName(_segmentName), axes(_axes)
{
    _init(Eigen::Vector3d::Zero(), _stiffness, _damping);
    // Have no idea this wrapper needs to be done
    setState(_poseDes);
}

/**
 * Constructor with both point on segment and desired pose
 *
 * \param _ctrl                 GHCJTController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _poseDes              Initial pose (cartesian) for task
 */
gOcraSegCartesianTaskManager::gOcraSegCartesianTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, const std::string& _segmentName, const Vector3d& _segPoint_Local, ocra::ECartesianDof _axes, double _stiffness, double _damping, const Eigen::Vector3d& _poseDes)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName), segmentName(_segmentName), axes(_axes)
{
    _init(_segPoint_Local, _stiffness, _damping);
    setState(_poseDes);
}


/** Initializer function for the constructor, sets up the frames, parameters, controller and task
 *
 */
void gOcraSegCartesianTaskManager::_init(const Eigen::Vector3d& _taskPoint_LocalFrame, double _stiffness, double _damping)
{
    featFrame = new ocra::SegmentFrame(name + ".SegmentFrame", model, model.SegmentName(segmentName), Eigen::Displacementd(_taskPoint_LocalFrame));
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::PositionFeature(name + ".PositionFeature", *featFrame, axes);
    featDes = new ocra::PositionFeature(name + ".PositionFeature_Des", *featDesFrame, axes);

         

    task = &(ctrl.createGHCJTTask(name, *feat, *featDes));
    ctrl.addTask(*task);

    task->activateAsObjective();
    task->setStiffness(_stiffness);
    task->setDamping(_damping);

    // Set the desired state to the current position of the segment with 0 vel or acc
    setState(model.getSegmentPosition(model.getSegmentIndex(segmentName)).getTranslation());
}

/** Sets the position for the task, only the translational position
 *
 * \param position                  Vector for desired position
 */
void gOcraSegCartesianTaskManager::setState(const Eigen::Vector3d& position)
{
    setState(position, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

/** Sets the position, linear velocity and linear acceleration for the task
 *
 * \param position                  Vector for desired position
 * \param velocity                  Vector for desired linear velocity
 * \param acceleration              Vector for desired linear acceleration
 */
void gOcraSegCartesianTaskManager::setState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration)
{
    featDesFrame->setPosition(Eigen::Displacementd(position));
    featDesFrame->setVelocity(Eigen::Twistd(0.0, 0.0, 0.0, velocity(0), velocity(1), velocity(2)) );
    featDesFrame->setAcceleration(Eigen::Twistd(0.0, 0.0, 0.0, acceleration(0), acceleration(1), acceleration(2)) );
}


/** Sets the stiffness for this task
 *
 * \param stiffness             Desired stiffness
 */
void gOcraSegCartesianTaskManager::setStiffness(double stiffness)
{
    task->setStiffness(stiffness);
}

/** Gets the stiffness constant for this task
 *
 *  \return                     The stiffness for this task
 */
double gOcraSegCartesianTaskManager::getStiffness()
{
    Eigen::MatrixXd K = task->getStiffness();
    return K(0, 0);
}

/** Sets the damping for this task
 *
 * \param damping               Desired damping
 */
void gOcraSegCartesianTaskManager::setDamping(double damping)
{
    task->setDamping(damping);
}

/** Gets the damping constant for this task
 *
 *  \return                     The damping for this task
 */
double gOcraSegCartesianTaskManager::getDamping()
{
    Eigen::MatrixXd C = task->getDamping();
    return C(0, 0);
}

/** Activates the task
 *
 */
void gOcraSegCartesianTaskManager::activate()
{
    task->activateAsObjective();
}

/** Deactivates the task
 *
 */
void gOcraSegCartesianTaskManager::deactivate()
{
    task->deactivate();
}

/** Gets the error for this task (Cartesian error)
 *
 */
Eigen::VectorXd gOcraSegCartesianTaskManager::getTaskError()
{
    return task->getError();
}


}

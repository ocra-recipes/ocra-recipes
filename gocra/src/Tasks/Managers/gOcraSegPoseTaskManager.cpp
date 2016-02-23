#include "gocra/Tasks/Managers/gOcraSegPoseTaskManager.h"

namespace gocra
{

/** Base constructor
 *
 * \param _ctrl                  GHCJTController to connect to
 * \param _model                 ocra model to setup the task
 * \param _taskName              Name of the task
 * \param _segmentName           Name of the segment for the task
 * \param _axes                  The axes used for the task
 * \param _stiffness             Stiffness constant for task
 * \param _damping               Damping constant for task
 */
gOcraSegPoseTaskManager::gOcraSegPoseTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, const std::string& _segmentName, ocra::ECartesianDof _axes, double _stiffness, double _damping)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName), segmentName(_segmentName), axes(_axes)
{
    _init(Eigen::Displacementd::Identity(), _stiffness, _damping);
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
 * \param _poseDes              Initial pose for task
 */
gOcraSegPoseTaskManager::gOcraSegPoseTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, const std::string& _segmentName, ocra::ECartesianDof _axes, double _stiffness, double _damping, const Eigen::Displacementd& _poseDes)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName), segmentName(_segmentName), axes(_axes)
{
    _init(Eigen::Displacementd::Identity(), _stiffness, _damping);
    setState(_poseDes);
}

/** Initializer function for the constructor, sets up the frames, parameters, controller and task
 *
 */
void gOcraSegPoseTaskManager::_init(const Eigen::Displacementd& _ref_LocalFrame, double _stiffness, double _damping)
{
    featFrame = new ocra::SegmentFrame(name + ".SegmentFrame", model, model.SegmentName(segmentName), _ref_LocalFrame);
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::DisplacementFeature(name + ".DisplacementFeature", *featFrame, axes);
    featDes = new ocra::DisplacementFeature(name + ".DisplacementFeature_Des", *featDesFrame, axes);

    task = &(ctrl.createGHCJTTask(name, *feat, *featDes));

    ctrl.addTask(*task);

    featDesFrame->setPosition(Eigen::Displacementd::Identity());
    featDesFrame->setVelocity(Eigen::Twistd::Zero());
    featDesFrame->setAcceleration(Eigen::Twistd::Zero());

    task->activateAsObjective();
    task->setStiffness(_stiffness);
    task->setDamping(_damping);


    // Set the desired state to the current pose of the segment with 0 vel or acc
    setState(model.getSegmentPosition(model.getSegmentIndex(segmentName)));
}

/** Sets the pose for the task, both the translational and rotational components
 *
 * \param pose                  Vector for desired position
 */
void gOcraSegPoseTaskManager::setState(const Eigen::Displacementd& pose)
{
    setState(pose, Eigen::Twistd::Zero(), Eigen::Twistd::Zero());
}

/** Sets the pose, velocity and acceleration for the task, both the translational and rotational components
 *
 * \param pose                  Desired pose
 * \param velocity              Desired velocity
 * \param acceleration          Desired acceleration
 */
void gOcraSegPoseTaskManager::setState(const Eigen::Displacementd& pose, const Eigen::Twistd& velocity, const Eigen::Twistd& acceleration)
{
    featDesFrame->setPosition(pose);
    featDesFrame->setVelocity(velocity);
    featDesFrame->setAcceleration(acceleration);
}


/** Sets the stiffness for this task
 *
 * \param stiffness             Desired stiffness
 */
void gOcraSegPoseTaskManager::setStiffness(double stiffness)
{
    task->setStiffness(stiffness);
}

/** Gets the stiffness constant for this task
 *
 *  \return                     The stiffness for this task
 */
double gOcraSegPoseTaskManager::getStiffness()
{
    Eigen::MatrixXd K = task->getStiffness();
    return K(0, 0);
}

/** Sets the damping for this task
 *
 * \param damping               Desired damping
 */
void gOcraSegPoseTaskManager::setDamping(double damping)
{
    task->setDamping(damping);
}

/** Gets the damping constant for this task
 *
 *  \return                     The damping for this task
 */
double gOcraSegPoseTaskManager::getDamping()
{
    Eigen::MatrixXd C = task->getDamping();
    return C(0, 0);
}

/** Gets the error for this task (COM position error)
 *
 */
Eigen::VectorXd gOcraSegPoseTaskManager::getTaskError()
{
    return task->getError();
}


/** Activates the task
 *
 */
void gOcraSegPoseTaskManager::activate()
{
    task->activateAsObjective();
}

/** Deactivates the task
 *
 */
void gOcraSegPoseTaskManager::deactivate()
{
    task->deactivate();
}

}

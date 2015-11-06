#include "gocra/Tasks/Managers/gOcraSegOrientationTaskManager.h"

namespace gocra
{

/** Base constructor
 *
 * \param _ctrl                 GHCJTController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 */
gOcraSegOrientationTaskManager::gOcraSegOrientationTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, const std::string& _segmentName, double _stiffness, double _damping)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName), segmentName(_segmentName)
{
    _init(Eigen::Rotation3d::Identity(), _stiffness, _damping);
}

/** Constructor with desired pose
 *
 * \param _ctrl                 GHCJTController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _orientationDes       Initial orientation for task
 */
gOcraSegOrientationTaskManager::gOcraSegOrientationTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, const std::string& _segmentName, double _stiffness, double _damping, const Eigen::Rotation3d& _orientationDes)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName), segmentName(_segmentName)
{
    _init(Eigen::Rotation3d::Identity(), _stiffness, _damping);
    setOrientation(_orientationDes);
}

/** Initializer function for the constructor, sets up the frames, parameters, controller and task
 *
 */
void gOcraSegOrientationTaskManager::_init(Eigen::Rotation3d _refOrientation_LocalFrame, double _stiffness, double _damping)
{
    featFrame = new ocra::SegmentFrame(name + ".SegmentFrame", model, model.SegmentName(segmentName), Eigen::Displacementd(Eigen::Vector3d::Zero(), _refOrientation_LocalFrame));
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::OrientationFeature(name + ".OrientationFeature", *featFrame);
    featDes = new ocra::OrientationFeature(name + ".OrientationFeature_Des", *featDesFrame);

    featDesFrame->setPosition(Eigen::Displacementd::Identity());
    featDesFrame->setVelocity(Eigen::Twistd::Zero());
    featDesFrame->setAcceleration(Eigen::Twistd::Zero());

    task = &(ctrl.createGHCJTTask(name, *feat, *featDes));

    ctrl.addTask(*task);

    task->activateAsObjective();
    task->setStiffness(_stiffness);
    task->setDamping(_damping);
}

/** Sets the orientation for the task, only the rotational pose
 *
 * \param orientation               Desired orientation
 */
void gOcraSegOrientationTaskManager::setOrientation(const Eigen::Rotation3d& orientation)
{
    featDesFrame->setPosition(Eigen::Displacementd(Eigen::Vector3d::Zero(), orientation));
    featDesFrame->setVelocity(Eigen::Twistd::Zero());
    featDesFrame->setAcceleration(Eigen::Twistd::Zero());
}



/** Sets the stiffness for this task
 *
 * \param stiffness             Desired stiffness
 */
void gOcraSegOrientationTaskManager::setStiffness(double stiffness)
{
    task->setStiffness(stiffness);
}

/** Gets the stiffness constant for this task
 *
 *  \return                     The stiffness for this task
 */
double gOcraSegOrientationTaskManager::getStiffness()
{
    Eigen::MatrixXd K = task->getStiffness();
    return K(0, 0);
}

/** Sets the damping for this task
 *
 * \param damping               Desired damping
 */
void gOcraSegOrientationTaskManager::setDamping(double damping)
{
    task->setDamping(damping);
}

/** Gets the damping constant for this task
 *
 *  \return                     The damping for this task
 */
double gOcraSegOrientationTaskManager::getDamping()
{
    Eigen::MatrixXd C = task->getDamping();
    return C(0, 0);
}

/** Activates the task
 *
 */
void gOcraSegOrientationTaskManager::activate()
{
    task->activateAsObjective();
}

/** Deactivates the task
 *
 */
void gOcraSegOrientationTaskManager::deactivate()
{
    task->deactivate();
}

/** Gets the error for this task (COM position error)
 *
 */
Eigen::VectorXd gOcraSegOrientationTaskManager::getTaskError()
{
    return task->getError();
}


}

#include "ocra/control/TaskManagers/SegOrientationTaskManager.h"

namespace ocra
{

/** Base constructor
 *
 * \param _ctrl                 ocra::Controller to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 */
SegOrientationTaskManager::SegOrientationTaskManager(ocra::Controller& _ctrl,
                                                                const ocra::Model& _model,
                                                                const std::string& _taskName,
                                                                const std::string& _segmentName,
                                                                double _stiffness,
                                                                double _damping,
                                                                double _weight,
                                                                bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName)
{
    _init(Eigen::Rotation3d::Identity(), _stiffness, _damping, _weight);
}

SegOrientationTaskManager::SegOrientationTaskManager(ocra::Controller& _ctrl,
                                                                const ocra::Model& _model,
                                                                const std::string& _taskName,
                                                                const std::string& _segmentName,
                                                                double _stiffness,
                                                                double _damping,
                                                                const Eigen::VectorXd& _weight,
                                                                bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName)
{
    _init(Eigen::Rotation3d::Identity(), _stiffness, _damping, _weight);
}

/** Constructor with desired pose
 *
 * \param _ctrl                 ocra::Controller to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 * \param _orientationDes       Initial orientation for task
 */
SegOrientationTaskManager::SegOrientationTaskManager(ocra::Controller& _ctrl,
                                                                const ocra::Model& _model,
                                                                const std::string& _taskName,
                                                                const std::string& _segmentName,
                                                                double _stiffness,
                                                                double _damping,
                                                                double _weight,
                                                                const Eigen::Rotation3d& _orientationDes,
                                                                bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName)
{
    _init(Eigen::Rotation3d::Identity(), _stiffness, _damping, _weight);
    setOrientation(_orientationDes);
}

SegOrientationTaskManager::SegOrientationTaskManager(ocra::Controller& _ctrl,
                                                                const ocra::Model& _model,
                                                                const std::string& _taskName,
                                                                const std::string& _segmentName,
                                                                double _stiffness,
                                                                double _damping,
                                                                const Eigen::VectorXd& _weight,
                                                                const Eigen::Rotation3d& _orientationDes,
                                                                bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName)
{
    _init(Eigen::Rotation3d::Identity(), _stiffness, _damping, _weight);
    setOrientation(_orientationDes);
}


SegOrientationTaskManager::~SegOrientationTaskManager()
{

}

/** Initializer function for the constructor, sets up the frames, parameters, controller and task
 *
 */
void SegOrientationTaskManager::_init(Eigen::Rotation3d _refOrientation_LocalFrame, double _stiffness, double _damping, double _weight)
{
    featFrame = new ocra::SegmentFrame(name + ".SegmentFrame", model, model.SegmentName(segmentName), Eigen::Displacementd(Eigen::Vector3d::Zero(), _refOrientation_LocalFrame));
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::OrientationFeature(name + ".OrientationFeature", *featFrame);
    featDes = new ocra::OrientationFeature(name + ".OrientationFeature_Des", *featDesFrame);

    featDesFrame->setPosition(Eigen::Displacementd::Identity());
    featDesFrame->setVelocity(Eigen::Twistd::Zero());
    featDesFrame->setAcceleration(Eigen::Twistd::Zero());

    task = ctrl.createTask(name, *feat, *featDes);
    task->setTaskType(ocra::Task::ACCELERATIONTASK);
    ctrl.addTask(task);

    task->activateAsObjective();
    task->setStiffness(_stiffness);
    task->setDamping(_damping);
    task->setWeight(_weight);

    setStateDimension(4); //Quaternion only.
}

void SegOrientationTaskManager::_init(Eigen::Rotation3d _refOrientation_LocalFrame, double _stiffness, double _damping, const Eigen::VectorXd& _weight)
{
    featFrame = new ocra::SegmentFrame(name + ".SegmentFrame", model, model.SegmentName(segmentName), Eigen::Displacementd(Eigen::Vector3d::Zero(), _refOrientation_LocalFrame));
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::OrientationFeature(name + ".OrientationFeature", *featFrame);
    featDes = new ocra::OrientationFeature(name + ".OrientationFeature_Des", *featDesFrame);

    featDesFrame->setPosition(Eigen::Displacementd::Identity());
    featDesFrame->setVelocity(Eigen::Twistd::Zero());
    featDesFrame->setAcceleration(Eigen::Twistd::Zero());

    task = ctrl.createTask(name, *feat, *featDes);
    task->setTaskType(ocra::Task::ACCELERATIONTASK);
    ctrl.addTask(task);

    task->activateAsObjective();
    task->setStiffness(_stiffness);
    task->setDamping(_damping);
    task->setWeight(_weight);

    setStateDimension(4); //Quaternion only.
}

/** Sets the orientation for the task, only the rotational pose
 *
 * \param orientation               Desired orientation
 */
void SegOrientationTaskManager::setOrientation(const Eigen::Rotation3d& orientation)
{
    featDesFrame->setPosition(Eigen::Displacementd(Eigen::Vector3d::Zero(), orientation));
    featDesFrame->setVelocity(Eigen::Twistd::Zero());
    featDesFrame->setAcceleration(Eigen::Twistd::Zero());


    eigenDesiredStateVector << orientation.w(), orientation.x(), orientation.y(), orientation.z();
    updateDesiredStateVector(eigenDesiredStateVector.data());

}

void SegOrientationTaskManager::setDesiredState()
{
    setOrientation(Eigen::Rotation3d(newDesiredStateVector[0], newDesiredStateVector[1], newDesiredStateVector[2], newDesiredStateVector[3]));
}


/** Sets the weight constant for this task
 *
 *  \param weight               Desired weight value
 */
void SegOrientationTaskManager::setWeight(double weight)
{
    task->setWeight(weight);
}

void SegOrientationTaskManager::setWeight(const Eigen::VectorXd& weight)
{
    task->setWeight(weight);
}

/** Gets the weight constant for this task
 *
 *  \return                     The weight for this task
 */
Eigen::VectorXd SegOrientationTaskManager::getWeight()
{
    return task->getWeight();
}

/** Sets the stiffness for this task
 *
 * \param stiffness             Desired stiffness
 */
void SegOrientationTaskManager::setStiffness(double stiffness)
{
    task->setStiffness(stiffness);
}

/** Gets the stiffness constant for this task
 *
 *  \return                     The stiffness for this task
 */
double SegOrientationTaskManager::getStiffness()
{
    Eigen::MatrixXd K = task->getStiffness();
    return K(0, 0);
}

/** Sets the damping for this task
 *
 * \param damping               Desired damping
 */
void SegOrientationTaskManager::setDamping(double damping)
{
    task->setDamping(damping);
}

/** Gets the damping constant for this task
 *
 *  \return                     The damping for this task
 */
double SegOrientationTaskManager::getDamping()
{
    Eigen::MatrixXd C = task->getDamping();
    return C(0, 0);
}

/** Gets the error for this task (COM position error)
 *
 */
Eigen::VectorXd SegOrientationTaskManager::getTaskError()
{
    return task->getError();
}


const double* SegOrientationTaskManager::getCurrentState()
{
    eigenCurrentStateVector << featFrame->getPosition().getRotation().w(), featFrame->getPosition().getRotation().x(), featFrame->getPosition().getRotation().y(), featFrame->getPosition().getRotation().z();

    return eigenCurrentStateVector.data();
}


std::string SegOrientationTaskManager::getTaskManagerType()
{
    return "SegOrientationTaskManager";
}


bool SegOrientationTaskManager::checkIfActivated()
{
    return task->isActiveAsObjective();
}

}

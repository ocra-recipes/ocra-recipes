#include "ocra/control/TaskManagers/SegPoseTaskManager.h"

namespace ocra
{

/** Base constructor
 *
 * \param _ctrl                  ocra::Controller to connect to
 * \param _model                 ocra model to setup the task
 * \param _taskName              Name of the task
 * \param _segmentName           Name of the segment for the task
 * \param _axes                  The axes used for the task
 * \param _stiffness             Stiffness constant for task
 * \param _damping               Damping constant for task
 * \param _weight                Weight constant for task
 */
SegPoseTaskManager::SegPoseTaskManager(ocra::Controller& _ctrl,
                                                    const ocra::Model& _model,
                                                    const std::string& _taskName,
                                                    const std::string& _segmentName,
                                                    ocra::ECartesianDof _axes,
                                                    double _stiffness,
                                                    double _damping,
                                                    double _weight,
                                                    bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName), axes(_axes)
{
    _init(Eigen::Displacementd::Identity(), _stiffness, _damping, _weight);
}

SegPoseTaskManager::SegPoseTaskManager(ocra::Controller& _ctrl,
                                                    const ocra::Model& _model,
                                                    const std::string& _taskName,
                                                    const std::string& _segmentName,
                                                    ocra::ECartesianDof _axes,
                                                    double _stiffness,
                                                    double _damping,
                                                    const Eigen::VectorXd& _weight,
                                                    bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName), axes(_axes)
{
    _init(Eigen::Displacementd::Identity(), _stiffness, _damping, _weight);
}

/** Constructor with offset frame
 *
 * \param _ctrl                  ocra::Controller to connect to
 * \param _model                 ocra model to setup the task
 * \param _taskName              Name of the task
 * \param _segmentName           Name of the segment for the task
 * \param _segFrame_Local        The point to track the task in local frame
 * \param _axes                  The axes used for the task
 * \param _stiffness             Stiffness constant for task
 * \param _damping               Damping constant for task
 * \param _weight                Weight constant for task
 */
SegPoseTaskManager::SegPoseTaskManager(ocra::Controller& _ctrl,
                                                    const ocra::Model& _model,
                                                    const std::string& _taskName,
                                                    const std::string& _segmentName,
                                                    const Eigen::Displacementd& _segFrame_Local,
                                                    ocra::ECartesianDof _axes,
                                                    double _stiffness,
                                                    double _damping,
                                                    double _weight,
                                                    bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName), axes(_axes)
{
    _init(_segFrame_Local, _stiffness, _damping, _weight);
}

SegPoseTaskManager::SegPoseTaskManager(ocra::Controller& _ctrl,
                                                    const ocra::Model& _model,
                                                    const std::string& _taskName,
                                                    const std::string& _segmentName,
                                                    const Eigen::Displacementd& _segFrame_Local,
                                                    ocra::ECartesianDof _axes,
                                                    double _stiffness,
                                                    double _damping,
                                                    const Eigen::VectorXd& _weight,
                                                    bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName), axes(_axes)
{
    _init(_segFrame_Local, _stiffness, _damping, _weight);
}

/** Constructor with desired pose
 *
 * \param _ctrl                 ocra::Controller to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 * \param _poseDes              Initial pose for task
 */
SegPoseTaskManager::SegPoseTaskManager(ocra::Controller& _ctrl,
                                                    const ocra::Model& _model,
                                                    const std::string& _taskName,
                                                    const std::string& _segmentName,
                                                    ocra::ECartesianDof _axes,
                                                    double _stiffness,
                                                    double _damping,
                                                    double _weight,
                                                    const Eigen::Displacementd& _poseDes,
                                                    bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName), axes(_axes)
{
    _init(Eigen::Displacementd::Identity(), _stiffness, _damping, _weight);
    setState(_poseDes);
}

SegPoseTaskManager::SegPoseTaskManager(ocra::Controller& _ctrl,
                                                    const ocra::Model& _model,
                                                    const std::string& _taskName,
                                                    const std::string& _segmentName,
                                                    ocra::ECartesianDof _axes,
                                                    double _stiffness,
                                                    double _damping,
                                                    const Eigen::VectorXd& _weight,
                                                    const Eigen::Displacementd& _poseDes,
                                                    bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName), axes(_axes)
{
    _init(Eigen::Displacementd::Identity(), _stiffness, _damping, _weight);
    setState(_poseDes);
}

/** Constructor with desired pose
 *
 * \param _ctrl                 ocra::Controller to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _segFrame_Local        The point to track the task in local frame
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 * \param _poseDes              Initial pose for task
 */
SegPoseTaskManager::SegPoseTaskManager(ocra::Controller& _ctrl,
                                                    const ocra::Model& _model,
                                                    const std::string& _taskName,
                                                    const std::string& _segmentName,
                                                    const Eigen::Displacementd& _segFrame_Local,
                                                    ocra::ECartesianDof _axes,
                                                    double _stiffness,
                                                    double _damping,
                                                    double _weight,
                                                    const Eigen::Displacementd& _poseDes,
                                                    bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName), axes(_axes)
{
    _init(_segFrame_Local, _stiffness, _damping, _weight);
    setState(_poseDes);
}

SegPoseTaskManager::SegPoseTaskManager(ocra::Controller& _ctrl,
                                                    const ocra::Model& _model,
                                                    const std::string& _taskName,
                                                    const std::string& _segmentName,
                                                    const Eigen::Displacementd& _segFrame_Local,
                                                    ocra::ECartesianDof _axes,
                                                    double _stiffness,
                                                    double _damping,
                                                    const Eigen::VectorXd& _weight,
                                                    const Eigen::Displacementd& _poseDes,
                                                    bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName), axes(_axes)
{
    _init(_segFrame_Local, _stiffness, _damping, _weight);
    setState(_poseDes);
}


SegPoseTaskManager::~SegPoseTaskManager()
{

}


/** Initializer function for the constructor, sets up the frames, parameters, controller and task
 *
 */
void SegPoseTaskManager::_init(const Eigen::Displacementd& _ref_LocalFrame, double _stiffness, double _damping, double _weight)
{
    featFrame = new ocra::SegmentFrame(name + ".SegmentFrame", model, model.SegmentName(segmentName), _ref_LocalFrame);
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::DisplacementFeature(name + ".DisplacementFeature", *featFrame, axes);
    featDes = new ocra::DisplacementFeature(name + ".DisplacementFeature_Des", *featDesFrame, axes);

    task = ctrl.createTask(name, *feat, *featDes);
    task->setTaskType(ocra::Task::ACCELERATIONTASK);
    ctrl.addTask(task);

    featDesFrame->setPosition(Eigen::Displacementd::Identity());
    featDesFrame->setVelocity(Eigen::Twistd::Zero());
    featDesFrame->setAcceleration(Eigen::Twistd::Zero());

    task->activateAsObjective();
    task->setStiffness(_stiffness);
    task->setDamping(_damping);
    task->setWeight(_weight);

    setStateDimension(7+6+6, 7); // dispd = 7 + 2*twistd = 6

    // Set the desired state to the current pose of the segment with 0 vel or acc
    setState(model.getSegmentPosition(model.getSegmentIndex(model.SegmentName(segmentName))));
}

void SegPoseTaskManager::_init(const Eigen::Displacementd& _ref_LocalFrame, double _stiffness, double _damping, const Eigen::VectorXd& _weight)
{
    featFrame = new ocra::SegmentFrame(name + ".SegmentFrame", model, model.SegmentName(segmentName), _ref_LocalFrame);
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::DisplacementFeature(name + ".DisplacementFeature", *featFrame, axes);
    featDes = new ocra::DisplacementFeature(name + ".DisplacementFeature_Des", *featDesFrame, axes);

    task = ctrl.createTask(name, *feat, *featDes);
    task->setTaskType(ocra::Task::ACCELERATIONTASK);
    ctrl.addTask(task);

    featDesFrame->setPosition(Eigen::Displacementd::Identity());
    featDesFrame->setVelocity(Eigen::Twistd::Zero());
    featDesFrame->setAcceleration(Eigen::Twistd::Zero());

    task->activateAsObjective();
    task->setStiffness(_stiffness);
    task->setDamping(_damping);
    task->setWeight(_weight);

    setStateDimension(7+6+6, 7); // dispd = 7 + 2*twistd = 6

    // Set the desired state to the current pose of the segment with 0 vel or acc
    setState(model.getSegmentPosition(model.getSegmentIndex(segmentName)));

}


/** Sets the pose for the task, both the translational and rotational components
 *
 * \param pose                  Vector for desired position
 */
void SegPoseTaskManager::setState(const Eigen::Displacementd& pose)
{
    setState(pose, Eigen::Twistd::Zero(), Eigen::Twistd::Zero());
}

/** Sets the pose, velocity and acceleration for the task, both the translational and rotational components
 *
 * \param pose                  Desired pose
 * \param velocity              Desired velocity
 * \param acceleration          Desired acceleration
 */
void SegPoseTaskManager::setState(const Eigen::Displacementd& pose, const Eigen::Twistd& velocity, const Eigen::Twistd& acceleration)
{
    featDesFrame->setPosition(pose);
    featDesFrame->setVelocity(velocity);
    featDesFrame->setAcceleration(acceleration);

    eigenDesiredStateVector << featDesFrame->getPosition().getTranslation(), featDesFrame->getPosition().getRotation().w(), featDesFrame->getPosition().getRotation().x(), featDesFrame->getPosition().getRotation().y(), featDesFrame->getPosition().getRotation().z(),  featDesFrame->getVelocity().getAngularVelocity(), featDesFrame->getVelocity().getLinearVelocity(), featDesFrame->getAcceleration().getAngularVelocity(), featDesFrame->getAcceleration().getLinearVelocity();

    updateDesiredStateVector(eigenDesiredStateVector.data());
}

void SegPoseTaskManager::setDesiredState()
{
    double * vectorStart = &newDesiredStateVector.front();
    Eigen::VectorXd newPosition = Eigen::VectorXd::Map(vectorStart, 7);
    Eigen::VectorXd newVelocity = Eigen::VectorXd::Map(vectorStart + 7, 6);
    Eigen::VectorXd newAcceleration = Eigen::VectorXd::Map(vectorStart + (7+6), 6);

    Eigen::Displacementd _newPosition = Eigen::Displacementd(newPosition(0), newPosition(1), newPosition(2),newPosition(3),newPosition(4),newPosition(5),newPosition(6));
    Eigen::Twistd _newVelocity = Eigen::Twistd(newVelocity.array());
    Eigen::Twistd _newAcceleration = Eigen::Twistd(newAcceleration.array());

    setState(_newPosition, _newVelocity, _newAcceleration);
}

/** Sets the weight constant for this task
 *
 *  \param weight               Desired weight value
 */
void SegPoseTaskManager::setWeight(double weight)
{
    task->setWeight(weight);
}

void SegPoseTaskManager::setWeight(const Eigen::VectorXd& weight)
{
    task->setWeight(weight);
}

/** Gets the weight constant for this task
 *
 *  \return                     The weight for this task
 */
Eigen::VectorXd SegPoseTaskManager::getWeight()
{
    return task->getWeight();
}

/** Sets the stiffness for this task
 *
 * \param stiffness             Desired stiffness
 */
void SegPoseTaskManager::setStiffness(double stiffness)
{
    task->setStiffness(stiffness);
}

/** Gets the stiffness constant for this task
 *
 *  \return                     The stiffness for this task
 */
double SegPoseTaskManager::getStiffness()
{
    Eigen::MatrixXd K = task->getStiffness();
    return K(0, 0);
}

/** Sets the damping for this task
 *
 * \param damping               Desired damping
 */
void SegPoseTaskManager::setDamping(double damping)
{
    task->setDamping(damping);
}

/** Gets the damping constant for this task
 *
 *  \return                     The damping for this task
 */
double SegPoseTaskManager::getDamping()
{
    Eigen::MatrixXd C = task->getDamping();
    return C(0, 0);
}

/** Gets the error for this task (COM position error)
 *
 */
Eigen::VectorXd SegPoseTaskManager::getTaskError()
{
    return task->getError();
}


/** Activates the task
 *
 */
void SegPoseTaskManager::activate()
{
    task->activateAsObjective();
}

/** Deactivates the task
 *
 */
void SegPoseTaskManager::deactivate()
{
    task->deactivate();
}

const double* SegPoseTaskManager::getCurrentState()
{
    eigenCurrentStateVector << featFrame->getPosition().getTranslation(), featFrame->getPosition().getRotation().w(), featFrame->getPosition().getRotation().x(), featFrame->getPosition().getRotation().y(), featFrame->getPosition().getRotation().z(),  featFrame->getVelocity().getAngularVelocity(), featFrame->getVelocity().getLinearVelocity(), featFrame->getAcceleration().getAngularVelocity(), featFrame->getAcceleration().getLinearVelocity();

    return eigenCurrentStateVector.data();
}


std::string SegPoseTaskManager::getTaskManagerType()
{
    return "SegPoseTaskManager";
}


bool SegPoseTaskManager::checkIfActivated()
{
    return task->isActiveAsObjective();
}



Eigen::Displacementd SegPoseTaskManager::getTaskFrameDisplacement()
{
    return featFrame->getPosition();
}

Eigen::Twistd SegPoseTaskManager::getTaskFrameVelocity()
{
    return featFrame->getVelocity();
}

Eigen::Twistd SegPoseTaskManager::getTaskFrameAcceleration()
{
    return featFrame->getAcceleration();
}

Eigen::Vector3d SegPoseTaskManager::getTaskFramePosition()
{
    return getTaskFrameDisplacement().getTranslation();
}

Eigen::Rotation3d SegPoseTaskManager::getTaskFrameOrientation()
{
    return getTaskFrameDisplacement().getRotation();
}

Eigen::Vector3d SegPoseTaskManager::getTaskFrameLinearVelocity()
{
    return getTaskFrameVelocity().getLinearVelocity();
}

Eigen::Vector3d SegPoseTaskManager::getTaskFrameAngularVelocity()
{
    return getTaskFrameVelocity().getAngularVelocity();
}

Eigen::Vector3d SegPoseTaskManager::getTaskFrameLinearAcceleration()
{
    return getTaskFrameAcceleration().getLinearVelocity();
}

Eigen::Vector3d SegPoseTaskManager::getTaskFrameAngularAcceleration()
{
    return getTaskFrameAcceleration().getAngularVelocity();
}

}

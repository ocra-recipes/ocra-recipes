#include "ocra/control/TaskManagers/SegCartesianTaskManager.h"

namespace ocra
{

/** Base constructor
 *
 * \param _ctrl                 ocra::Controller to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 */
SegCartesianTaskManager::SegCartesianTaskManager(ocra::Controller& _ctrl,
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
    _init(Eigen::Vector3d::Zero(), _stiffness, _damping, _weight);
}

SegCartesianTaskManager::SegCartesianTaskManager(ocra::Controller& _ctrl,
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
    _init(Eigen::Vector3d::Zero(), _stiffness, _damping, _weight);
}

/** Constructor with the specifying the point of reference on the segment to track
 *
 * \param _ctrl                 ocra::Controller to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _segPoint_Local       The point to track the task in local frame
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 */
SegCartesianTaskManager::SegCartesianTaskManager(ocra::Controller& _ctrl,
                                                            const ocra::Model& _model,
                                                            const std::string& _taskName,
                                                            const std::string& _segmentName,
                                                            const Eigen::Vector3d& _segPoint_Local,
                                                            ocra::ECartesianDof _axes,
                                                            double _stiffness,
                                                            double _damping,
                                                            double _weight,
                                                            bool _usesYarpPorts)

    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName), axes(_axes)
{
    _init(_segPoint_Local, _stiffness, _damping, _weight);
}

SegCartesianTaskManager::SegCartesianTaskManager(ocra::Controller& _ctrl,
                                                            const ocra::Model& _model,
                                                            const std::string& _taskName,
                                                            const std::string& _segmentName,
                                                            const Eigen::Vector3d& _segPoint_Local,
                                                            ocra::ECartesianDof _axes,
                                                            double _stiffness,
                                                            double _damping,
                                                            const Eigen::VectorXd& _weight,
                                                            bool _usesYarpPorts)

    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName), axes(_axes)
{
    _init(_segPoint_Local, _stiffness, _damping, _weight);
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
 * \param _poseDes              Initial pose (cartesian) for task
 */
SegCartesianTaskManager::SegCartesianTaskManager(ocra::Controller& _ctrl,
                                                            const ocra::Model& _model,
                                                            const std::string& _taskName,
                                                            const std::string& _segmentName,
                                                            ocra::ECartesianDof _axes,
                                                            double _stiffness,
                                                            double _damping,
                                                            double _weight,
                                                            const Eigen::Vector3d& _poseDes,
                                                            bool _usesYarpPorts)

    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName), axes(_axes)
{
    _init(Eigen::Vector3d::Zero(), _stiffness, _damping, _weight);
    // Have no idea this wrapper needs to be done
    setState(_poseDes);
}

SegCartesianTaskManager::SegCartesianTaskManager(ocra::Controller& _ctrl,
                                                            const ocra::Model& _model,
                                                            const std::string& _taskName,
                                                            const std::string& _segmentName,
                                                            ocra::ECartesianDof _axes,
                                                            double _stiffness,
                                                            double _damping,
                                                            const Eigen::VectorXd& _weight,
                                                            const Eigen::Vector3d& _poseDes,
                                                            bool _usesYarpPorts)

    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName), axes(_axes)
{
    _init(Eigen::Vector3d::Zero(), _stiffness, _damping, _weight);
    // Have no idea this wrapper needs to be done
    setState(_poseDes);
}
/**
 * Constructor with both point on segment and desired pose
 *
 * \param _ctrl                 ocra::Controller to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 * \param _poseDes              Initial pose (cartesian) for task
 */
SegCartesianTaskManager::SegCartesianTaskManager(ocra::Controller& _ctrl,
                                                            const ocra::Model& _model,
                                                            const std::string& _taskName,
                                                            const std::string& _segmentName,
                                                            const Eigen::Vector3d& _segPoint_Local,
                                                            ocra::ECartesianDof _axes,
                                                            double _stiffness,
                                                            double _damping,
                                                            double _weight,
                                                            const Eigen::Vector3d& _poseDes,
                                                            bool _usesYarpPorts)

    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName), axes(_axes)
{
    _init(_segPoint_Local, _stiffness, _damping, _weight);
    setState(_poseDes);
}

SegCartesianTaskManager::SegCartesianTaskManager(ocra::Controller& _ctrl,
                                                            const ocra::Model& _model,
                                                            const std::string& _taskName,
                                                            const std::string& _segmentName,
                                                            const Eigen::Vector3d& _segPoint_Local,
                                                            ocra::ECartesianDof _axes,
                                                            double _stiffness,
                                                            double _damping,
                                                            const Eigen::VectorXd& _weight,
                                                            const Eigen::Vector3d& _poseDes,
                                                            bool _usesYarpPorts)

    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName), axes(_axes)
{
    _init(_segPoint_Local, _stiffness, _damping, _weight);
    setState(_poseDes);
}

SegCartesianTaskManager::~SegCartesianTaskManager()
{

}

/** Initializer function for the constructor, sets up the frames, parameters, controller and task
 *
 */
void SegCartesianTaskManager::_init(const Eigen::Vector3d& _taskPoint_LocalFrame, double _stiffness, double _damping, double _weight)
{
    featFrame = new ocra::SegmentFrame(name + ".SegmentFrame", model, model.SegmentName(segmentName), Eigen::Displacementd(_taskPoint_LocalFrame));
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::PositionFeature(name + ".PositionFeature", *featFrame, axes);
    featDes = new ocra::PositionFeature(name + ".PositionFeature_Des", *featDesFrame, axes);

    task = &ctrl.createTask(name, *feat, *featDes);
    task->setTaskType(ocra::Task::ACCELERATIONTASK);
    ctrl.addTask(*task);

    task->activateAsObjective();
    task->setStiffness(_stiffness);
    task->setDamping(_damping);
    task->setWeight(_weight);

    setStateDimension(9, 3); //3 dof for pos vel and acc
    // Set the desired state to the current position of the segment with 0 vel or acc
    setState(model.getSegmentPosition(model.getSegmentIndex(segmentName)).getTranslation());
}

void SegCartesianTaskManager::_init(const Eigen::Vector3d& _taskPoint_LocalFrame, double _stiffness, double _damping, const Eigen::VectorXd& _weight)
{
    featFrame = new ocra::SegmentFrame(name + ".SegmentFrame", model, model.SegmentName(segmentName), Eigen::Displacementd(_taskPoint_LocalFrame));
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::PositionFeature(name + ".PositionFeature", *featFrame, axes);
    featDes = new ocra::PositionFeature(name + ".PositionFeature_Des", *featDesFrame, axes);

    task = &ctrl.createTask(name, *feat, *featDes);
    task->setTaskType(ocra::Task::ACCELERATIONTASK);
    ctrl.addTask(*task);

    task->activateAsObjective();
    task->setStiffness(_stiffness);
    task->setDamping(_damping);
    task->setWeight(_weight);

    setStateDimension(9, 3); //3 dof for pos vel and acc
    // Set the desired state to the current position of the segment with 0 vel or acc
    setState(model.getSegmentPosition(model.getSegmentIndex(segmentName)).getTranslation());
}
/** Sets the position for the task, only the translational position
 *
 * \param position                  Vector for desired position
 */
void SegCartesianTaskManager::setState(const Eigen::Vector3d& position)
{
    setState(position, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

}

/** Sets the position, linear velocity and linear acceleration for the task
 *
 * \param position                  Vector for desired position
 * \param velocity                  Vector for desired linear velocity
 * \param acceleration              Vector for desired linear acceleration
 */
void SegCartesianTaskManager::setState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration)
{
    featDesFrame->setPosition(Eigen::Displacementd(position));
    featDesFrame->setVelocity(Eigen::Twistd(0.0, 0.0, 0.0, velocity(0), velocity(1), velocity(2)) );
    featDesFrame->setAcceleration(Eigen::Twistd(0.0, 0.0, 0.0, acceleration(0), acceleration(1), acceleration(2)) );

    eigenDesiredStateVector << position, velocity, acceleration;
    updateDesiredStateVector(eigenDesiredStateVector.data());
}

void SegCartesianTaskManager::setDesiredState()
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
void SegCartesianTaskManager::setWeight(double weight)
{
    task->setWeight(weight);
}

void SegCartesianTaskManager::setWeight(const Eigen::VectorXd& weight)
{
    task->setWeight(weight);
}

/** Gets the weight constant for this task
 *
 *  \return                     The weight for this task
 */
Eigen::VectorXd SegCartesianTaskManager::getWeight()
{
    return task->getWeight();
}

/** Sets the stiffness for this task
 *
 * \param stiffness             Desired stiffness
 */
void SegCartesianTaskManager::setStiffness(double stiffness)
{
    task->setStiffness(stiffness);
}

/** Gets the stiffness constant for this task
 *
 *  \return                     The stiffness for this task
 */
double SegCartesianTaskManager::getStiffness()
{
    Eigen::MatrixXd K = task->getStiffness();
    return K(0, 0);
}

/** Sets the damping for this task
 *
 * \param damping               Desired damping
 */
void SegCartesianTaskManager::setDamping(double damping)
{
    task->setDamping(damping);
}

/** Gets the damping constant for this task
 *
 *  \return                     The damping for this task
 */
double SegCartesianTaskManager::getDamping()
{
    Eigen::MatrixXd C = task->getDamping();
    return C(0, 0);
}

/** Activates the task
 *
 */
void SegCartesianTaskManager::activate()
{
    task->activateAsObjective();
}

/** Deactivates the task
 *
 */
void SegCartesianTaskManager::deactivate()
{
    task->deactivate();
}

/** Gets the error for this task (Cartesian error)
 *
 */
Eigen::VectorXd SegCartesianTaskManager::getTaskError()
{
    return task->getError();
}


const double* SegCartesianTaskManager::getCurrentState()
{
    eigenCurrentStateVector << featFrame->getPosition().getTranslation(), featFrame->getVelocity().getLinearVelocity(), featFrame->getAcceleration().getLinearVelocity();
    return eigenCurrentStateVector.data();
}


std::string SegCartesianTaskManager::getTaskManagerType()
{
    return "SegCartesianTaskManager";
}


bool SegCartesianTaskManager::checkIfActivated()
{
    return task->isActiveAsObjective();
}




Eigen::Displacementd SegCartesianTaskManager::getTaskFrameDisplacement()
{
    return featFrame->getPosition();
}

Eigen::Twistd SegCartesianTaskManager::getTaskFrameVelocity()
{
    return featFrame->getVelocity();
}

Eigen::Twistd SegCartesianTaskManager::getTaskFrameAcceleration()
{
    return featFrame->getAcceleration();
}

Eigen::Vector3d SegCartesianTaskManager::getTaskFramePosition()
{
    return getTaskFrameDisplacement().getTranslation();
}

Eigen::Rotation3d SegCartesianTaskManager::getTaskFrameOrientation()
{
    return getTaskFrameDisplacement().getRotation();
}

Eigen::Vector3d SegCartesianTaskManager::getTaskFrameLinearVelocity()
{
    return getTaskFrameVelocity().getLinearVelocity();
}

Eigen::Vector3d SegCartesianTaskManager::getTaskFrameAngularVelocity()
{
    return getTaskFrameVelocity().getAngularVelocity();
}

Eigen::Vector3d SegCartesianTaskManager::getTaskFrameLinearAcceleration()
{
    return getTaskFrameAcceleration().getLinearVelocity();
}

Eigen::Vector3d SegCartesianTaskManager::getTaskFrameAngularAcceleration()
{
    return getTaskFrameAcceleration().getAngularVelocity();
}

}

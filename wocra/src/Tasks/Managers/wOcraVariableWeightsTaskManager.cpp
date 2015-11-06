#include "wocra/Tasks/Managers/wOcraVariableWeightsTaskManager.h"

namespace wocra
{

/** Base constructor
 *
 * \param _ctrl                 wOcraController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 */
wOcraVariableWeightsTaskManager::wOcraVariableWeightsTaskManager(wOcraController& _ctrl,
                                                                const wOcraModel& _model,
                                                                const std::string& _taskName,
                                                                const std::string& _segmentName,
                                                                double _stiffness,
                                                                double _damping,
                                                                Eigen::Vector3d _weight,
                                                                bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts),
        segmentName(_segmentName)
{
    _init(Eigen::Vector3d::Zero(), _stiffness, _damping, _weight);
}

/** Constructor with the specifying the point of reference on the segment to track
 *
 * \param _ctrl                 wOcraController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _segPoint_Local       The point to track the task in local frame
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 */
wOcraVariableWeightsTaskManager::wOcraVariableWeightsTaskManager(wOcraController& _ctrl,
                                                                const wOcraModel& _model,
                                                                const std::string& _taskName,
                                                                const std::string& _segmentName,
                                                                const Eigen::Vector3d& _segPoint_Local,
                                                                double _stiffness,
                                                                double _damping,
                                                                Eigen::Vector3d _weight,
                                                                bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts),
        segmentName(_segmentName)
{
    _init(_segPoint_Local, _stiffness, _damping, _weight);
}

/** Constructor with desired pose
 *
 * \param _ctrl                 wOcraController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 * \param _poseDes              Initial pose (cartesian) for task
 */
wOcraVariableWeightsTaskManager::wOcraVariableWeightsTaskManager(wOcraController& _ctrl,
                                                                const wOcraModel& _model,
                                                                const std::string& _taskName,
                                                                const std::string& _segmentName,
                                                                double _stiffness,
                                                                double _damping,
                                                                Eigen::Vector3d _weight,
                                                                const Eigen::Vector3d& _poseDes,
                                                                bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts),
        segmentName(_segmentName)
{
    _init(Eigen::Vector3d::Zero(), _stiffness, _damping, _weight);
    setState(_poseDes);
}

/**
 * Constructor with both point on segment and desired pose
 *
 * \param _ctrl                 wOcraController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 * \param _poseDes              Initial pose (cartesian) for task
 */
wOcraVariableWeightsTaskManager::wOcraVariableWeightsTaskManager(wOcraController& _ctrl,
                                                                const wOcraModel& _model,
                                                                const std::string& _taskName,
                                                                const std::string& _segmentName,
                                                                const Vector3d& _segPoint_Local,
                                                                double _stiffness,
                                                                double _damping,
                                                                Eigen::Vector3d _weight,
                                                                const Eigen::Vector3d& _poseDes,
                                                                bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts),
        segmentName(_segmentName)
{
    _init(_segPoint_Local, _stiffness, _damping, _weight);
    setState(_poseDes);
}


wOcraVariableWeightsTaskManager::~wOcraVariableWeightsTaskManager()
{

}



/** Initializer function for the constructor, sets up the frames, parameters, controller and task
 *
 */
void wOcraVariableWeightsTaskManager::_init(const Eigen::Vector3d& _taskPoint_LocalFrame, double _stiffness, double _damping, Eigen::Vector3d _weight)
{
    nDoF = 3;
    axes.resize(nDoF);
    axesLabels.resize(nDoF);
    tasks.resize(nDoF);

    axes[0] = ocra::X;
    axes[1] = ocra::Y;
    axes[2] = ocra::Z;

    axesLabels[0] = "X";
    axesLabels[1] = "Y";
    axesLabels[2] = "Z";

    featFrame = new ocra::SegmentFrame(name + ".SegmentFrame", model, model.SegmentName(segmentName), Eigen::Displacementd(_taskPoint_LocalFrame));
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);


    for (int i=0; i<nDoF; i++)
    {
        ocra::PositionFeature* feat = new ocra::PositionFeature(name + ".PositionFeature", *featFrame, axes[i]);
        ocra::PositionFeature* featDes = new ocra::PositionFeature(name + ".PositionFeature_Des", *featDesFrame, axes[i]);

        tasks[i] = &(ctrl.createwOcraTask(name+axesLabels[i], *feat, *featDes));
        tasks[i]->initAsAccelerationTask();
        ctrl.addTask(*tasks[i]);

        tasks[i]->activateAsObjective();
        tasks[i]->setStiffness(_stiffness);
        tasks[i]->setDamping(_damping);
        tasks[i]->setWeight(_weight(i));

    }

    // Set the desired state to the current position of the segment with 0 vel or acc
    setState(model.getSegmentPosition(model.getSegmentIndex(segmentName)).getTranslation());
}

/** Sets the position for the task, only the translational position
 *
 * \param position                  Vector for desired position
 */
void wOcraVariableWeightsTaskManager::setState(const Eigen::Vector3d& position)
{
    setState(position, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

/** Sets the position, linear velocity and linear acceleration for the task
 *
 * \param position                  Vector for desired position
 * \param velocity                  Vector for desired linear velocity
 * \param acceleration              Vector for desired linear acceleration
 */
void wOcraVariableWeightsTaskManager::setState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration)
{
    featDesFrame->setPosition(Eigen::Displacementd(position));
    featDesFrame->setVelocity(Eigen::Twistd(0.0, 0.0, 0.0, velocity(0), velocity(1), velocity(2)) );
    featDesFrame->setAcceleration(Eigen::Twistd(0.0, 0.0, 0.0, acceleration(0), acceleration(1), acceleration(2)) );
}


/** Sets the weight constant for this task
 *
 *  \param weight               Desired weight value
 */
void wOcraVariableWeightsTaskManager::setWeights(Eigen::Vector3d weight)
{
    for (int i=0; i<nDoF; i++)
    {
        tasks[i]->setWeight(weight(i));
    }
}

/** Gets the weight constant for this task
 *
 *  \return                     The weight for this task
 */
Eigen::VectorXd wOcraVariableWeightsTaskManager::getWeights()
{
    Eigen::VectorXd weights(nDoF);
    for (int i=0; i<nDoF; i++)
    {
        weights(i) = tasks[i]->getWeight()[0];
    }
    return weights;
}

/** Sets the stiffness for this task
 *
 * \param stiffness             Desired stiffness
 */
void wOcraVariableWeightsTaskManager::setStiffness(double stiffness)
{
    for (int i=0; i<nDoF; i++)
    {
        tasks[i]->setStiffness(stiffness);
    }
}

/** Gets the stiffness constant for this task
 *
 *  \return                     The stiffness for this task
 */
double wOcraVariableWeightsTaskManager::getStiffness()
{
    Eigen::MatrixXd K = tasks[0]->getStiffness();
    return K(0, 0);
}

/** Sets the damping for this task
 *
 * \param damping               Desired damping
 */
void wOcraVariableWeightsTaskManager::setDamping(double damping)
{
    for (int i=0; i<nDoF; i++)
    {
        tasks[i]->setDamping(damping);
    }
}

/** Gets the damping constant for this task
 *
 *  \return                     The damping for this task
 */
double wOcraVariableWeightsTaskManager::getDamping()
{
    Eigen::MatrixXd C = tasks[0]->getDamping();
    return C(0, 0);
}

/** Activates the task
 *
 */
void wOcraVariableWeightsTaskManager::activate()
{
    for (int i=0; i<nDoF; i++)
    {
        tasks[i]->activateAsObjective();
    }
}

/** Deactivates the task
 *
 */
void wOcraVariableWeightsTaskManager::deactivate()
{
    for (int i=0; i<nDoF; i++)
    {
        tasks[i]->deactivate();
    }
}

/** Gets the error for this task (Cartesian error)
 *
 */
Eigen::VectorXd wOcraVariableWeightsTaskManager::getTaskError()
{
    Eigen::VectorXd errors(nDoF);
    for (int i=0; i<nDoF; i++)
    {
        errors(i) = tasks[i]->getError()[i];
    }
    return errors;
}

Eigen::Vector3d wOcraVariableWeightsTaskManager::getTaskFramePosition()
{
    return featFrame->getPosition().getTranslation();
}


}

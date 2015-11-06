#include "wocra/Tasks/Managers/wOcraFullPostureTaskManager.h"

namespace wocra
{

/** Base constructor
 *
 * \param _ctrl                 wOcraController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _fullStateType        ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 */
wOcraFullPostureTaskManager::wOcraFullPostureTaskManager(wOcraController& _ctrl,
                                                        const wOcraModel& _model,
                                                        const std::string& _taskName,
                                                        int _fullStateType,
                                                        double _stiffness,
                                                        double _damping,
                                                        double _weight,
                                                        bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts)
{
    _init(_fullStateType, _stiffness, _damping, _weight);
}

/** Base constructor
 *
 * \param _ctrl                 wOcraController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _fullStateType        ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight vector for task
 */
wOcraFullPostureTaskManager::wOcraFullPostureTaskManager(wOcraController& _ctrl,
                                                        const wOcraModel& _model,
                                                        const std::string& _taskName,
                                                        int _fullStateType,
                                                        double _stiffness,
                                                        double _damping,
                                                        const Eigen::VectorXd& _weight,
                                                        bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts)
{
    _init(_fullStateType, _stiffness, _damping, _weight);
}
/** Constructor with desired joint space posture
 *
 * \param _ctrl                 wOcraController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _fullStateType        ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 * \param _init_q               Initial posture
 */
wOcraFullPostureTaskManager::wOcraFullPostureTaskManager(wOcraController& _ctrl,
                                                        const wOcraModel& _model,
                                                        const std::string& _taskName,
                                                        int _fullStateType,
                                                        double _stiffness,
                                                        double _damping,
                                                        double _weight,
                                                        const Eigen::VectorXd& _init_q,
                                                        bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts)
{
    _init(_fullStateType, _stiffness, _damping, _weight);
    setPosture(_init_q);
}
/** Constructor with desired joint space posture
 *
 * \param _ctrl                 wOcraController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _fullStateType        ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight vector for task
 * \param _init_q               Initial posture
 */
wOcraFullPostureTaskManager::wOcraFullPostureTaskManager(wOcraController& _ctrl,
                                                        const wOcraModel& _model,
                                                        const std::string& _taskName,
                                                        int _fullStateType,
                                                        double _stiffness,
                                                        double _damping,
                                                        const Eigen::VectorXd& _weight,
                                                        const Eigen::VectorXd& _init_q,
                                                        bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts)
{
    _init(_fullStateType, _stiffness, _damping, _weight);
    setPosture(_init_q);
}

wOcraFullPostureTaskManager::~wOcraFullPostureTaskManager()
{
    
}



/** Initializer function for constructor, sets up the frames, parameters, controller and task
 *
 */
void wOcraFullPostureTaskManager::_init(int fullStateType, double stiffness, double damping, double weight)
{
    featState = new ocra::FullModelState(name + ".FullModelState", model, fullStateType);
    featDesState = new ocra::FullTargetState(name + ".FullTargetState", model, fullStateType);
    feat = new ocra::FullStateFeature(name + ".FullStateFeature", *featState);
    featDes = new ocra::FullStateFeature(name + ".FullStateFeature_Des", *featDesState);

    // The feature initializes as Zero for posture
    task = &(ctrl.createwOcraTask(name, *feat, *featDes));
    task->initAsAccelerationTask();
    ctrl.addTask(*task);


    task->setStiffness(stiffness);
    task->setDamping(damping);
    task->setWeight(weight);

    task->activateAsObjective();

    setStateDimension(task->getDimension());
}

void wOcraFullPostureTaskManager::_init(int fullStateType, double stiffness, double damping, const Eigen::VectorXd& weight)
{
    featState = new ocra::FullModelState(name + ".FullModelState", model, fullStateType);
    featDesState = new ocra::FullTargetState(name + ".FullTargetState", model, fullStateType);
    feat = new ocra::FullStateFeature(name + ".FullStateFeature", *featState);
    featDes = new ocra::FullStateFeature(name + ".FullStateFeature_Des", *featDesState);

    // The feature initializes as Zero for posture
    task = &(ctrl.createwOcraTask(name, *feat, *featDes));
    task->initAsAccelerationTask();
    ctrl.addTask(*task);


    task->setStiffness(stiffness);
    task->setDamping(damping);
    task->setWeight(weight);

    task->activateAsObjective();

    setStateDimension(task->getDimension());
}

/** Sets the full joint space posture for the task
 *
 */
void wOcraFullPostureTaskManager::setPosture(const Eigen::VectorXd& q)
{
    setPosture(q, Eigen::VectorXd::Zero(featDesState->getSize()), Eigen::VectorXd::Zero(featDesState->getSize()));
}

/** Sets the full joint space posture, velocity and acceleration for the task
 *
 */
void wOcraFullPostureTaskManager::setPosture(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot)
{
    // Need to check size of q
    if (q.size() != featDesState->getSize())
    {
        throw std::runtime_error("[wOcraFullPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose and required dimension does not match");
    }
    else if (qdot.size() != featDesState->getSize())
    {
        throw std::runtime_error("[wOcraFullPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose velocity and required dimension does not match");
    }
    else if (qddot.size() != featDesState->getSize())
    {
        throw std::runtime_error("[wOcraFullPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose acceleration and required dimension does not match");
    }

    featDesState->set_q(q);
    featDesState->set_qdot(qdot);
    featDesState->set_qddot(qddot);

    //TODO: Need to include dq and ddq. Adjust stateDimension accordingly.
    updateDesiredStateVector(q.data());
}

void wOcraFullPostureTaskManager::setDesiredState()
{
    Eigen::VectorXd newPosture = Eigen::VectorXd::Map(&newDesiredStateVector.front(), stateDimension);
    setPosture(newPosture);
}


/** Sets the weight constant for this task
 *
 *  \param weight               Desired weight value
 */
void wOcraFullPostureTaskManager::setWeight(double weight)
{
    task->setWeight(weight);
}

/** Sets the weight vector for this task
 *
 *  \param weight               Desired weight value
 */
void wOcraFullPostureTaskManager::setWeight(const Eigen::VectorXd& weight)
{
    task->setWeight(weight);
}

/** Gets the weight for this task
 *
 *  \return                     The weight for this task
 */
Eigen::VectorXd wOcraFullPostureTaskManager::getWeight()
{
    return task->getWeight();
}

/** Sets the stiffness for this task
 *
 * \param stiffness             Desired stiffness
 */
void wOcraFullPostureTaskManager::setStiffness(double stiffness)
{
    task->setStiffness(stiffness);
}

/** Gets the stiffness constant for this task
 *
 *  \return                     The stiffness for this task
 */
double wOcraFullPostureTaskManager::getStiffness()
{
    Eigen::MatrixXd K = task->getStiffness();
    return K(0, 0);
}

/** Sets the damping for this task
 *
 * \param damping               Desired damping
 */
void wOcraFullPostureTaskManager::setDamping(double damping)
{
    task->setDamping(damping);
}

/** Gets the damping constant for this task
 *
 *  \return                     The damping for this task
 */
double wOcraFullPostureTaskManager::getDamping()
{
    Eigen::MatrixXd C = task->getDamping();
    return C(0, 0);
}

/** Activate function
 *
 */
void wOcraFullPostureTaskManager::activate()
{
    task->activateAsObjective();
}

/** Deactivate function
 *
 */
void wOcraFullPostureTaskManager::deactivate()
{
    task->deactivate();
}

/** Gets the error for this task (posture error)
 *
 */
Eigen::VectorXd wOcraFullPostureTaskManager::getTaskError()
{
    return task->getError();
}

const double* wOcraFullPostureTaskManager::getCurrentState()
{
    return featState->q().data();
}


std::string wOcraFullPostureTaskManager::getTaskManagerType()
{
    return "wOcraFullPostureTaskManager";
}

bool wOcraFullPostureTaskManager::checkIfActivated()
{
    return task->isActiveAsObjective();
}


}

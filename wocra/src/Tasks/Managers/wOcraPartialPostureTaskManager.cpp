#include "wocra/Tasks/Managers/wOcraPartialPostureTaskManager.h"

namespace wocra
{

/** Base constructor
 *
 * \param ctrl                  wOcraController to connect to
 * \param model                 ocra model to setup the task
 * \param taskName              Name of the task
 * \param fullStateType         ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param dofIndices            Vector of indices for the DOFs to control
 * \param stiffness             Stiffness constant for task
 * \param damping               Damping constant for task
 * \param weight                Weight constant for task
 */
wOcraPartialPostureTaskManager::wOcraPartialPostureTaskManager(wOcraController& _ctrl,
                                                                const wOcraModel& _model,
                                                                const std::string& _taskName,
                                                                int _fullStateType,
                                                                Eigen::VectorXi& _dofIndices,
                                                                double _stiffness,
                                                                double _damping,
                                                                double _weight,
                                                                bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts)
{
    _init(_fullStateType, _dofIndices, _stiffness, _damping, _weight);
}

wOcraPartialPostureTaskManager::wOcraPartialPostureTaskManager(wOcraController& _ctrl,
                                                                const wOcraModel& _model,
                                                                const std::string& _taskName,
                                                                int _fullStateType,
                                                                Eigen::VectorXi& _dofIndices,
                                                                double _stiffness,
                                                                double _damping,
                                                                const Eigen::VectorXd& _weight,
                                                                bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts)
{
    _init(_fullStateType, _dofIndices, _stiffness, _damping, _weight);
}


/** Constructor with desired initial position
 *
 * \param ctrl                  wOcraController to connect to
 * \param model                 ocra model to setup the task
 * \param taskName              Name of the task
 * \param fullStateType         ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param dofIndices            Vector of indices for the DOFs to control
 * \param stiffness             Stiffness constant for task
 * \param damping               Damping constant for task
 * \param weight                Weight constant for task
 * \param init_q                Initial posture
 */
wOcraPartialPostureTaskManager::wOcraPartialPostureTaskManager(wOcraController& _ctrl,
                                                                const wOcraModel& _model,
                                                                const std::string& _taskName,
                                                                int _fullStateType,
                                                                Eigen::VectorXi& _dofIndices,
                                                                double _stiffness,
                                                                double _damping,
                                                                double _weight,
                                                                Eigen::VectorXd& _init_q,
                                                                bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts)
{
    _init(_fullStateType, _dofIndices, _stiffness, _damping, _weight);
    setPosture(_init_q);
}

wOcraPartialPostureTaskManager::wOcraPartialPostureTaskManager(wOcraController& _ctrl,
                                                                const wOcraModel& _model,
                                                                const std::string& _taskName,
                                                                int _fullStateType,
                                                                Eigen::VectorXi& _dofIndices,
                                                                double _stiffness,
                                                                double _damping,
                                                                const Eigen::VectorXd& _weight,
                                                                Eigen::VectorXd& _init_q,
                                                                bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts)
{
    _init(_fullStateType, _dofIndices, _stiffness, _damping, _weight);
    setPosture(_init_q);
}

wOcraPartialPostureTaskManager::~wOcraPartialPostureTaskManager()
{
    
}


/** Initializer function for constructor, sets up the frames, parameters, controller and task
 *
 */
void wOcraPartialPostureTaskManager::_init(int _fullStateType, VectorXi& _dofIndices, double _stiffness, double _damping, double _weight)
{
    featState = new wocra::PartialModelState(name + ".PartialModelState", model, _dofIndices, _fullStateType);
    featDesState = new wocra::PartialTargetState(name + ".PartialTargetState", model, _dofIndices, _fullStateType);
    feat = new wocra::PartialStateFeature(name + ".PartialStateFeature", *featState);
    featDes = new wocra::PartialStateFeature(name + ".PartialStateFeature_Des", *featDesState);

    // The feature initializes as Zero for posture
    task = &(ctrl.createwOcraTask(name, *feat, *featDes));
    task->initAsAccelerationTask();
    ctrl.addTask(*task);


    task->setStiffness(_stiffness);
    task->setDamping(_damping);
    task->setWeight(_weight);

    task->activateAsObjective();

    setStateDimension(task->getDimension());
}

void wOcraPartialPostureTaskManager::_init(int _fullStateType, VectorXi& _dofIndices, double _stiffness, double _damping, const Eigen::VectorXd& _weight)
{
    featState = new wocra::PartialModelState(name + ".PartialModelState", model, _dofIndices, _fullStateType);
    featDesState = new wocra::PartialTargetState(name + ".PartialTargetState", model, _dofIndices, _fullStateType);
    feat = new wocra::PartialStateFeature(name + ".PartialStateFeature", *featState);
    featDes = new wocra::PartialStateFeature(name + ".PartialStateFeature_Des", *featDesState);

    // The feature initializes as Zero for posture
    task = &(ctrl.createwOcraTask(name, *feat, *featDes));
    task->initAsAccelerationTask();
    ctrl.addTask(*task);


    task->setStiffness(_stiffness);
    task->setDamping(_damping);
    task->setWeight(_weight);

    task->activateAsObjective();

    setStateDimension(task->getDimension());
}

/** Sets the partial joint space posture for the task
 *
 */
void wOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd& q)
{
    // Need to check size of q
    if (q.size() != featDesState->getSize())
    {
        throw std::runtime_error("[wOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd&)] Input pose and required dimension does not match");
    }
    featDesState->set_q(q);
    featDesState->set_qdot(Eigen::VectorXd::Zero(featDesState->getSize()));
    featDesState->set_qddot(Eigen::VectorXd::Zero(featDesState->getSize()));


    //TODO: Need to include dq and ddq. Adjust stateDimension accordingly.
    updateDesiredStateVector(q.data());
}

/** Sets the partial joint space posture, velocity and acceleration for the task
 *
 */
void wOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd& q, Eigen::VectorXd& qdot, Eigen::VectorXd& qddot)
{
    // Need to check size of q
    if (q.size() != featDesState->getSize())
    {
        throw std::runtime_error("[wOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose and required dimension does not match");
    }
    else if (qdot.size() != featDesState->getSize())
    {
        throw std::runtime_error("[wOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose velocity and required dimension does not match");
    }
    else if (qddot.size() != featDesState->getSize())
    {
        throw std::runtime_error("[wOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose acceleration and required dimension does not match");
    }

    featDesState->set_q(q);
    featDesState->set_qdot(qdot);
    featDesState->set_qddot(qddot);
}

void wOcraPartialPostureTaskManager::setDesiredState()
{
    Eigen::VectorXd newPosture = Eigen::VectorXd::Map(&newDesiredStateVector.front(), stateDimension);
    setPosture(newPosture);
}


/** Activate function
 *
 */
void wOcraPartialPostureTaskManager::activate()
{
    task->activateAsObjective();
}

/** Deactivate function
 *
 */
void wOcraPartialPostureTaskManager::deactivate()
{
    task->deactivate();
}

/** Sets the weight constant for this task
 *
 *  \param weight               Desired weight value
 */
void wOcraPartialPostureTaskManager::setWeight(double weight)
{
    task->setWeight(weight);
}

void wOcraPartialPostureTaskManager::setWeight(const Eigen::VectorXd& weight)
{
    task->setWeight(weight);
}

/** Gets the weight constant for this task
 *
 *  \return                     The weight for this task
 */
Eigen::VectorXd wOcraPartialPostureTaskManager::getWeight()
{
    return task->getWeight();
}

/** Sets the stiffness for this task
 *
 * \param stiffness             Desired stiffness
 */
void wOcraPartialPostureTaskManager::setStiffness(double stiffness)
{
    task->setStiffness(stiffness);
}

/** Gets the stiffness constant for this task
 *
 *  \return                     The stiffness for this task
 */
double wOcraPartialPostureTaskManager::getStiffness()
{
    Eigen::MatrixXd K = task->getStiffness();
    return K(0, 0);
}

/** Sets the damping for this task
 *
 * \param damping               Desired damping
 */
void wOcraPartialPostureTaskManager::setDamping(double damping)
{
    task->setDamping(damping);
}

/** Gets the damping constant for this task
 *
 *  \return                     The damping for this task
 */
double wOcraPartialPostureTaskManager::getDamping()
{
    Eigen::MatrixXd C = task->getDamping();
    return C(0, 0);
}

const double* wOcraPartialPostureTaskManager::getCurrentState()
{
    return featState->q().data();
}

std::string wOcraPartialPostureTaskManager::getTaskManagerType()
{
    return "wOcraPartialPostureTaskManager";
}


bool wOcraPartialPostureTaskManager::checkIfActivated()
{
    return task->isActiveAsObjective();
}






}

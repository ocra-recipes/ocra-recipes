#include "ocra/control/TaskManagers/FullPostureTaskManager.h"

namespace ocra
{

/** Base constructor
 *
 * \param _ctrl                 ocra::Controller to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _fullStateType        ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 */
FullPostureTaskManager::FullPostureTaskManager(ocra::Controller& _ctrl,
                                                        const ocra::Model& _model,
                                                        const std::string& _taskName,
                                                        int _fullStateType,
                                                        double _stiffness,
                                                        double _damping,
                                                        double _weight,
                                                        bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts)
{
    _init(_fullStateType, _stiffness, _damping, _weight);
    Eigen::VectorXd _init_q = _model.getJointPositions();
    setPosture(_init_q);

}

/** Base constructor
 *
 * \param _ctrl                 ocra::Controller to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _fullStateType        ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight vector for task
 */
FullPostureTaskManager::FullPostureTaskManager(ocra::Controller& _ctrl,
                                                        const ocra::Model& _model,
                                                        const std::string& _taskName,
                                                        int _fullStateType,
                                                        double _stiffness,
                                                        double _damping,
                                                        const Eigen::VectorXd& _weight,
                                                        bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts)
{
    _init(_fullStateType, _stiffness, _damping, _weight);
    Eigen::VectorXd _init_q = _model.getJointPositions();
    setPosture(_init_q);

}
/** Constructor with desired joint space posture
 *
 * \param _ctrl                 ocra::Controller to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _fullStateType        ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 * \param _init_q               Initial posture
 */
FullPostureTaskManager::FullPostureTaskManager(ocra::Controller& _ctrl,
                                                        const ocra::Model& _model,
                                                        const std::string& _taskName,
                                                        int _fullStateType,
                                                        double _stiffness,
                                                        double _damping,
                                                        double _weight,
                                                        const Eigen::VectorXd& _init_q,
                                                        bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts)
{
    _init(_fullStateType, _stiffness, _damping, _weight);
    setPosture(_init_q);
}
/** Constructor with desired joint space posture
 *
 * \param _ctrl                 ocra::Controller to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _fullStateType        ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight vector for task
 * \param _init_q               Initial posture
 */
FullPostureTaskManager::FullPostureTaskManager(ocra::Controller& _ctrl,
                                                        const ocra::Model& _model,
                                                        const std::string& _taskName,
                                                        int _fullStateType,
                                                        double _stiffness,
                                                        double _damping,
                                                        const Eigen::VectorXd& _weight,
                                                        const Eigen::VectorXd& _init_q,
                                                        bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts)
{
    _init(_fullStateType, _stiffness, _damping, _weight);
    setPosture(_init_q);
}

FullPostureTaskManager::~FullPostureTaskManager()
{

}



/** Initializer function for constructor, sets up the frames, parameters, controller and task
 *
 */
void FullPostureTaskManager::_init(int fullStateType, double stiffness, double damping, double weight)
{
    featState = new ocra::FullModelState(name + ".FullModelState", model, fullStateType);
    featDesState = new ocra::FullTargetState(name + ".FullTargetState", model, fullStateType);
    feat = new ocra::FullStateFeature(name + ".FullStateFeature", *featState);
    featDes = new ocra::FullStateFeature(name + ".FullStateFeature_Des", *featDesState);

    // The feature initializes as Zero for posture
    task = ctrl.createTask(name, *feat, *featDes);
    task->setTaskType(ocra::Task::ACCELERATIONTASK);
    ctrl.addTask(task);


    task->setStiffness(stiffness);
    task->setDamping(damping);
    task->setWeight(weight);

    task->activateAsObjective();

    setStateDimension(task->getDimension());
}

void FullPostureTaskManager::_init(int fullStateType, double stiffness, double damping, const Eigen::VectorXd& weight)
{
    featState = new ocra::FullModelState(name + ".FullModelState", model, fullStateType);
    featDesState = new ocra::FullTargetState(name + ".FullTargetState", model, fullStateType);
    feat = new ocra::FullStateFeature(name + ".FullStateFeature", *featState);
    featDes = new ocra::FullStateFeature(name + ".FullStateFeature_Des", *featDesState);

    // The feature initializes as Zero for posture
    task = ctrl.createTask(name, *feat, *featDes);
    task->setTaskType(ocra::Task::ACCELERATIONTASK);
    ctrl.addTask(task);


    task->setStiffness(stiffness);
    task->setDamping(damping);
    task->setWeight(weight);

    task->activateAsObjective();

    setStateDimension(task->getDimension());
}

/** Sets the full joint space posture for the task
 *
 */
void FullPostureTaskManager::setPosture(const Eigen::VectorXd& q)
{
    setPosture(q, Eigen::VectorXd::Zero(featDesState->getSize()), Eigen::VectorXd::Zero(featDesState->getSize()));
}

/** Sets the full joint space posture, velocity and acceleration for the task
 *
 */
void FullPostureTaskManager::setPosture(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot)
{
    // Need to check size of q
    if (q.size() != featDesState->getSize())
    {
        throw std::runtime_error("[FullPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose and required dimension does not match");
    }
    else if (qdot.size() != featDesState->getSize())
    {
        throw std::runtime_error("[FullPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose velocity and required dimension does not match");
    }
    else if (qddot.size() != featDesState->getSize())
    {
        throw std::runtime_error("[FullPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose acceleration and required dimension does not match");
    }

    featDesState->set_q(q);
    featDesState->set_qdot(qdot);
    featDesState->set_qddot(qddot);

    //TODO: Need to include dq and ddq. Adjust stateDimension accordingly.
    updateDesiredStateVector(q.data());
}

void FullPostureTaskManager::setDesiredState()
{
    Eigen::VectorXd newPosture = Eigen::VectorXd::Map(&newDesiredStateVector.front(), stateDimension);
    setPosture(newPosture);
}


/** Sets the weight constant for this task
 *
 *  \param weight               Desired weight value
 */
void FullPostureTaskManager::setWeight(double weight)
{
    task->setWeight(weight);
}

/** Sets the weight vector for this task
 *
 *  \param weight               Desired weight value
 */
void FullPostureTaskManager::setWeight(const Eigen::VectorXd& weight)
{
    task->setWeight(weight);
}

/** Gets the weight for this task
 *
 *  \return                     The weight for this task
 */
Eigen::VectorXd FullPostureTaskManager::getWeight()
{
    return task->getWeight();
}

/** Sets the stiffness for this task
 *
 * \param stiffness             Desired stiffness
 */
void FullPostureTaskManager::setStiffness(double stiffness)
{
    task->setStiffness(stiffness);
}

/** Gets the stiffness constant for this task
 *
 *  \return                     The stiffness for this task
 */
double FullPostureTaskManager::getStiffness()
{
    Eigen::MatrixXd K = task->getStiffness();
    return K(0, 0);
}

/** Sets the damping for this task
 *
 * \param damping               Desired damping
 */
void FullPostureTaskManager::setDamping(double damping)
{
    task->setDamping(damping);
}

/** Gets the damping constant for this task
 *
 *  \return                     The damping for this task
 */
double FullPostureTaskManager::getDamping()
{
    Eigen::MatrixXd C = task->getDamping();
    return C(0, 0);
}


/** Gets the error for this task (posture error)
 *
 */
Eigen::VectorXd FullPostureTaskManager::getTaskError()
{
    return task->getError();
}

const double* FullPostureTaskManager::getCurrentState()
{
    return featState->q().data();
}


std::string FullPostureTaskManager::getTaskManagerType()
{
    return "FullPostureTaskManager";
}

bool FullPostureTaskManager::checkIfActivated()
{
    return task->isActiveAsObjective();
}


}

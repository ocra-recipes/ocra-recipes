#include "gocra/Tasks/Managers/gOcraFullPostureTaskManager.h"

namespace gocra
{

/** Base constructor
 *
 * \param _ctrl                 GHCJTController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _fullStateType        ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 */
gOcraFullPostureTaskManager::gOcraFullPostureTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, int _fullStateType, double _stiffness, double _damping)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName)
{
    _init(_fullStateType, _stiffness, _damping);
}

/** Constructor with desired joint space posture
 *
 * \param _ctrl                 GHCJTController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _fullStateType        ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _init_q               Initial posture
 */
gOcraFullPostureTaskManager::gOcraFullPostureTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, int _fullStateType, double _stiffness, double _damping, const Eigen::VectorXd& _init_q)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName)
{
    _init(_fullStateType, _stiffness, _damping);
    setPosture(_init_q);
}

/** Initializer function for constructor, sets up the frames, parameters, controller and task
 *
 */
void gOcraFullPostureTaskManager::_init(int fullStateType, double stiffness, double damping)
{
    featState = new ocra::FullModelState(name + ".FullModelState", model, fullStateType);
    featDesState = new ocra::FullTargetState(name + ".FullTargetState", model, fullStateType);
    feat = new ocra::FullStateFeature(name + ".FullStateFeature", *featState);
    featDes = new ocra::FullStateFeature(name + ".FullStateFeature_Des", *featDesState);

    // The feature initializes as Zero for posture
    task = &(ctrl.createGHCJTTask(name, *feat, *featDes));

    ctrl.addTask(*task);


    task->setStiffness(stiffness);
    task->setDamping(damping);
    task->setWeight(1);
    task->activateAsObjective();
}

/** Sets the full joint space posture for the task
 *
 */
void gOcraFullPostureTaskManager::setPosture(const Eigen::VectorXd& q)
{
    setPosture(q, Eigen::VectorXd::Zero(featDesState->getSize()), Eigen::VectorXd::Zero(featDesState->getSize()));
}

/** Sets the full joint space posture, velocity and acceleration for the task
 *
 */
void gOcraFullPostureTaskManager::setPosture(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot)
{
    // Need to check size of q
    if (q.size() != featDesState->getSize())
    {
        throw std::runtime_error("[gOcraFullPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose and required dimension does not match");
    }
    else if (qdot.size() != featDesState->getSize())
    {
        throw std::runtime_error("[gOcraFullPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose velocity and required dimension does not match");
    }
    else if (qddot.size() != featDesState->getSize())
    {
        throw std::runtime_error("[gOcraFullPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose acceleration and required dimension does not match");
    }

    featDesState->set_q(q);
    featDesState->set_qdot(qdot);
    featDesState->set_qddot(qddot);
}



/** Sets the stiffness for this task
 *
 * \param stiffness             Desired stiffness
 */
void gOcraFullPostureTaskManager::setStiffness(double stiffness)
{
    task->setStiffness(stiffness);
}

/** Gets the stiffness constant for this task
 *
 *  \return                     The stiffness for this task
 */
double gOcraFullPostureTaskManager::getStiffness()
{
    Eigen::MatrixXd K = task->getStiffness();
    return K(0, 0);
}

/** Sets the damping for this task
 *
 * \param damping               Desired damping
 */
void gOcraFullPostureTaskManager::setDamping(double damping)
{
    task->setDamping(damping);
}

/** Gets the damping constant for this task
 *
 *  \return                     The damping for this task
 */
double gOcraFullPostureTaskManager::getDamping()
{
    Eigen::MatrixXd C = task->getDamping();
    return C(0, 0);
}

/** Activate function
 *
 */
void gOcraFullPostureTaskManager::activate()
{
    task->activateAsObjective();
}

/** Deactivate function
 *
 */
void gOcraFullPostureTaskManager::deactivate()
{
    task->deactivate();
}

/** Gets the error for this task (posture error)
 *
 */
Eigen::VectorXd gOcraFullPostureTaskManager::getTaskError()
{
    return task->getError();
}


}

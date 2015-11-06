#include "gocra/Tasks/Managers/gOcraPartialPostureTaskManager.h"

namespace gocra
{

/** Base constructor
 *
 * \param ctrl                  GHCJTController to connect to
 * \param model                 ocra model to setup the task
 * \param taskName              Name of the task
 * \param fullStateType         ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param dofIndices            Vector of indices for the DOFs to control
 * \param stiffness             Stiffness constant for task
 * \param damping               Damping constant for task
 */
gOcraPartialPostureTaskManager::gOcraPartialPostureTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, int _fullStateType, Eigen::VectorXi& _dofIndices, double _stiffness, double _damping)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName)
{
    _init(_fullStateType, _dofIndices, _stiffness, _damping);
}

/** Constructor with desired initial position
 *
 * \param ctrl                  GHCJTController to connect to
 * \param model                 ocra model to setup the task
 * \param taskName              Name of the task
 * \param fullStateType         ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param dofIndices            Vector of indices for the DOFs to control
 * \param stiffness             Stiffness constant for task
 * \param damping               Damping constant for task
 * \param init_q                Initial posture
 */
gOcraPartialPostureTaskManager::gOcraPartialPostureTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, int _fullStateType, Eigen::VectorXi& _dofIndices, double _stiffness, double _damping, Eigen::VectorXd& _init_q)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName)
{
    _init(_fullStateType, _dofIndices, _stiffness, _damping);
    setPosture(_init_q);
}

/** Initializer function for constructor, sets up the frames, parameters, controller and task
 *
 */
void gOcraPartialPostureTaskManager::_init(int _fullStateType, VectorXi& _dofIndices, double _stiffness, double _damping)
{
    featState = new gocra::PartialModelState(name + ".PartialModelState", model, _dofIndices, _fullStateType);
    featDesState = new gocra::PartialTargetState(name + ".PartialTargetState", model, _dofIndices, _fullStateType);
    feat = new gocra::PartialStateFeature(name + ".PartialStateFeature", *featState);
    featDes = new gocra::PartialStateFeature(name + ".PartialStateFeature_Des", *featDesState);

    // The feature initializes as Zero for posture
    task = &(ctrl.createGHCJTTask(name, *feat, *featDes));
    ctrl.addTask(*task);


    task->setStiffness(_stiffness);
    task->setDamping(_damping);
    task->setWeight(1);

    task->activateAsObjective();
}

/** Sets the partial joint space posture for the task
 *
 */
void gOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd& q)
{
    // Need to check size of q
    if (q.size() != featDesState->getSize())
    {
        throw std::runtime_error("[gOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd&)] Input pose and required dimension does not match");
    }
    featDesState->set_q(q);
    featDesState->set_qdot(Eigen::VectorXd::Zero(featDesState->getSize()));
    featDesState->set_qddot(Eigen::VectorXd::Zero(featDesState->getSize()));
}

/** Sets the partial joint space posture, velocity and acceleration for the task
 *
 */
void gOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd& q, Eigen::VectorXd& qdot, Eigen::VectorXd& qddot)
{
    // Need to check size of q
    if (q.size() != featDesState->getSize())
    {
        throw std::runtime_error("[gOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose and required dimension does not match");
    }
    else if (qdot.size() != featDesState->getSize())
    {
        throw std::runtime_error("[gOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose velocity and required dimension does not match");
    }
    else if (qddot.size() != featDesState->getSize())
    {
        throw std::runtime_error("[gOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose acceleration and required dimension does not match");
    }

    featDesState->set_q(q);
    featDesState->set_qdot(qdot);
    featDesState->set_qddot(qddot);
}


/** Activate function
 *
 */
void gOcraPartialPostureTaskManager::activate()
{
    task->activateAsObjective();
}

/** Deactivate function
 *
 */
void gOcraPartialPostureTaskManager::deactivate()
{
    task->deactivate();
}


}

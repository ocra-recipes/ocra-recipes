#include <ocra/control/TaskBuilders/FullPostureTaskBuilder.h>

using namespace ocra;

FullPostureTaskBuilder::FullPostureTaskBuilder(const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr)
: TaskBuilder(taskOptions, modelPtr)
{
    // Set the type of Full State we want to use:
    // FULL_STATE, FREE_FLYER, or INTERNAL
    // FULL_STATE = FloatingBase + InternalDoF
    // FREE_FLYER = FloatingBase
    // INTERNAL = InternalDoF (joints)

    this->fullStateType = FullState::INTERNAL;
}

FullPostureTaskBuilder::~FullPostureTaskBuilder()
{
}

Feature::Ptr FullPostureTaskBuilder::buildFeature()
{
    std::string featStateName = this->options.taskName + ".FullModelState";
    std::string featName = this->options.taskName + ".FullStateFeature";

    FullState::Ptr featState =  std::make_shared<FullModelState>(featStateName, *this->model, this->fullStateType);

    return std::make_shared<FullStateFeature>(featName, featState);
}

Feature::Ptr FullPostureTaskBuilder::buildFeatureDesired()
{
    std::string featDesStateName = this->options.taskName + ".FullTargetState";
    std::string featDesName = this->options.taskName + ".FullStateFeatureDesired";

    FullState::Ptr featDesState = std::make_shared<FullTargetState>(featDesStateName, *this->model, this->fullStateType);

    return std::make_shared<FullStateFeature>(featDesName, featDesState);
}

void FullPostureTaskBuilder::setTaskState()
{
    TaskState state;
    // Get the number of joints on the robot.
    int nJoints = this->model->nbInternalDofs();
    // Make sure the desired vector is the same size as the number of axes being controlled.
    if(this->options.desired.size() == nJoints){
        // Set the pose
        state.setQ(this->options.desired);
    }else{
        // If the desired position was not given then just get it from the current state of the task.
        state.setQ(this->task->getTaskState().getQ());
    }

    // Set velocity and acceleration to zero.
    state.setQd(Eigen::VectorXd::Zero(nJoints));
    state.setQdd(Eigen::VectorXd::Zero(nJoints));

    this->task->setDesiredTaskStateDirect(state);
}

void FullPostureTaskBuilder::setTaskType()
{
    this->task->setTaskType(Task::ACCELERATIONTASK);
}

#include <ocra/control/TaskBuilders/PartialPostureTaskBuilder.h>

using namespace ocra;

PartialPostureTaskBuilder::PartialPostureTaskBuilder(const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr)
: TaskBuilder(taskOptions, modelPtr)
{
    // Set the type of Partial State we want to use:
    // FULL_STATE, FREE_FLYER, or INTERNAL
    // FULL_STATE = FloatingBase + InternalDoF
    // FREE_FLYER = FloatingBase
    // INTERNAL = InternalDoF (joints)

    this->partialStateType = FullState::INTERNAL;
}

PartialPostureTaskBuilder::~PartialPostureTaskBuilder()
{
}

Feature::Ptr PartialPostureTaskBuilder::buildFeature()
{
    std::string featStateName = this->options.taskName + ".PartialModelState";
    std::string featName = this->options.taskName + ".PartialStateFeature";

    PartialState::Ptr featState =  std::make_shared<PartialModelState>(featStateName, *this->model, this->options.jointIndexes, this->partialStateType);

    return std::make_shared<PartialStateFeature>(featName, featState);
}

Feature::Ptr PartialPostureTaskBuilder::buildFeatureDesired()
{
    std::string featDesStateName = this->options.taskName + ".PartialTargetState";
    std::string featDesName = this->options.taskName + ".PartialStateFeatureDesired";

    PartialState::Ptr featDesState = std::make_shared<PartialTargetState>(featDesStateName, *this->model, this->options.jointIndexes, this->partialStateType);

    return std::make_shared<PartialStateFeature>(featDesName, featDesState);
}

void PartialPostureTaskBuilder::setTaskState()
{
    TaskState state;
    // Get the number of joints on the robot.
    int nJoints = this->options.jointIndexes.rows();
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

void PartialPostureTaskBuilder::setTaskType()
{
    this->task->setTaskType(Task::ACCELERATIONTASK);
    this->task->setMetaTaskType(Task::PARTIAL_POSTURE);
}

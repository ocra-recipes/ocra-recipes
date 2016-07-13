#include <ocra/control/TaskBuilders/PoseTaskBuilder.h>

using namespace ocra;

PoseTaskBuilder::PoseTaskBuilder(const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr)
: TaskBuilder(taskOptions, modelPtr)
{
}

PoseTaskBuilder::~PoseTaskBuilder()
{

}

Feature::Ptr PoseTaskBuilder::buildFeature()
{
    std::string featFrameName = this->options.taskName + ".SegmentFrame";
    std::string featName = this->options.taskName + ".DisplacementFeature";
    std::string segmentName = this->model->SegmentName(this->options.segment);

    ControlFrame::Ptr featFrame =  std::make_shared<SegmentFrame>(featFrameName, *this->model, segmentName, this->options.offset);

    return std::make_shared<DisplacementFeature>(featName, featFrame, this->options.axes);
}

Feature::Ptr PoseTaskBuilder::buildFeatureDesired()
{
    std::string featDesFrameName = this->options.taskName + ".TargetFrame";
    std::string featDesName = this->options.taskName + ".DisplacementFeatureDesired";

    ControlFrame::Ptr featDesFrame = std::make_shared<TargetFrame>(featDesFrameName, *this->model);

    return std::make_shared<DisplacementFeature>(featDesName, featDesFrame, this->options.axes);
}

void PoseTaskBuilder::setTaskState()
{
    TaskState state;
    // Make sure the desired vector is the same size as the number of axes being controlled.
    if(this->options.desired.size() > 0){
        // Set the pose
        state.setPosition(util::eigenVectorToDisplacementd(this->options.desired));
    }else{
        // If the desired position was not given then just get it from the current state of the task.
        state.setPosition(this->task->getTaskState().getPosition());
    }

    // Set velocity and acceleration to zero.
    state.setVelocity(Eigen::Twistd::Zero());
    state.setAcceleration(Eigen::Twistd::Zero());

    this->task->setDesiredTaskStateDirect(state);
}

void PoseTaskBuilder::setTaskType()
{
    this->task->setTaskType(Task::ACCELERATIONTASK);
    this->task->setMetaTaskType(Task::POSE);
}

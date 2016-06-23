#include <ocra/control/TaskBuilders/OrientationTaskBuilder.h>

using namespace ocra;

OrientationTaskBuilder::OrientationTaskBuilder(const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr)
: TaskBuilder(taskOptions, modelPtr)
{
}

OrientationTaskBuilder::~OrientationTaskBuilder()
{

}

Feature::Ptr OrientationTaskBuilder::buildFeature()
{
    std::string featFrameName = this->options.taskName + ".SegmentFrame";
    std::string featName = this->options.taskName + ".OrientationFeature";
    std::string segmentName = this->model->SegmentName(this->options.segment);

    ControlFrame::Ptr featFrame =  std::make_shared<SegmentFrame>(featFrameName, *this->model, segmentName, this->options.offset);

    return std::make_shared<OrientationFeature>(featName, featFrame);
}

Feature::Ptr OrientationTaskBuilder::buildFeatureDesired()
{
    std::string featDesFrameName = this->options.taskName + ".TargetFrame";
    std::string featDesName = this->options.taskName + ".OrientationFeatureDesired";

    ControlFrame::Ptr featDesFrame = std::make_shared<TargetFrame>(featDesFrameName, *this->model);

    return std::make_shared<OrientationFeature>(featDesName, featDesFrame);
}

void OrientationTaskBuilder::setTaskState()
{
    TaskState state;
    // Make sure the desired vector is the same size as a quaternion.
    if(this->options.desired.size() == ORIENTATION_VECTOR_SIZE){
        // Set the orientation
        Eigen::Rotation3d orientation(this->options.desired.data());
        state.setPosition(Eigen::Displacementd(Eigen::Vector3d::Zero(), orientation));
    }else{
        // If the desired orientation was not given then just get it from the current state of the task.
        state.setPosition(this->task->getTaskState().getPosition());
    }

    // Set velocity and acceleration to zero.
    state.setVelocity(Eigen::Twistd::Zero());
    state.setAcceleration(Eigen::Twistd::Zero());

    this->task->setDesiredTaskStateDirect(state);
}

void OrientationTaskBuilder::setTaskType()
{
    this->task->setTaskType(Task::ACCELERATIONTASK);
}

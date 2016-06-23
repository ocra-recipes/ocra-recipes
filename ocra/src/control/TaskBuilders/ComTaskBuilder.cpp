#include <ocra/control/TaskBuilders/ComTaskBuilder.h>

using namespace ocra;

ComTaskBuilder::ComTaskBuilder(const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr)
: TaskBuilder(taskOptions, modelPtr)
{
    this->nDoF = utils::computeDimensionFor(this->options.axes);
}

ComTaskBuilder::~ComTaskBuilder()
{

}

Feature::Ptr ComTaskBuilder::buildFeature()
{
    std::string featFrameName = this->options.taskName + ".ComFrame";
    std::string featName = this->options.taskName + ".PositionFeature";

    // The only difference between a CoM task and a Cartesian Task is the feature frame used.
    ControlFrame::Ptr featFrame =  std::make_shared<CoMFrame>(featFrameName, *this->model);

    return std::make_shared<PositionFeature>(featName, featFrame, this->options.axes);
}

Feature::Ptr ComTaskBuilder::buildFeatureDesired()
{
    std::string featDesFrameName = this->options.taskName + ".TargetFrame";
    std::string featDesName = this->options.taskName + ".PositionFeatureDesired";

    ControlFrame::Ptr featDesFrame = std::make_shared<TargetFrame>(featDesFrameName, *this->model);

    return std::make_shared<PositionFeature>(featDesName, featDesFrame, this->options.axes);
}

void ComTaskBuilder::setTaskState()
{
    TaskState state;
    // Make sure the desired vector is the same size as the number of axes being controlled.
    if(this->options.desired.size() == this->nDoF){
        // Make a temporary 3 dimensional vector and fill it with the desired values
        Eigen::Vector3d tmpPosVec = Eigen::Vector3d::Zero();
        for(auto i=0; i<this->nDoF; ++i){
            tmpPosVec(i) = this->options.desired(i);
        }
        // Set the position
        state.setPosition(Eigen::Displacementd(tmpPosVec, Eigen::Rotation3d::Identity()));
    }else{
        // If the desired position was not given then just get it from the current state of the task.
        state.setPosition(this->task->getTaskState().getPosition());
    }

    // Set velocity and acceleration to zero.
    state.setVelocity(Eigen::Twistd::Zero());
    state.setAcceleration(Eigen::Twistd::Zero());

    this->task->setDesiredTaskStateDirect(state);
}

void ComTaskBuilder::setTaskType()
{
    this->task->setTaskType(Task::ACCELERATIONTASK);
}

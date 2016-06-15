#include <ocra/control/TaskBuilders/CartesianTaskBuilder.h>

using namespace ocra;

CartesianTaskBuilder::CartesianTaskBuilder(const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr)
: TaskBuilder(taskOptions, modelPtr)
{
    this->axes = ECartesianDof(this->options.axes);
    this->nDoF = utils::computeDimensionFor(this->axes);
}

CartesianTaskBuilder::~CartesianTaskBuilder()
{

}

Feature::Ptr CartesianTaskBuilder::buildFeature()
{
    std::string featFrameName = this->options.taskName + ".SegmentFrame";
    std::string featName = this->options.taskName + ".PositionFeature";
    std::string segmentName = this->model->SegmentName(this->options.segment);

    Eigen::VectorXd offsetVectorXd = this->options.offset.front();
    Eigen::Vector3d offsetVector3d = Eigen::Vector3d::Zero();
    if (offsetVectorXd.size() >= this->nDoF) {
        offsetVector3d = offsetVectorXd.head(this->nDoF);
    }
    Eigen::Displacementd frameOffset = Eigen::Displacementd(offsetVector3d);
    ControlFrame::Ptr featFrame =  std::make_shared<SegmentFrame>(featFrameName, *this->model, segmentName, frameOffset);

    return std::make_shared<PositionFeature>(featName, featFrame, this->axes);
}

Feature::Ptr CartesianTaskBuilder::buildFeatureDesired()
{
    std::string featDesFrameName = this->options.taskName + ".TargetFrame";
    std::string featDesName = this->options.taskName + ".PositionFeatureDesired";

    ControlFrame::Ptr featDesFrame = std::make_shared<TargetFrame>(featDesFrameName, *this->model);

    return std::make_shared<PositionFeature>(featDesName, featDesFrame, this->axes);
}

void CartesianTaskBuilder::setTaskState()
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
        state.position = Eigen::Displacementd(tmpPosVec, Eigen::Rotation3d::Identity());
    }else{
        // If the desired position was not given then just get it from the current state of the task.
        state.position = this->task->getTaskState().position;
    }

    // Set velocity and acceleration to zero.
    state.velocity = Eigen::Twistd::Zero();
    state.acceleration = Eigen::Twistd::Zero();

    this->task->setDesiredTaskStateDirect(state);
}

void CartesianTaskBuilder::setTaskType()
{
    this->task->setTaskType(Task::ACCELERATIONTASK);
}

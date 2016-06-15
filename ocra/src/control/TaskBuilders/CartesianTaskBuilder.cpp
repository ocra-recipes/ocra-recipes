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
    // TODO: This is where the parsing and shit needs to happen.
    if(this->options.desired.size() == this->nDoF){
        // state.position = Eigen::Displacementd(this->options.desired);
    }else{
        state.position = this->model->getSegmentPosition(this->options.segment);
    }


    state.velocity = Eigen::Twistd::Zero();
    state.acceleration = Eigen::Twistd::Zero();
    this->task->setDesiredTaskStateDirect(state);
}

void CartesianTaskBuilder::setTaskType()
{
    this->task->setTaskType(Task::ACCELERATIONTASK);
}

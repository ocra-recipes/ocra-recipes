#include <ocra/control/TaskBuilders/CartesianTaskBuilder.h>

using namespace ocra;

CartesianTaskBuilder::CartesianTaskBuilder(const TaskBuilderOptions& options, Model::Ptr model)
: TaskBuilder(options, model)
{
    _axes = ECartesianDof(_options.axes);
    _nDoF = utils::computeDimensionFor(_axes);
}

CartesianTaskBuilder::~CartesianTaskBuilder()
{

}

Feature * CartesianTaskBuilder::buildFeature()
{
    std::string featFrameName = _options.taskName + ".SegmentFrame";
    std::string featName = _options.taskName + ".PositionFeature";
    std::string segmentName = _model->SegmentName(_options.segment);

    Eigen::VectorXd offsetVectorXd = _options.offset.front();
    Eigen::Vector3d offsetVector3d = Eigen::Vector3d::Zero();
    if (offsetVectorXd.size() >= _nDoF) {
        offsetVector3d = offsetVectorXd.head(_nDoF);
    }
    Eigen::Displacementd frameOffset = Eigen::Displacementd(offsetVector3d);
    ControlFrame * featFrame =  new SegmentFrame(featFrameName, *_model, segmentName, frameOffset);

    return new PositionFeature(featName, *featFrame, _axes);
}

Feature * CartesianTaskBuilder::buildFeatureDesired()
{
    std::string featDesFrameName = _options.taskName + ".TargetFrame";
    std::string featDesName = _options.taskName + ".PositionFeatureDesired";

    ControlFrame * featDesFrame = new TargetFrame(featDesFrameName, *_model);

    return new PositionFeature(featDesName, *featDesFrame, _axes);
}

void CartesianTaskBuilder::setTaskState()
{

}

void CartesianTaskBuilder::setTaskType()
{
    _task->setTaskType(Task::ACCELERATIONTASK);
}

#include <ocra/control/TaskBuilders/PointContactTaskBuilder.h>

using namespace ocra;

PointContactTaskBuilder::PointContactTaskBuilder(const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr)
: TaskBuilder(taskOptions, modelPtr)
{
}

PointContactTaskBuilder::~PointContactTaskBuilder()
{

}

Feature::Ptr PointContactTaskBuilder::buildFeature()
{
    std::string featFrameName = this->options.taskName + ".SegmentFrame";
    std::string featName = this->options.taskName + ".PositionFeature";
    std::string segmentName = this->model->SegmentName(this->options.segment);

    ControlFrame::Ptr featFrame =  std::make_shared<SegmentFrame>(featFrameName, *this->model, segmentName, this->options.offset);

    return std::make_shared<PointContactFeature>(featName, featFrame);
}

Feature::Ptr PointContactTaskBuilder::buildFeatureDesired()
{
    Feature::Ptr emptyFeatureDesPtr;
    return emptyFeatureDesPtr;
}

void PointContactTaskBuilder::setTaskState()
{
    this->task->setFrictionCoeff(this->options.mu);
    this->task->setMargin(this->options.margin);
    this->task->activateContactMode();
}

void PointContactTaskBuilder::setTaskType()
{
    this->task->setTaskType(Task::ACCELERATIONTASK);
    this->task->setMetaTaskType(Task::POINT_CONTACT);
}

void PointContactTaskBuilder::setTaskAsObjectiveOrConstraint()
{
    this->task->activateAsConstraint();
}

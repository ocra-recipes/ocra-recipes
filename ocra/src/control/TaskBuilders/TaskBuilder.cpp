#include <ocra/control/TaskBuilders/TaskBuilder.h>

using namespace ocra;


TaskBuilder::TaskBuilder(const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr)
: options(taskOptions)
, model(modelPtr)
{

}

TaskBuilder::~TaskBuilder()
{

}

void TaskBuilder::buildTask()
{
    Feature::Ptr feature = buildFeature();
    Feature::Ptr featureDes = buildFeatureDesired();
    this->task = std::make_shared<Task>(this->options.taskName, this->model, feature, featureDes);
    setTaskType();
}

Task::Ptr TaskBuilder::getTask()
{
    return this->task;
}

void TaskBuilder::setTaskParameters()
{
    // Generic to all tasks
    setTaskAsObjective();
    setTaskLevel();
    setTaskWeight();
    setTaskStiffness();
    setTaskDamping();

    // Specific to the type of task
    setTaskState();
}

void TaskBuilder::setTaskAsObjective()
{
    this->task->activateAsObjective();
}

void TaskBuilder::setTaskLevel()
{
    this->task->setHierarchyLevel(this->options.hierarchyLevel);
}

void TaskBuilder::setTaskWeight()
{
    if (this->options.useWeightVectorConstructor) {
        this->task->setWeight(this->options.weightVector);
    } else {
        this->task->setWeight(this->options.weight);
    }
}

void TaskBuilder::setTaskStiffness()
{
    this->task->setStiffness(this->options.kp);
}

void TaskBuilder::setTaskDamping()
{
    this->task->setDamping(this->options.kd);
}

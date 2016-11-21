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
    if (featureDes && feature) {
        this->task = std::make_shared<Task>(this->options.taskName, this->model, feature, featureDes);
    } else {
            if (feature)
                this->task = std::make_shared<Task>(this->options.taskName, this->model, feature);
            else {
                std::cout << "[ERROR] TaskBuilder::buildTask() : Neither feature nor featureDes were created" << std::endl;
            }
    } 
    setTaskType();
}

Task::Ptr TaskBuilder::getTask()
{
    return this->task;
}

void TaskBuilder::setTaskParameters()
{
    // Generic to all tasks
    setTaskAsObjectiveOrConstraint();
    setTaskLevel();
    setTaskWeight();
    setTaskStiffness();
    setTaskDamping();

    // Specific to the type of task
    setTaskState();
}

void TaskBuilder::setTaskAsObjectiveOrConstraint()
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

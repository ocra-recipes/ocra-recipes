#include <ocra/control/TaskBuilders/TaskBuilder.h>

using namespace ocra;


TaskBuilder::TaskBuilder(const TaskBuilderOptions& options, Model::Ptr model)
: _options(options)
, _model(model)
{

}

TaskBuilder::~TaskBuilder()
{

}

void TaskBuilder::buildTask()
{
    Feature* feature = buildFeature();
    Feature* featureDes = buildFeatureDesired();
    _task = std::make_shared<Task>(_options.taskName, _model, *feature, *featureDes);
    setTaskType();
}

Task::Ptr TaskBuilder::getTask()
{
    return _task;
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
    _task->activateAsObjective();
}

void TaskBuilder::setTaskLevel()
{
    _task->setHierarchyLevel(_options.hierarchyLevel);
}

void TaskBuilder::setTaskWeight()
{
    if (_options.useWeightVectorConstructor) {
        _task->setWeight(_options.weightVector);
    } else {
        _task->setWeight(_options.weight);
    }
}

void TaskBuilder::setTaskStiffness()
{
    _task->setStiffness(_options.kp);
}

void TaskBuilder::setTaskDamping()
{
    _task->setDamping(_options.kd);
}

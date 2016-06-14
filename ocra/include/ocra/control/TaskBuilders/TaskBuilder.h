#ifndef TASK_BUILDER_H
#define TASK_BUILDER_H

#include <ocra/control/TaskBuilders/TaskBuilderOptions.h>
#include <ocra/control/Model.h>
#include <ocra/control/Task.h>
#include <ocra/control/Feature.h>
#include <ocra/control/ControlFrame.h>
#include <ocra/utilities.h>

namespace ocra {

class TaskBuilder {
DEFINE_CLASS_POINTER_TYPEDEFS(TaskBuilder)

protected:
    Task::Ptr _task;
    Model::Ptr _model;
    TaskBuilderOptions _options;

protected: // pure virtual functions
    virtual void setTaskType() = 0;
    virtual void setTaskState() = 0;
    virtual Feature * buildFeature() = 0;
    virtual Feature * buildFeatureDesired() = 0;


public:
    TaskBuilder (const TaskBuilderOptions& options, Model::Ptr model);
    virtual ~TaskBuilder ();

    void buildTask();
    Task::Ptr getTask();
    void setTaskParameters();


private:
    void setTaskAsObjective();
    void setTaskLevel();
    void setTaskWeight();
    void setTaskStiffness();
    void setTaskDamping();



};


} // namespace ocra

#endif //TASK_BUILDER_H

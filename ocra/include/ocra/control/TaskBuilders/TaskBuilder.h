#ifndef TASK_BUILDER_H
#define TASK_BUILDER_H

#include <ocra/control/TaskBuilders/TaskBuilderOptions.h>
#include <ocra/control/Model.h>
#include <ocra/control/Task.h>
#include <ocra/control/TaskState.h>
#include <ocra/control/Feature.h>
#include <ocra/control/ControlFrame.h>
#include <ocra/util/Macros.h>
#include <ocra/util/EigenUtilities.h>

namespace ocra {

class TaskBuilder {
DEFINE_CLASS_POINTER_TYPEDEFS(TaskBuilder)

protected:
    Task::Ptr task;
    Model::Ptr model;
    TaskBuilderOptions options;

protected: // pure virtual functions
    virtual void setTaskType() = 0;
    virtual void setTaskState() = 0;
    virtual Feature::Ptr buildFeature() = 0;
    virtual Feature::Ptr buildFeatureDesired() = 0;

    /**
     * This function is overloaded for cases where the task should be treated as a constraint and not as an objective.
     * If not overloaded the base class implementation will be called and set the task to an objective.
     */
    virtual void setTaskAsObjectiveOrConstraint();

public:
    TaskBuilder (const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr);
    virtual ~TaskBuilder ();

    /**
     *   Defines a feature and a desired feature of type ocra::Feature and consequently instantiates the corresponding task. The way a feature is built depends on the implementation (e.g. FullPostureTaskBuilder::buildFeature) written for each task-specific TaskBuilder.
     */
    void buildTask();
    
    /**
     *  Returns a pointer to the task associated to the TaskBuilder.
     *
     *  @return Pointer to the task contained in the TaskBuilder object.
     */
    Task::Ptr getTask();
    
    /**
     *  First sets a specific task as objective or constraint, its task level, weight, stiffness, damping and the task state (which is specific to the type of task. This information comes naturally from the task builder options passed during the construction of a TaskBuilder.
     */
    void setTaskParameters();


private:
    /**
     *  Sets the task's hierarchy level as found in the TaskBuilderOptions object of this class.
     */
    void setTaskLevel();
    
    /**
     *  Sets the task weights (single or vector of weights).
     */
    void setTaskWeight();
    
    /**
     *  Sets the stiffness of as specified in the TaskBuilderOptions object of this class.
     */
    void setTaskStiffness();
    
    /**
     *  Sets the damping value specified in the TaskBuilderOptions object of this class.
     */
    void setTaskDamping();



};


} // namespace ocra

#endif //TASK_BUILDER_H

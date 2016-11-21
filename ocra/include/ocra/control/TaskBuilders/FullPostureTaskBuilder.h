#ifndef OCRA_CONTROL_FULL_POSTURE_TASK_BUILDER_H
#define OCRA_CONTROL_FULL_POSTURE_TASK_BUILDER_H

#include <ocra/control/TaskBuilders/TaskBuilder.h>

namespace ocra {

class FullPostureTaskBuilder : public TaskBuilder {
DEFINE_CLASS_POINTER_TYPEDEFS(FullPostureTaskBuilder)

public:
    FullPostureTaskBuilder (const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr);
    virtual ~FullPostureTaskBuilder ();

private:
    int fullStateType;

protected: // pure virtual functions
    virtual void setTaskType();
    virtual void setTaskState();
    virtual Feature::Ptr buildFeature();
    virtual Feature::Ptr buildFeatureDesired();

};

} // namespace ocra

#endif //OCRA_CONTROL_FULL_POSTURE_TASK_BUILDER_H

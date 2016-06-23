#ifndef OCRA_CONTROL_PARTIAL_POSTURE_TASK_BUILDER_H
#define OCRA_CONTROL_PARTIAL_POSTURE_TASK_BUILDER_H

#include <ocra/control/TaskBuilders/TaskBuilder.h>

namespace ocra {

class PartialPostureTaskBuilder : public TaskBuilder {
DEFINE_CLASS_POINTER_TYPEDEFS(PartialPostureTaskBuilder)

public:
    PartialPostureTaskBuilder (const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr);
    virtual ~PartialPostureTaskBuilder ();

private:
    int partialStateType;

protected: // pure virtual functions
    virtual void setTaskType();
    virtual void setTaskState();
    virtual Feature::Ptr buildFeature();
    virtual Feature::Ptr buildFeatureDesired();

};

} // namespace ocra

#endif //OCRA_CONTROL_PARTIAL_POSTURE_TASK_BUILDER_H

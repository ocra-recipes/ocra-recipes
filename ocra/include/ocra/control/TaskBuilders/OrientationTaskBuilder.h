#ifndef OCRA_CONTROL_ORIENTATION_TASK_BUILDER_H
#define OCRA_CONTROL_ORIENTATION_TASK_BUILDER_H

#include <ocra/control/TaskBuilders/TaskBuilder.h>

namespace ocra {

class OrientationTaskBuilder : public TaskBuilder {
DEFINE_CLASS_POINTER_TYPEDEFS(OrientationTaskBuilder)

public:
    OrientationTaskBuilder (const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr);
    virtual ~OrientationTaskBuilder ();

private:
    const int ORIENTATION_VECTOR_SIZE = 4;

protected: // pure virtual functions
    virtual void setTaskType();
    virtual void setTaskState();
    virtual Feature::Ptr buildFeature();
    virtual Feature::Ptr buildFeatureDesired();

};

} // namespace ocra

#endif //OCRA_CONTROL_ORIENTATION_TASK_BUILDER_H

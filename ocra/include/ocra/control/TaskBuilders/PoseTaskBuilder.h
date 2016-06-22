#ifndef OCRA_CONTROL_POSE_TASK_BUILDER_H
#define OCRA_CONTROL_POSE_TASK_BUILDER_H

#include <ocra/control/TaskBuilders/TaskBuilder.h>
#include <ocra/util/EigenUtilities.h>

namespace ocra {

class PoseTaskBuilder : public TaskBuilder {
DEFINE_CLASS_POINTER_TYPEDEFS(PoseTaskBuilder)

public:
    PoseTaskBuilder (const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr);
    virtual ~PoseTaskBuilder ();

private:
    int nDoF;
    const int DISPLACEMENT_VECTOR_SIZE = 7;

protected: // pure virtual functions
    virtual void setTaskType();
    virtual void setTaskState();
    virtual Feature::Ptr buildFeature();
    virtual Feature::Ptr buildFeatureDesired();

};

} // namespace ocra

#endif //OCRA_CONTROL_POSE_TASK_BUILDER_H

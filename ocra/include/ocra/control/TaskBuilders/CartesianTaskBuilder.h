#ifndef CARTESIAN_TASK_BUILDER_H
#define CARTESIAN_TASK_BUILDER_H

#include <ocra/control/TaskBuilders/TaskBuilder.h>

namespace ocra {

class CartesianTaskBuilder : public TaskBuilder {
DEFINE_CLASS_POINTER_TYPEDEFS(CartesianTaskBuilder)

public:
    CartesianTaskBuilder (const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr);
    virtual ~CartesianTaskBuilder ();

private:
    ECartesianDof axes;
    int nDoF;

protected: // pure virtual functions
    virtual void setTaskType();
    virtual void setTaskState();
    virtual Feature::Ptr buildFeature();
    virtual Feature::Ptr buildFeatureDesired();

};

} // namespace ocra

#endif //CARTESIAN_TASK_BUILDER_H

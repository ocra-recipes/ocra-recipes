#ifndef CARTESIAN_TASK_BUILDER_H
#define CARTESIAN_TASK_BUILDER_H

#include <ocra/control/TaskBuilders/TaskBuilder.h>

namespace ocra {

class CartesianTaskBuilder : public TaskBuilder {
DEFINE_CLASS_POINTER_TYPEDEFS(CartesianTaskBuilder)

public:
    CartesianTaskBuilder (const TaskBuilderOptions& options, Model::Ptr model);
    virtual ~CartesianTaskBuilder ();

private:
    ECartesianDof _axes;
    int _nDoF;

protected: // pure virtual functions
    virtual void setTaskType();
    virtual void setTaskState();
    virtual Feature * buildFeature();
    virtual Feature * buildFeatureDesired();

};

} // namespace ocra

#endif //CARTESIAN_TASK_BUILDER_H

#ifndef OCRA_CONTROL_COM_TASK_BUILDER_H
#define OCRA_CONTROL_COM_TASK_BUILDER_H

#include <ocra/control/TaskBuilders/TaskBuilder.h>

namespace ocra {

class ComTaskBuilder : public TaskBuilder {
DEFINE_CLASS_POINTER_TYPEDEFS(ComTaskBuilder)

public:
    ComTaskBuilder (const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr);
    virtual ~ComTaskBuilder ();

private:
    int nDoF;

protected: // pure virtual functions
    virtual void setTaskType();
    virtual void setTaskState();
    virtual Feature::Ptr buildFeature();
    virtual Feature::Ptr buildFeatureDesired();

};

} // namespace ocra

#endif //OCRA_CONTROL_COM_TASK_BUILDER_H

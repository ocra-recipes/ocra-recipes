#ifndef OCRA_CONTROL_POINT_CONTACT_TASK_BUILDER_H
#define OCRA_CONTROL_POINT_CONTACT_TASK_BUILDER_H

#include <ocra/control/TaskBuilders/TaskBuilder.h>

namespace ocra {

class PointContactTaskBuilder : public TaskBuilder {
DEFINE_CLASS_POINTER_TYPEDEFS(PointContactTaskBuilder)

public:
    PointContactTaskBuilder (const TaskBuilderOptions& taskOptions, Model::Ptr modelPtr);
    virtual ~PointContactTaskBuilder ();

private:

protected: // pure virtual functions
    virtual void setTaskType();
    virtual void setTaskState();
    virtual Feature::Ptr buildFeature();
    virtual Feature::Ptr buildFeatureDesired();

    virtual void setTaskAsObjectiveOrConstraint();


};

} // namespace ocra

#endif //OCRA_CONTROL_POINT_CONTACT_TASK_BUILDER_H

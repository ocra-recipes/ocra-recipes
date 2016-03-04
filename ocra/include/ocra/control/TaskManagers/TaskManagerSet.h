#ifndef TASK_MANAGER_SET_H
#define TASK_MANAGER_SET_H

#include <cmath>
#include <memory>

// Includes the set of all possible task managers to make it easier to reference
#include "ocra/control/TaskManagers/TaskManager.h"
#include "ocra/control/TaskManagers/CoMTaskManager.h"
#include "ocra/control/TaskManagers/CoMMomentumTaskManager.h"
#include "ocra/control/TaskManagers/ContactTaskManager.h"
#include "ocra/control/TaskManagers/ContactSetTaskManager.h"
#include "ocra/control/TaskManagers/FullPostureTaskManager.h"
#include "ocra/control/TaskManagers/PartialPostureTaskManager.h"
#include "ocra/control/TaskManagers/SegCartesianTaskManager.h"
#include "ocra/control/TaskManagers/SegOrientationTaskManager.h"
#include "ocra/control/TaskManagers/SegPoseTaskManager.h"
#include "ocra/control/TaskManagers/VariableWeightsTaskManager.h"

#include "ocra/control/Model.h"
#include "ocra/control/Controller.h"


namespace ocra
{
typedef std::map<std::string, std::shared_ptr<TaskManager> > TaskManagerDict;
typedef std::map<std::string, std::shared_ptr<TaskManager> >::iterator tmIterator;

class TaskManagerSet
{
public:
    TaskManagerSet(std::shared_ptr<Controller> ctrlPtr, std::shared_ptr<Model> modelPtr);

    virtual ~TaskManagerSet();

    bool addTaskManager(const std::string& keyValue, std::shared_ptr<TaskManager> newTaskManager);
    bool removeTaskManager(const std::string& keyValue);
    bool clearSequence();
    std::vector<std::string> getTaskList();
    std::vector<std::string> getTaskPorts();

    std::shared_ptr<TaskManager> getTaskManagerPointer(const std::string& taskName);

protected:
    std::shared_ptr<Controller> ctrl;
    std::shared_ptr<Model> model;
    TaskManagerDict taskManagers;
};

} // namespace ocra

#endif // TASK_MANAGER_SET_H

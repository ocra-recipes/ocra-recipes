#ifndef TASKSEQUENCEBASE_H
#define TASKSEQUENCEBASE_H

#include <cmath>

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
    typedef std::map<std::string, TaskManager*> TaskManagerDict;
    typedef std::map<std::string, TaskManager*>::iterator tmIterator;

    class TaskSequence
    {
        public:
            virtual ~TaskSequence();


            void init(ocra::Controller& ctrl, ocra::Model& model);
            void update(double time, ocra::Model& state, void** args);

            bool addTaskManager(std::string keyValue, TaskManager* newTaskManager);
            bool removeTaskManager(std::string keyValue);
            bool clearSequence();
            std::vector<std::string> getTaskList();
            std::vector<std::string> getTaskPorts();

            TaskManager* getTaskManagerPointer(std::string taskName);



        protected:
            virtual void doInit(ocra::Controller& ctrl, ocra::Model& model) = 0;
            virtual void doUpdate(double time, ocra::Model& state, void** args) = 0;

            TaskManagerDict taskManagers;



    };
}

#endif // TASKSEQUENCE_H

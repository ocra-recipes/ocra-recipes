#ifndef wOcraTASKSEQUENCEBASE_H
#define wOcraTASKSEQUENCEBASE_H

// Includes the set of all possible task managers to make it easier to reference
#include "wocra/Tasks/wOcraTaskManagerBase.h"
#include "wocra/Tasks/Managers/wOcraCoMTaskManager.h"
#include "wocra/Tasks/Managers/wOcraCoMMomentumTaskManager.h"
#include "wocra/Tasks/Managers/wOcraContactTaskManager.h"
#include "wocra/Tasks/Managers/wOcraContactSetTaskManager.h"
#include "wocra/Tasks/Managers/wOcraFullPostureTaskManager.h"
#include "wocra/Tasks/Managers/wOcraPartialPostureTaskManager.h"
#include "wocra/Tasks/Managers/wOcraSegCartesianTaskManager.h"
#include "wocra/Tasks/Managers/wOcraSegOrientationTaskManager.h"
#include "wocra/Tasks/Managers/wOcraSegPoseTaskManager.h"
#include "wocra/Tasks/Managers/wOcraVariableWeightsTaskManager.h"

#include "wocra/Models/wOcraModel.h"


namespace wocra
{
    typedef std::map<std::string, wOcraTaskManagerBase*> TaskManagerDict;
    typedef std::map<std::string, wOcraTaskManagerBase*>::iterator tmIterator;

    class wOcraTaskSequenceBase
    {
        public:
            virtual ~wOcraTaskSequenceBase();


            void init(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
            void update(double time, wocra::wOcraModel& state, void** args);

            bool addTaskManager(std::string keyValue, wOcraTaskManagerBase* newTaskManager);
            bool removeTaskManager(std::string keyValue);
            bool clearSequence();
            std::vector<std::string> getTaskList();
            std::vector<std::string> getTaskPorts();




        protected:
            virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model) = 0;
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args) = 0;

            TaskManagerDict taskManagers;



    };
}

#endif // wOcraTASKSEQUENCE_H

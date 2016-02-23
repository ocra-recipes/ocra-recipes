#ifndef gOcraTASKMANAGERCOLLECTIONBASE_H
#define gOcraTASKMANAGERCOLLECTIONBASE_H

// Includes the set of all possible task managers to make it easier to reference
#include "gocra/Tasks/gOcraTaskManagerBase.h"
#include "gocra/Tasks/Managers/gOcraCoMTaskManager.h"
#include "gocra/Tasks/Managers/gOcraContactTaskManager.h"
#include "gocra/Tasks/Managers/gOcraContactSetTaskManager.h"
#include "gocra/Tasks/Managers/gOcraFullPostureTaskManager.h"
#include "gocra/Tasks/Managers/gOcraPartialPostureTaskManager.h"
#include "gocra/Tasks/Managers/gOcraSegCartesianTaskManager.h"
#include "gocra/Tasks/Managers/gOcraSegOrientationTaskManager.h"
#include "gocra/Tasks/Managers/gOcraSegPoseTaskManager.h"
//#include "gocra/Tasks/Managers/gOcraVariableWeightsTaskManager.h"

#include "gocra/Models/gOcraModel.h"

namespace gocra
{
    typedef std::map<std::string, gOcraTaskManagerBase*> TaskManagerDict;

    class gOcraTaskManagerCollectionBase
    {
        public:
            virtual ~gOcraTaskManagerCollectionBase();
            void init(gocra::GHCJTController& ctrl, gocra::gOcraModel& model);
            void update(double time, gocra::gOcraModel& state, void** args);
        protected: 
            virtual void doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model) = 0;
            virtual void doUpdate(double time, gocra::gOcraModel& state, void** args) = 0;

            TaskManagerDict taskManagers;
    };
}

#endif // gOcraTASKMANAGERCOLLECTION_H

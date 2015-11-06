#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "wocra/wOcraController.h"
#include "wocra/Model.h"

#include <iterator>

namespace wocra
{
    wOcraTaskSequenceBase::~wOcraTaskSequenceBase()
    {
    }

    void wOcraTaskSequenceBase::init(wOcraController& ctrl, wocra::wOcraModel& model)
    {
        doInit(ctrl, model);
    }

    void wOcraTaskSequenceBase::update(double time, wocra::wOcraModel& state, void** args)
    {
        for (tmIterator it = taskManagers.begin(); it != taskManagers.end(); it++)
        {
            if(it->second->isFollowingTrajectory())
            {
                it->second->updateTrajectory(time);
            }
        }

        // Run the custom doUpdate function of the cpp Sequences for hard coded control logic:
        doUpdate(time, state, args);
    }

    bool wOcraTaskSequenceBase::addTaskManager(std::string keyValue, wOcraTaskManagerBase* newTaskManager)
    {
        if (newTaskManager==NULL) {
            std::cout << "[WARNING] (wOcraTaskSequenceBase::addTaskManager): The newTaskManager pointer you passed was empty." << std::endl;
            return false;
        }

        //Check if key already exists... If not, .find() will return a iterator to the end of the map.
        if (taskManagers.find(keyValue) == taskManagers.end()) {
            taskManagers[keyValue] = newTaskManager;
            return true;
        }
        else{
            std::cout << "[WARNING] (wOcraTaskSequenceBase::addTaskManager): The key value you passed already exists. Cannot overwrite tasks, please remove tasks before replacing them." << std::endl;
            return false;
        }
    }

    bool wOcraTaskSequenceBase::removeTaskManager(std::string keyValue)
    {
        if (taskManagers.find(keyValue) != taskManagers.end()) {
            //TODO: Deactivate tasks smoothly?
            taskManagers[keyValue]->deactivate();
            delete(taskManagers[keyValue]);
            taskManagers.erase(keyValue);
            return true;
        }
        else{
            std::cout << "[ERROR] (wOcraTaskSequenceBase::removeTaskManager): The task key you passed does not exist." << std::endl;
            return false;
        }
    }

    bool wOcraTaskSequenceBase::clearSequence()
    {
        std::cout << "\n=== Deactivating tasks ===" << std::endl;
        for (tmIterator it = taskManagers.begin(); it != taskManagers.end(); it++)
        {
            std::cout << " --> " << it->first << std::endl;
            it->second->deactivate();
            delete(it->second);
        }

        std::cout << "\n Clearing task sequence...\n" << std::endl;
        taskManagers.clear();
        return true;

    }

    std::vector<std::string> wOcraTaskSequenceBase::getTaskList()
    {
        std::vector<std::string> strVector(taskManagers.size());
        int i = 0;
        for (tmIterator it = taskManagers.begin(); it != taskManagers.end(); it++)
        {
            strVector[i] = it->first;
            i++;
        }
        return strVector;
    }

    std::vector<std::string> wOcraTaskSequenceBase::getTaskPorts()
    {
        std::vector<std::string> strVector(taskManagers.size());
        int i = 0;
        for (tmIterator it = taskManagers.begin(); it != taskManagers.end(); it++)
        {
            strVector[i] = it->second->getPortName();
            i++;
        }
        return strVector;
    }

}

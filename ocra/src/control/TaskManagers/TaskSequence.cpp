#include "ocra/control/TaskManagers/TaskSequence.h"
// #include "wocra/Controller.h"
// #include "wocra/Model.h"

#include <iterator>

namespace ocra
{
    TaskSequence::~TaskSequence()
    {
    }

    void TaskSequence::init(ocra::Controller& ctrl, ocra::Model& model)
    {
        doInit(ctrl, model);
    }

    void TaskSequence::update(double time, ocra::Model& state, void** args)
    {
        // for (tmIterator it = taskManagers.begin(); it != taskManagers.end(); it++)
        // {
        //     if(it->second->isFollowingTrajectory())
        //     {
        //         it->second->updateTrajectory(time);
        //     }
        // }

        // Run the custom doUpdate function of the cpp Sequences for hard coded control logic:
        doUpdate(time, state, args);
    }

    bool TaskSequence::addTaskManager(std::string keyValue, std::shared_ptr<TaskManager> newTaskManager)
    {
        if (newTaskManager==NULL) {
            std::cout << "[WARNING] (TaskSequence::addTaskManager): The newTaskManager pointer you passed was empty." << std::endl;
            return false;
        }

        //Check if key already exists... If not, .find() will return a iterator to the end of the map.
        if (taskManagers.find(keyValue) == taskManagers.end()) {
            taskManagers[keyValue] = newTaskManager;
            return true;
        }
        else{
            std::cout << "[WARNING] (TaskSequence::addTaskManager): The key value you passed already exists. Cannot overwrite tasks, please remove tasks before replacing them." << std::endl;
            return false;
        }
    }

    bool TaskSequence::removeTaskManager(std::string keyValue)
    {
        if (taskManagers.find(keyValue) != taskManagers.end()) {
            //TODO: Deactivate tasks smoothly?
            taskManagers[keyValue]->deactivate();
            // (std::dynamic_pointer_cast<ocra::OneLevelTask>(taskManagers[keyValue]))->disconnectFromController();
            // delete(taskManagers[keyValue]);
            taskManagers.erase(keyValue);
            return true;
        }
        else{
            std::cout << "[ERROR] (TaskSequence::removeTaskManager): The task key you passed does not exist." << std::endl;
            return false;
        }
    }

    bool TaskSequence::clearSequence()
    {
        std::cout << "\n=== Deactivating tasks ===" << std::endl;
        for (tmIterator it = taskManagers.begin(); it != taskManagers.end(); it++)
        {
            std::cout << " --> " << it->first << std::endl;
            it->second->deactivate();
            // (std::dynamic_pointer_cast<ocra::OneLevelTask>(it->second))->disconnectFromController();

            // delete(it->second);
        }

        std::cout << "\n Clearing task sequence...\n" << std::endl;
        taskManagers.clear();
        return true;

    }

    std::vector<std::string> TaskSequence::getTaskList()
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

    std::vector<std::string> TaskSequence::getTaskPorts()
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

    std::shared_ptr<TaskManager> TaskSequence::getTaskManagerPointer(std::string taskName)
    {
        if (taskManagers.find(taskName) != taskManagers.end()) {
            return taskManagers[taskName];
        }
        else{
            std::cout << "[WARNING] (TaskSequence::getTaskManagerPointer): Could not find, " << taskName << ", in the list of tasks." << std::endl;
            return NULL;
        }
    }

}

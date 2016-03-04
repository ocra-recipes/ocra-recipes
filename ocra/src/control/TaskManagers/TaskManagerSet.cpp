#include "ocra/control/TaskManagers/TaskManagerSet.h"

#include <iterator>

namespace ocra
{
TaskManagerSet::TaskManagerSet(std::shared_ptr<Controller> ctrlPtr, std::shared_ptr<Model> modelPtr)
{
    ctrl = ctrlPtr;
    model = modelPtr;
}

TaskManagerSet::~TaskManagerSet()
{
}

bool TaskManagerSet::addTaskManager(const std::string& keyValue, std::shared_ptr<TaskManager> newTaskManager)
{
    if (newTaskManager==NULL) {
        std::cout << "[WARNING] (TaskManagerSet::addTaskManager): The newTaskManager pointer you passed was empty." << std::endl;
        return false;
    }

    //Check if key already exists... If not, .find() will return a iterator to the end of the map.
    if (taskManagers.find(keyValue) == taskManagers.end()) {
        taskManagers[keyValue] = newTaskManager;
        return true;
    }
    else{
        std::cout << "[WARNING] (TaskManagerSet::addTaskManager): The key value you passed already exists. Cannot overwrite tasks, please remove tasks before replacing them." << std::endl;
        return false;
    }
}

bool TaskManagerSet::removeTaskManager(const std::string& keyValue)
{
    if (taskManagers.find(keyValue) != taskManagers.end()) {
        //TODO: Deactivate tasks smoothly?
        // Deactivate the task first.
        taskManagers[keyValue]->deactivate();
        // If it is in the controller then remove it.
        if(ctrl->getTasks().find(keyValue) != ctrl->getTasks().end()){
            ctrl->removeTask(keyValue);
        }

        // (std::dynamic_pointer_cast<ocra::OneLevelTask>(taskManagers[keyValue]))->disconnectFromController();
        // delete(taskManagers[keyValue]);

        // Remove the manager from the set.
        taskManagers.erase(keyValue);
        return true;
    }
    else{
        std::cout << "[ERROR] (TaskManagerSet::removeTaskManager): The task key you passed does not exist." << std::endl;
        return false;
    }
}

bool TaskManagerSet::clearSet()
{
    std::cout << "\n=== Deactivating tasks ===" << std::endl;
    for (tmIterator it = taskManagers.begin(); it != taskManagers.end(); ++it)
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

std::vector<std::string> TaskManagerSet::getTaskList()
{
    std::vector<std::string> strVector(taskManagers.size());
    int i = 0;
    for (tmIterator it = taskManagers.begin(); it != taskManagers.end(); ++it)
    {
        strVector[i] = it->first;
        ++i;
    }
    return strVector;
}

std::vector<std::string> TaskManagerSet::getTaskPorts()
{
    std::vector<std::string> strVector(taskManagers.size());
    int i = 0;
    for (tmIterator it = taskManagers.begin(); it != taskManagers.end(); ++it)
    {
        strVector[i] = it->second->getPortName();
        ++i;
    }
    return strVector;
}

std::shared_ptr<TaskManager> TaskManagerSet::getTaskManagerPointer(const std::string& taskName)
{
    if (taskManagers.find(taskName) != taskManagers.end()) {
        return taskManagers[taskName];
    }
    else{
        std::cout << "[WARNING] (TaskManagerSet::getTaskManagerPointer): Could not find, " << taskName << ", in the list of tasks." << std::endl;
        return NULL;
    }
}

}

#ifndef TASK_MANAGER_FACTORY_H
#define TASK_MANAGER_FACTORY_H

#include <iostream>
#include <tinyxml.h>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <boost/filesystem.hpp>

#include "ocra/control/Model.h"
#include "ocra/control/Controller.h"
#include "ocra/control/TaskManagers/TaskManager.h"
#include "ocra/control/TaskManagers/TaskSequence.h"
#include "ocra/control/TaskManagers/TaskManagerSet.h"
#include "ocra/control/TaskManagers/TaskManagerOptions.h"

#include <Eigen/Dense>

#include <yarp/os/Bottle.h>

namespace ocra
{

class TaskManagerFactory
{
public:
    TaskManagerFactory();
    ~TaskManagerFactory();

    Eigen::VectorXd stringToVectorXd(const char * valueString);
    Eigen::VectorXi stringToVectorXi(const char * valueString);
    bool parseTasksXML(const std::string& filePath);
    bool parseTasksXML(const char * filePath);
    bool parseTasksXML(TiXmlDocument* newTasksFile);
    void printTaskArguments();
    bool addTaskManagerOptions(TaskManagerOptions& tmOpts);

    bool addTaskManagersToSet(std::shared_ptr<ocra::Controller> ctrl, std::shared_ptr<ocra::Model> model, std::shared_ptr<TaskManagerSet> taskSet);

    // bool parseTasksYarp(yarp::os::Bottle* yarpMessage);
    // bool xmlToYarp(const char* filePath, yarp::os::Bottle* yarpMessage);
    // bool yarpToXML(yarp::os::Bottle* yarpMessage, char* filePath);




private:
    std::vector<TaskManagerOptions> tmOptsVector;
    using tmOptsIterator = std::vector<TaskManagerOptions>::iterator;

    const char * getDisplacementArgs(TiXmlElement* xmlElem);

    std::shared_ptr<TaskManager> constructTaskManager(std::shared_ptr<ocra::Controller> ctrl, std::shared_ptr<ocra::Model> model, tmOptsIterator tmOptsPtr);

    Eigen::Displacementd eigenVectorToDisplacementd(Eigen::VectorXd& eigenVector);

    std::vector<Eigen::Displacementd> eigenVectorToDisplacementd(std::vector<Eigen::VectorXd>& eigenVector);

    void prepareTaskManagerArguments(std::shared_ptr<ocra::Model> model, tmOptsIterator tmOptsPtr);
};

} // namespace ocra
#endif // TASK_MANAGER_FACTORY_H
